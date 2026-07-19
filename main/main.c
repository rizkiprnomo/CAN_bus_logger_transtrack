#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <sys/stat.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_timer.h"

// ========================== PIN CONFIGURATION ==========================

#define SDMMC_CLK_PIN   GPIO_NUM_36
#define SDMMC_CMD_PIN   GPIO_NUM_35
#define SDMMC_D0_PIN    GPIO_NUM_37
#define SDMMC_D1_PIN    GPIO_NUM_38
#define SDMMC_D2_PIN    GPIO_NUM_39
#define SDMMC_D3_PIN    GPIO_NUM_40

#define TWAI_TX_PIN     GPIO_NUM_4
#define TWAI_RX_PIN     GPIO_NUM_5

#define USER_BUTTON_PIN GPIO_NUM_1
#define STATUS_LED_PIN  GPIO_NUM_6

#define UART_NUM        UART_NUM_0
#define UART_BUF_SIZE   1024

#define MOUNT_POINT     "/sdcard"
#define BASE_LOG_PATH   "/sdcard/can_%03d.csv"

#define CIRCULAR_BUFFER_SIZE    256     //(power of 2 recommended)

// ========================== GLOBAL VARIABLES ==========================

sdmmc_card_t *sd_card = NULL;
static const char *TAG = "CAN_LOGGER";

twai_node_handle_t twai_node = NULL;

typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
    bool extd;
    bool rtr;
    uint32_t tstamp;
} can_flat_frame_t;

typedef struct {
    uint64_t timestamp_us;
    can_flat_frame_t msg;
} can_log_frame_t;

//Queus
static QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t can_rx_queue; 

// Task handles
TaskHandle_t can_rx_task_handle = NULL;

// Statistics (atomic for thread safety)
atomic_uint_fast32_t frames_received = 0;
atomic_uint_fast32_t frames_dropped = 0;
atomic_uint_fast32_t write_errors = 0;

// System state
atomic_bool logging_active = false; 
atomic_int current_file_index = 1;
atomic_bool force_file_rotate = false;
atomic_bool sd_card_not_found = false; 
atomic_bool bus_off_detected = false;  

// LED status
typedef enum {
    LED_STATUS_STARTUP,    
    LED_STATUS_SD_FAIL,    
    LED_STATUS_IDLE,       
    LED_STATUS_LOGGING,    
    LED_STATUS_BUS_OFF,    
    LED_STATUS_ERROR       
} led_status_t;

volatile led_status_t current_led_status = LED_STATUS_LOGGING;

static SemaphoreHandle_t log_file_mutex = NULL;

//Log file struct
typedef struct {
    FILE* fp;
    char current_path[64];
    uint32_t frames_written;
    int64_t last_flush_time;
    bool is_open;
} log_file_state_t;

static log_file_state_t log_state = { .fp = NULL, .is_open = false };

// Circular buffer struct
typedef struct {
    can_log_frame_t buffer[CIRCULAR_BUFFER_SIZE];
    atomic_uint head;
    atomic_uint tail;
    atomic_uint count;   
} circular_log_buffer_t;

static circular_log_buffer_t log_buffer = {0};

// ========================== FUNCTION PROTOTYPES ==========================
void decode_j1939_id(uint32_t id);
void init_twai(void);
void init_sdmmc(void);
void init_ui_and_uart(void);
int scan_next_available_index(void);
static bool open_log_file(void);
static void close_current_log_file(void);
static void flush_log_file(void);
static bool write_can_frame_to_csv(const can_log_frame_t *frame);
static bool circular_buffer_push(const can_log_frame_t *frame);

// ========================== ISR & CALLBACKS ==========================
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static bool IRAM_ATTR twai_listener_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "bus error: 0x%x", edata->err_flags.val);
    return false;
}

static bool IRAM_ATTR twai_listener_on_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    const char *twai_state_name[] = {"active", "warning", "passive", "bus_off"};
    
    ESP_EARLY_LOGI(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);

    const char *current_status = twai_state_name[edata->new_sta];

    if (strcmp(current_status, "bus_off") == 0) {
        if (can_rx_task_handle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(can_rx_task_handle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
            return (xHigherPriorityTaskWoken == pdTRUE);
        }
    } 
    else if (strcmp(current_status, "active") == 0) {
        ESP_EARLY_LOGI(TAG, "The TWAI driver successfully recovered and is back ONLINE (Active).");
    }
    
    return false;
}

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx) 
{
    uint8_t recv_buff[8] = {0};
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        if (can_rx_queue != NULL) {
            can_flat_frame_t flat_frame = {
                .identifier = rx_frame.header.id,
                .data_length_code = rx_frame.header.dlc,
                .extd = rx_frame.header.ide,
                .rtr = rx_frame.header.rtr,
                .tstamp = rx_frame.header.timestamp,
            };
            memcpy(flat_frame.data, recv_buff, rx_frame.header.dlc);
            xQueueSendFromISR(can_rx_queue, &flat_frame, &xHigherPriorityTaskWoken);
        }
    }
    return (xHigherPriorityTaskWoken == pdTRUE);
}

// ========================== J1939 HELPER ==========================
void decode_j1939_id(uint32_t id) {
    
    uint8_t priority = (id >> 26) & 0x07;
    uint32_t pgn = (id >> 8) & 0x3FFFF;
    uint8_t source_address = id & 0xFF;

    printf("--- J1939 Frame Decode ---\n");
    printf("Raw ID : 0x%08lX\n", id);
    printf("Priority : %u\n", priority);
    printf("PGN      : 0x%05lX (%lu)\n", pgn, pgn);
    printf("Source Addr: 0x%02X (%u)\n", source_address, source_address);
    printf("--------------------------\n");
}

// ========================== CAN RX TASK ==========================
void can_rx_task(void *pvParameters) {
    can_flat_frame_t rx_frame;
    can_log_frame_t log_frame;
    uint32_t notification_value = 0;

    can_rx_task_handle = xTaskGetCurrentTaskHandle();

    while (1) {
        // Check bus-off notification (non-blocking)
        xTaskNotifyWait(0, ULONG_MAX, &notification_value, 0);

        if (notification_value & 0x01) {
            ESP_LOGW("CAN_TASK", "Bus-Off detected, attempting recovery...");
            if (twai_node_recover(twai_node) == ESP_OK) {
                atomic_store(&bus_off_detected, false);
                ESP_LOGI("CAN_TASK", "TWAI recovered successfully");
            }
            notification_value = 0;
        }

        if (xQueueReceive(can_rx_queue, &rx_frame, portMAX_DELAY) == pdTRUE) {
            if (!atomic_load(&logging_active)) continue;

            //decode_j1939_id(rx_frame.identifier);
            log_frame.timestamp_us = esp_timer_get_time();
            log_frame.msg = rx_frame;
            
            // Push to circular buffer
            if (!circular_buffer_push(&log_frame)) {
                atomic_fetch_add(&frames_dropped, 1);
            } else {
                atomic_fetch_add(&frames_received, 1);
            }
        }
    }
}


// Initialize circular buffer
static void circular_buffer_init(void) {
    atomic_store(&log_buffer.head, 0);
    atomic_store(&log_buffer.tail, 0);
    atomic_store(&log_buffer.count, 0);
}

// Push frame
static bool circular_buffer_push(const can_log_frame_t *frame) {
    uint32_t current_count = atomic_load(&log_buffer.count);
    
    if (current_count >= CIRCULAR_BUFFER_SIZE) {
        atomic_fetch_add(&frames_dropped, 1);
        return false;               
    }

    uint32_t head = atomic_load(&log_buffer.head);
    log_buffer.buffer[head] = *frame;
    
    atomic_store(&log_buffer.head, (head + 1) % CIRCULAR_BUFFER_SIZE);
    atomic_fetch_add(&log_buffer.count, 1);
    
    return true;
}

// Pop frame 
static bool circular_buffer_pop(can_log_frame_t *frame) {
    if (atomic_load(&log_buffer.count) == 0) {
        return false;
    }

    uint32_t tail = atomic_load(&log_buffer.tail);
    *frame = log_buffer.buffer[tail];
    
    atomic_store(&log_buffer.tail, (tail + 1) % CIRCULAR_BUFFER_SIZE);
    atomic_fetch_sub(&log_buffer.count, 1);
    
    return true;
}

// Get current count
static uint32_t circular_buffer_count(void) {
    return atomic_load(&log_buffer.count);
}

// ========================== SD CARD LOG HANDLER ==========================

int scan_next_available_index(void) {
    char path_buffer[64];
    struct stat st;
    
    uint8_t header_size = 44; 
    int index = atomic_load(&current_file_index);
    const int MAX_SCAN = 500;        

    for (int i = 0; i < MAX_SCAN; i++) {   
        snprintf(path_buffer, sizeof(path_buffer), BASE_LOG_PATH, index);
        
        if (stat(path_buffer, &st) != 0) {
            atomic_store(&current_file_index, index);
            return index;
        }
        
        if (st.st_size <header_size) {
            atomic_store(&current_file_index, index);
            return index;
        }
        index++;
    }

    ESP_LOGE("SD_INDEX", "No available index found! Using %d", index);
    return index;
}

// ========================== SD CARD LOG HANDLER ==========================

//with circular buffer

void log_writer_task(void *pvParameters) {
    can_log_frame_t log_frame;
    int sync_counter = 0;

    const int FLUSH_INTERVAL = 30;
    const uint32_t MAX_FRAMES_PER_FILE = 8000;

    log_file_mutex = xSemaphoreCreateMutex();
    if (log_file_mutex == NULL) {
        ESP_LOGE("SD_LOG", "Failed to create mutex!");
        vTaskDelete(NULL);
    }

    circular_buffer_init();

    vTaskDelay(pdMS_TO_TICKS(300));
    atomic_store(&current_file_index, scan_next_available_index());

    while (1) {
        if (!atomic_load(&logging_active)) {
            close_current_log_file();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (atomic_load(&force_file_rotate)) {
            close_current_log_file();
            atomic_store(&current_file_index, scan_next_available_index());
            atomic_store(&force_file_rotate, false);
        }

        if (!log_state.is_open) {
            if (!open_log_file()) {
                vTaskDelay(pdMS_TO_TICKS(150));
                continue;
            }
        }

        // batching cirular buffer
        bool wrote_something = false;
        while (circular_buffer_pop(&log_frame)) {
            if (write_can_frame_to_csv(&log_frame)) {
                log_state.frames_written++;
                sync_counter++;
                wrote_something = true;
            }

            if (sync_counter >= FLUSH_INTERVAL) {
                flush_log_file();
                sync_counter = 0;
            }

            if (log_state.frames_written >= MAX_FRAMES_PER_FILE) {
                atomic_store(&force_file_rotate, true);
                break;
            }
        }

        if (!wrote_something) {            
            if (log_state.is_open && (esp_timer_get_time() - log_state.last_flush_time > 1000000)) {
                flush_log_file();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static bool open_log_file(void) {
    
    struct stat st;
    bool file_exists = (stat(log_state.current_path, &st) == 0);
    bool is_empty = (!file_exists || st.st_size == 0);
    
    if (xSemaphoreTake(log_file_mutex, pdMS_TO_TICKS(250)) != pdTRUE) 
        return false;

    snprintf(log_state.current_path, sizeof(log_state.current_path), 
             BASE_LOG_PATH, atomic_load(&current_file_index));

    
    log_state.fp = fopen(log_state.current_path, "a");
    if (log_state.fp == NULL) {
        atomic_fetch_add(&write_errors, 1);
        xSemaphoreGive(log_file_mutex);
        return false;
    }

    log_state.is_open = true;
    log_state.frames_written = 0;

    if (is_empty) {
        const char *header = "timestamp_us,id,extended,rtr,dlc,data_hex\n";
        fwrite(header, 1, strlen(header), log_state.fp);
        fflush(log_state.fp);
        ESP_LOGI("SD_LOG", "Created new file with header: %s", log_state.current_path);
    }

    xSemaphoreGive(log_file_mutex);
    return true;
}

static bool write_can_frame_to_csv(const can_log_frame_t *frame) {
    char hex_str[17] = {0};
    char csv_buffer[128];
    
    if (!log_state.fp) return false;

    if (xSemaphoreTake(log_file_mutex, pdMS_TO_TICKS(60)) != pdTRUE) {
        atomic_fetch_add(&write_errors, 1);
        return false;
    }

    for (int i = 0; i < frame->msg.data_length_code; i++) {
        sprintf(&hex_str[i*2], "%02X", frame->msg.data[i]);
    }

    int len = snprintf(csv_buffer, sizeof(csv_buffer),
                       "%llu,0x%08lX,%d,%d,%d,%s\n",
                       frame->timestamp_us, frame->msg.identifier,
                       frame->msg.extd, frame->msg.rtr,
                       frame->msg.data_length_code, hex_str);

    bool success = false;
    if (len > 0) {
        if (fwrite(csv_buffer, 1, len, log_state.fp) == (size_t)len) {
            success = true;
        } else {
            atomic_fetch_add(&write_errors, 1);
        }
    }

    xSemaphoreGive(log_file_mutex);
    return success;
}

static void flush_log_file(void) {
    if (!log_state.is_open || !log_state.fp) return;
    if (xSemaphoreTake(log_file_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

    fflush(log_state.fp);
    xSemaphoreGive(log_file_mutex);
}

static void close_current_log_file(void) {
    if (xSemaphoreTake(log_file_mutex, pdMS_TO_TICKS(300)) != pdTRUE) return;

    if (log_state.fp) {
        fflush(log_state.fp);
        fclose(log_state.fp);
        log_state.fp = NULL;
    }
    log_state.is_open = false;
    xSemaphoreGive(log_file_mutex);
}

// ========================== BUTTON & UART CONTROL ==========================

void button_task(void *pvParameters) {
    uint32_t gpio_num;
    static int64_t last_interrupt_time = 0;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            int64_t now = esp_timer_get_time();

            if (now - last_interrupt_time > 200000) {  // 200ms debounce
                bool new_state = !atomic_load(&logging_active);
                atomic_store(&logging_active, new_state);

                if (new_state) {
                    current_file_index=scan_next_available_index();
                    ESP_LOGI("BUTTON", "Logging STARTED -> can_%03d.csv", 
                             atomic_load(&current_file_index));
                    atomic_store(&force_file_rotate, true);
                } else {
                    ESP_LOGW("BUTTON", "Logging STOPPED");
                    atomic_store(&force_file_rotate, false);
                    logging_active = false;
                    force_file_rotate = false;
                    frames_received = 0;
                    frames_dropped = 0;
                    write_errors = 0;
                }
                last_interrupt_time = now;
            }
        }
    }
}

void uart_task(void *pvParameters) {
    char cmd_buffer[32];
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    int pos = 0;

    while (1) {
        char c;
     
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&c, 1, pdMS_TO_TICKS(10));
        if (c == 0x00) {
        continue; 
        }
        
        if (len > 0) {
            if (c == '\r' || c == '\n' || c == ' ') { 
                if (pos > 0) {
                    cmd_buffer[pos] = '\0';
                    
                    if (strcmp(cmd_buffer, "start") == 0) {
                        logging_active = true;
                    } else if (strcmp(cmd_buffer, "stop") == 0) {
                        logging_active = false;
                        ESP_LOGW("BUTTON", "Logging STOPPED");
                    } else if (strcmp(cmd_buffer, "stats") == 0) {
                        printf("\n--- SYSTEM STATS ---\n");
                        printf("Status      : %s\n", logging_active ? "RUNNING" : "IDLE");
                        printf("Write Errors: %d\n", write_errors);
                        printf("Bus Error   : %s\n", bus_off_detected ? "YES" : "NO");
                        printf("--------------------\n");
                    }
                    
                    pos = 0;
                    memset(cmd_buffer, 0, sizeof(cmd_buffer));
                }
            } else {
                if (pos < sizeof(cmd_buffer) - 1) {
                    cmd_buffer[pos++] = c;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========================== STATUS MONITOR & LED ==========================

void monitor_status_task(void *pvParameters) {
    gpio_reset_pin(STATUS_LED_PIN);
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);

    int64_t last_print_time = 0;
    int64_t last_led_time = 0;
    uint8_t led_state = 0;
    uint8_t write_error_threshold = 5;
    uint32_t blink_interval = 500;  // default

    while (1) {
        int64_t now = esp_timer_get_time();

        // === Update System Status ===
        if (now - last_print_time > 1000000) {  // 1 detik
            if (atomic_load(&logging_active)) {
                printf("\n--- [CAN LOGGER STATUS] ---\n");
                printf("Frames Received : %d\n", atomic_load(&frames_received));
                printf("Frames Dropped  : %d\n", atomic_load(&frames_dropped));
                printf("Write Errors    : %d\n", atomic_load(&write_errors));
                printf("Buffered        : %lu / %d\n", circular_buffer_count(), CIRCULAR_BUFFER_SIZE);
                printf("Current File    : can_%03d.csv\n", atomic_load(&current_file_index));
                printf("---------------------------\n");
            }

            last_print_time = now;
        }
        
        // === Determine Current Status ===
        led_status_t status;
        if (sd_card_not_found) {
            status = LED_STATUS_SD_FAIL;
        } else if (atomic_load(&bus_off_detected)) {
            status = LED_STATUS_BUS_OFF;
        } else if (atomic_load(&write_errors) > write_error_threshold) {
            status = LED_STATUS_ERROR;
        } else if (atomic_load(&logging_active)) {
            status = LED_STATUS_LOGGING;
        } else {
            status = LED_STATUS_IDLE;
        }
        

        // === Non-blocking LED Pattern ===
        
        switch (status) {
            case LED_STATUS_SD_FAIL:      // Fast triple blink
                blink_interval = 80000;
                if (now - last_led_time > blink_interval) {
                    led_state = !led_state;
                    gpio_set_level(STATUS_LED_PIN, led_state);
                    last_led_time = now;
                }
                break;

            case LED_STATUS_BUS_OFF:      // Solid OFF
                gpio_set_level(STATUS_LED_PIN, 0);
                break;

            case LED_STATUS_IDLE:         // Slow blink (1s on, 4s off)
                blink_interval = 500000;
                if (now - last_led_time > (led_state ? 100000 : 400000)) {
                    led_state = !led_state;
                    gpio_set_level(STATUS_LED_PIN, led_state);
                    last_led_time = now;
                }
                break;

            case LED_STATUS_LOGGING:      // Fast heartbeat (50ms on, 150ms off)
                blink_interval = 200000;
                if (now - last_led_time > (led_state ? 50000 : 150000)) {
                    led_state = !led_state;
                    gpio_set_level(STATUS_LED_PIN, led_state);
                    last_led_time = now;
                }
                break;

            case LED_STATUS_ERROR:        // Double blink
                blink_interval = 250000;
                if (now - last_led_time > blink_interval) {
                    led_state++;
                    if (led_state > 3) led_state = 0;
                    
                    if (led_state == 1 || led_state == 3) {
                        gpio_set_level(STATUS_LED_PIN, 1);
                    } else {
                        gpio_set_level(STATUS_LED_PIN, 0);
                    }
                    last_led_time = now;
                }
                break;

            default:
                gpio_set_level(STATUS_LED_PIN, 0);
                break;
            
        }
                
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ========================== INITIALIZATION FUNCTIONS ==========================
void init_twai(void) {
    ESP_LOGI(TAG, "Initializing On-Chip TWAI Node with Advanced Timing...");

    can_rx_queue = xQueueCreate(100, sizeof(can_flat_frame_t));

    if (can_rx_queue == NULL) {
        ESP_LOGE("MAIN", "Failed to allocate Queue!");
        return;
    }

    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TWAI_TX_PIN,
        .io_cfg.rx = TWAI_RX_PIN,
        .bit_timing.bitrate = CONFIG_TWAI_BITRATE_VALUE, 
        .timestamp_resolution_hz = 1000000,
        .flags.enable_listen_only = true, 
        .flags.enable_self_test = false,
    };

    esp_err_t ret = twai_new_node_onchip(&node_config, &twai_node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(ret));
        return;
    }

    twai_timing_advanced_config_t timing_cfg;
    
    if (CONFIG_TWAI_BITRATE_VALUE == 500000) {
        timing_cfg.brp = 8;
        timing_cfg.prop_seg =10;
        timing_cfg.tseg_1 = 4;
        timing_cfg.tseg_2 = 5;
        timing_cfg.sjw = 3;
        timing_cfg.ssp_offset=0;
        ESP_LOGI(TAG, "Applying Advanced Timing: 500kbps (BRP:8,prop seg:10, Tseg1:4, Tseg2:5)");
    } else if(CONFIG_TWAI_BITRATE_VALUE == 250000) {
        timing_cfg.brp = 16;
        timing_cfg.prop_seg =10;
        timing_cfg.tseg_1 = 4;
        timing_cfg.tseg_2 = 5;
        timing_cfg.sjw = 3;
        timing_cfg.ssp_offset=0;
        
        ESP_LOGI(TAG, "Applying Advanced Timing: 250kbps (BRP:16,prop seg:10, Tseg1:4, Tseg2:5)");
    }
        
    ret = twai_node_reconfig_timing(twai_node, &timing_cfg, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TWAI advanced timing: %s", esp_err_to_name(ret));
        return;
    }

    twai_mask_filter_config_t dual_config = twai_make_dual_filter(
        0x0CF0, 0xFFFF,   // Filter 1
        0x0F00, 0xFFFF,   // Filter 2
        true              // Extended frame
    );

    ret = twai_node_config_mask_filter(twai_node, 0, &dual_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TWAI acceptance filter configured successfully");
    } else {
        ESP_LOGW(TAG, "Failed to set filter: %s (continuing without filter)", esp_err_to_name(ret));
    }
    
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
        .on_error = twai_listener_on_error_callback,
        .on_state_change = twai_listener_on_state_change_callback,
    };
    
    esp_err_t err_cb = twai_node_register_event_callbacks(twai_node, &user_cbs, NULL);
    if (err_cb != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event callbacks: %s", esp_err_to_name(err_cb));
        return;
    }
    
    ret = twai_node_enable(twai_node);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TWAI Controller started successfully | Bitrate: %d bps | Listen-Only: %s",
                 CONFIG_TWAI_BITRATE_VALUE,
                 node_config.flags.enable_listen_only ? "ENABLED" : "DISABLED");
    } else {
        ESP_LOGE(TAG, "Failed to enable TWAI node: %s", esp_err_to_name(ret));
    }
}


void init_sdmmc(void) {
    ESP_LOGI(TAG, "Initializing SD Card via SDMMC 4-bit mode...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    //host.max_freq_khz = 4000; // Un-comment to customize frequency
    
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = (gpio_num_t)SDMMC_CLK_PIN;
    slot_config.cmd = (gpio_num_t)SDMMC_CMD_PIN;
    slot_config.d0  = (gpio_num_t)SDMMC_D0_PIN;
    slot_config.d1  = (gpio_num_t)SDMMC_D1_PIN;
    slot_config.d2  = (gpio_num_t)SDMMC_D2_PIN;
    slot_config.d3  = (gpio_num_t)SDMMC_D3_PIN;
    
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; 

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SD Card initialization: SUCCESSFUL.");
        sd_card_not_found = false;
        sdmmc_card_print_info(stdout, sd_card); 
    } else {
        sd_card = NULL; 
        sd_card_not_found = true;
        ESP_LOGE(TAG, "SD Card initialization: FAILED (%s). The device continues to operate without logging functionality.", esp_err_to_name(ret));
    }
}

void init_ui_and_uart(void) {
    esp_err_t ret;
    bool ui_success = true;
    bool uart_success = true;

    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << STATUS_LED_PIN), 
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    if (gpio_config(&led_conf) != ESP_OK) {
        ui_success = false;
    }

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << USER_BUTTON_PIN), 
        .mode = GPIO_MODE_INPUT, 
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    if (gpio_config(&btn_conf) != ESP_OK) {
        ui_success = false;
    }

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ui_success = false;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { 
        ui_success = false;
    }

    if (gpio_isr_handler_add((gpio_num_t)USER_BUTTON_PIN, gpio_isr_handler, (void*) USER_BUTTON_PIN) != ESP_OK) {
        ui_success = false;
    }

    if (ui_success) {
        ESP_LOGI(TAG, "GPIO UI Initialization (LED & Button): SUCCESSFUL");
    } else {
        ESP_LOGE(TAG, "GPIO UI Initialization (LED & Button): FAILED");
    }

    uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);

    if (!uart_is_driver_installed(UART_NUM)){
    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
        if  (ret != ESP_OK) {
            ESP_LOGE(TAG, "UART driver installation FAILED: %s", esp_err_to_name(ret));
            uart_success = false;
        }
        if (uart_success) {
            ESP_LOGI(TAG, "UART Shell Initialization (UART %d): SUCCESS", UART_NUM);
        }   else {
            ESP_LOGE(TAG, "UART Shell initialization (UART %d): Overall FAILED", UART_NUM);
        }
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "=== CAN Logger Starting ===");

    init_twai();
    init_sdmmc();
    init_ui_and_uart();

    xTaskCreatePinnedToCore(button_task, "button_task", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx_task", 3072, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(log_writer_task, "log_writer_task", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(monitor_status_task, "status_monitor_t", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(uart_task, "uart_task", 2048, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "All tasks started successfully.");
}