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
QueueHandle_t sd_log_queue = NULL;

// Statistics (atomic for thread safety)
atomic_uint_fast32_t frames_received = 0;
atomic_uint_fast32_t frames_dropped = 0;
atomic_uint_fast32_t write_errors = 0;

// System state
volatile bool logging_active = false; 
volatile int current_file_index = 1;
volatile bool force_file_rotate = false;
volatile bool sd_card_not_found = false; 
volatile bool bus_off_detected = false;  

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

// ========================== FUNCTION PROTOTYPES ==========================
void decode_j1939_id(uint32_t id);
void init_twai(void);
void init_sdmmc(void);
void init_ui_and_uart(void);
int scan_next_available_index(void);
FILE* verify_and_open_csv(bool*);

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
                .rtr = rx_frame.header.rtr
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
        // Handle Bus-Off recovery notification
        if (xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, 0) == pdTRUE) {
            if (notification_value & 0x01) {
                ESP_LOGW("CAN_TASK", "Received Bus-Off signal... Executing twai_node_recover()...");
                if (twai_node_recover(twai_node) == ESP_OK) {
                    bus_off_detected = true; 
                    ESP_LOGI("CAN_TASK", "twai_node_recover() successful.");
                } else {
                    bus_off_detected = false;
                    ESP_LOGE("CAN_TASK", "Failed to execute twai_node_recover()!");
                }
            }
        }

        if (xQueueReceive(can_rx_queue, &rx_frame, portMAX_DELAY) == pdTRUE) {
            // Process incoming CAN frames
            if (!logging_active) {
                continue; 
            }
            decode_j1939_id(rx_frame.identifier);

            if (force_file_rotate) {
                frames_received = 0;
                frames_dropped = 0;
            }

            atomic_fetch_add(&frames_received, 1);
    
            log_frame.timestamp_us = esp_timer_get_time();
            log_frame.msg = rx_frame;

            if (xQueueSend(sd_log_queue, &log_frame, 0) != pdTRUE) {
                atomic_fetch_add(&frames_dropped, 1);
            }
        }
    }
}

// ========================== SD CARD LOG HANDLER ==========================
FILE* verify_and_open_csv(bool *is_new_file) {
    struct stat st;
    
    if (stat(BASE_LOG_PATH, &st) == 0) {
        *is_new_file = false;
    } else {
        *is_new_file = true;
    }

    FILE *f = fopen(BASE_LOG_PATH, "a");
    if (f == NULL) {
        atomic_fetch_add(&write_errors, 1);
        return NULL;
    }

    if (*is_new_file) {
        const char *header = "timestamp_us,id,extended,rtr,dlc,data_hex\n";
        if (fwrite(header, 1, strlen(header), f) != strlen(header)) {
            atomic_fetch_add(&write_errors, 1);
            fclose(f);
            return NULL;
        }
        fflush(f); 
    }
    return f;
}

int scan_next_available_index(void) {
    int index = 1;
    char path_buffer[64];
    struct stat st;

    while (1) {
        snprintf(path_buffer, sizeof(path_buffer), BASE_LOG_PATH, index);
        
        if (stat(path_buffer, &st) != 0) {
            return index;
        }
        
        if (st.st_size == 0) {
            return index;
        }

        index++;
    }
}

void log_writer_task(void *pvParameters) {
    can_log_frame_t log_frame;
    char csv_buffer[128];
    char current_file_path[64];
    int sync_counter = 0;
    FILE *f = NULL;
    bool sd_healthy = true;

    vTaskDelay(pdMS_TO_TICKS(200)); 
    current_file_index = scan_next_available_index();
    ESP_LOGI("SD_INIT", "Initial index file ready for use: can_%03d.csv", current_file_index);

    while (1) {
        if (xQueueReceive(sd_log_queue, &log_frame, pdMS_TO_TICKS(500)) == pdTRUE) {
            
            if (!logging_active) {
                if (f != NULL) {
                    fclose(f); 
                    f = NULL;
                }
                continue; 
            }
            // Handle file rotation
            if (force_file_rotate) {
                if (f != NULL) {
                fclose(f);
                f = NULL;
            }
                sync_counter = 0;
    
                force_file_rotate = false; 
            }

            if (force_file_rotate) {
                if (f != NULL) {
                    fclose(f);
                    f = NULL;
                }
                sync_counter = 0;
                force_file_rotate = false;
            }
            // Open file if not open
            if (f == NULL) {
                snprintf(current_file_path, sizeof(current_file_path), BASE_LOG_PATH, current_file_index);

                struct stat st;
                bool need_header = (stat(current_file_path, &st) != 0 || st.st_size == 0);

                f = fopen(current_file_path, "a");
                if (f == NULL) {
                    sd_healthy = false;
                    atomic_fetch_add(&write_errors, 1);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue; 
                }
                sd_healthy = true;

                if (need_header) {
                    const char *header = "timestamp_us,id,extended,rtr,dlc,data_hex\n";
                    if (fwrite(header, 1, strlen(header), f) == strlen(header)) {
                        fflush(f);
                    } else {
                        atomic_fetch_add(&write_errors, 1);
                    }
                }
            }
            // Convert data to hex string
            char hex_str[17] = {0}; 
            for (int i = 0; i < log_frame.msg.data_length_code; i++) {
                sprintf(&hex_str[i * 2], "%02X", log_frame.msg.data[i]);
            }
            // Write CSV line
            int len = snprintf(csv_buffer, sizeof(csv_buffer),
                               "%llu,0x%08lX,%d,%d,%d,%s\n",
                               log_frame.timestamp_us, log_frame.msg.identifier,
                               log_frame.msg.extd, log_frame.msg.rtr,
                               log_frame.msg.data_length_code, hex_str);

            if (len > 0 && f != NULL) {
                if (fwrite(csv_buffer, 1, len, f) != len) {
                    atomic_fetch_add(&write_errors, 1);
                    fclose(f);
                    f = NULL;
                    sd_healthy = false; 
                } else {
                    sync_counter++;
                }
            }
            // Periodic sync
            if (sync_counter >= 50) {
                if (f != NULL) {
                    fclose(f);
                    f = NULL;
                }
                sync_counter = 0;
            }
        } else {
            // Timeout: close file for safety
            if (f != NULL) {
                fclose(f);
                f = NULL;
                sync_counter = 0;
            }
        }
    }
}

// ========================== BUTTON & UART CONTROL ==========================
void button_task(void *pvParameters) {
    uint32_t gpio_num;
    static int64_t last_interrupt_time = 0;
    int64_t debounce_delay_us = 200000; // 200ms in microseconds

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            int64_t current_time = esp_timer_get_time();

            if ((current_time - last_interrupt_time) > debounce_delay_us) {
                
                logging_active = !logging_active;

                if (logging_active) {
                    current_file_index = scan_next_available_index();
                    force_file_rotate = true; 
                    ESP_LOGI("BUTTON", "Logging STARTED -> Target File: can_%03d.csv", current_file_index);
                } else {
                    ESP_LOGW("BUTTON", "Logging STOPPED");
                    frames_received = 0;
                    frames_dropped = 0;
                    write_errors = 0;
                }
                last_interrupt_time = current_time;
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
                        ESP_LOGI("BUTTON", "Logging STARTED -> Target File: can_%03d.csv", current_file_index);
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
    
    while (1) {
        if (logging_active) {
            printf("\n--- [CAN LOGGER STATUS] ---\n");
            printf("Frames Received : %d\n", frames_received);
            printf("Frames Dropped  : %d\n", frames_dropped);
            printf("Write Errors    : %d\n", write_errors);
            printf("---------------------------\n");
        }

        // Update LED status
        if (sd_card_not_found) {
            current_led_status = LED_STATUS_SD_FAIL;
        } else if (bus_off_detected) {
            current_led_status = LED_STATUS_BUS_OFF;
        } else if (write_errors > 0) {
            current_led_status = LED_STATUS_ERROR;
        } else if (logging_active) {
            current_led_status = LED_STATUS_LOGGING;
        } else {
            current_led_status = LED_STATUS_IDLE;
        }
        
        // LED patterns (non-blocking style)
        switch (current_led_status) {
            case LED_STATUS_STARTUP:
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;

            case LED_STATUS_SD_FAIL: 
                for(int i=0; i<3; i++) { 
                    gpio_set_level(STATUS_LED_PIN, 1); vTaskDelay(100); gpio_set_level(STATUS_LED_PIN, 0); vTaskDelay(100);
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case LED_STATUS_BUS_OFF: 
                gpio_set_level(STATUS_LED_PIN, 0); 
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case LED_STATUS_IDLE: 
                
                gpio_set_level(STATUS_LED_PIN, 1); vTaskDelay(200); gpio_set_level(STATUS_LED_PIN, 0); vTaskDelay(800);
                break;

            case LED_STATUS_LOGGING: 
                
                gpio_set_level(STATUS_LED_PIN, 1); vTaskDelay(50); gpio_set_level(STATUS_LED_PIN, 0); vTaskDelay(100);
                break;

            case LED_STATUS_ERROR: 
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(STATUS_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(STATUS_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(600)); 
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}

// ========================== INITIALIZATION FUNCTIONS ==========================
void init_twai(void) {
    ESP_LOGI(TAG, "Initializing On-Chip TWAI Node with Advanced Timing...");

    can_rx_queue = xQueueCreate(30, sizeof(can_flat_frame_t));
    sd_log_queue = xQueueCreate(100, sizeof(can_log_frame_t));

    if (can_rx_queue == NULL || sd_log_queue == NULL) {
        ESP_LOGE("MAIN", "Failed to allocate Queue!");
        return;
    }

    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TWAI_TX_PIN,
        .io_cfg.rx = TWAI_RX_PIN,
        .bit_timing.bitrate = CONFIG_TWAI_BITRATE_VALUE, 
        .timestamp_resolution_hz = 1000000,
        .flags.enable_listen_only = true, 
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
    0x0CF0, 
    0xFFFF, 
    0x0F00, 
    0xFFFF, 
    true    
    );

    ESP_ERROR_CHECK(twai_node_config_mask_filter(twai_node, 0, &dual_config));
    ESP_LOGI(TAG, "Filter enabled");
    

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
        ESP_LOGI(TAG, "TWAI Node with Advanced Timing Successfully Operated.");
    } else {
        ESP_LOGE(TAG, "Failed to activate TWAI node: %s", esp_err_to_name(ret));
    }
}


void init_sdmmc(void) {
    ESP_LOGI(TAG, "Initializing SD Card via SDMMC 4-bit mode...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    //host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    host.max_freq_khz = 4000; // Un-comment to customize frequency
    
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

    xTaskCreatePinnedToCore(button_task, "button_task", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx_task", 3072, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(log_writer_task, "log_writer_task", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(monitor_status_task, "status_monitor_t", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(uart_task, "uart_task", 2048, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "All tasks started successfully.");
}