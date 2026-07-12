#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <sys/stat.h> // Wajib ditambahkan di bagian atas berkas

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
#define LOG_FILE_PATH   MOUNT_POINT "/can_log.csv"

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

static QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t can_rx_queue; 
TaskHandle_t can_rx_task_handle = NULL;
QueueHandle_t sd_log_queue = NULL;


atomic_uint_fast32_t frames_received = 0;
atomic_uint_fast32_t frames_dropped = 0;
atomic_uint_fast32_t write_errors = 0;

#define BASE_LOG_PATH         "/sdcard/can_%03d.csv"

volatile bool logging_active = false; 
volatile int current_file_index = 1;
volatile bool force_file_rotate = false;
volatile bool sd_card_not_found = false; // Flag error mount SD
volatile bool bus_off_detected = false;  // Flag error CAN Bus-Off

typedef enum {
    LED_STATUS_STARTUP,    // Solid ON saat booting
    LED_STATUS_SD_FAIL,    // Pola SOS (3 kedipan pendek)
    LED_STATUS_IDLE,       // Slow blink
    LED_STATUS_LOGGING,    // Heartbeat/Fast blink
    LED_STATUS_BUS_OFF,    // Solid OFF (atau pola khusus)
    LED_STATUS_ERROR       // Double blink
} led_status_t;

volatile led_status_t current_led_status = LED_STATUS_IDLE;

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

void can_rx_task(void *pvParameters) {
    can_flat_frame_t rx_frame;
    can_log_frame_t log_frame;
    uint32_t notification_value = 0;

    can_rx_task_handle = xTaskGetCurrentTaskHandle();

    while (1) {
        // 1. Error Handling & Recovery (Tetap terjaga)
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
            
            if (!logging_active) {
                continue; 
            }

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

FILE* verify_and_open_csv(bool *is_new_file) {
    struct stat st;
    
    if (stat(LOG_FILE_PATH, &st) == 0) {
        *is_new_file = false;
    } else {
        *is_new_file = true;
    }

    FILE *f = fopen(LOG_FILE_PATH, "a");
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

            char hex_str[17] = {0}; 
            for (int i = 0; i < log_frame.msg.data_length_code; i++) {
                sprintf(&hex_str[i * 2], "%02X", log_frame.msg.data[i]);
            }

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

            if (sync_counter >= 50) {
                if (f != NULL) {
                    fclose(f);
                    f = NULL;
                }
                sync_counter = 0;
            }
        } else {
            if (f != NULL) {
                fclose(f);
                f = NULL;
                sync_counter = 0;
            }
        }
    }
}


void button_task(void *pvParameters) {
    int last_state = 1;
    
    while (1) {
        int current_state = gpio_get_level(USER_BUTTON_PIN);

        if (last_state == 1 && current_state == 0) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
            if (gpio_get_level(USER_BUTTON_PIN) == 0) {
                
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
                
                while (gpio_get_level(USER_BUTTON_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void led_status_task(void *pvParameters) {
    gpio_reset_pin(STATUS_LED_PIN);
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        // Tentukan status berdasarkan logika global
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

        // Jalankan pola kedipan
        switch (current_led_status) {
            case LED_STATUS_STARTUP:
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;

            case LED_STATUS_SD_FAIL: // SOS Pattern (3 pendek, 3 panjang, 3 pendek)
                for(int i=0; i<3; i++) { // 3 pendek
                    gpio_set_level(STATUS_LED_PIN, 1); vTaskDelay(100); gpio_set_level(STATUS_LED_PIN, 0); vTaskDelay(100);
                }
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case LED_STATUS_BUS_OFF: // Pola OFF atau nyala sangat redup
                gpio_set_level(STATUS_LED_PIN, 0); 
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case LED_STATUS_IDLE: // Slow blink 1x per detik
                gpio_set_level(STATUS_LED_PIN, 1); vTaskDelay(200); gpio_set_level(STATUS_LED_PIN, 0); vTaskDelay(800);
                break;

            case LED_STATUS_LOGGING: // Fast heartbeat
                gpio_set_level(STATUS_LED_PIN, 1); vTaskDelay(50); gpio_set_level(STATUS_LED_PIN, 0); vTaskDelay(250);
                break;

            case LED_STATUS_ERROR: // Double Blink (Peringatan Error)
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(STATUS_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(STATUS_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(600)); // Jeda sebelum mengulang pola
                break;
        }
    }
}

void monitor_status_task(void *pvParameters) {
    while (1) {
        if (logging_active) {
            printf("\n--- [CAN LOGGER STATUS] ---\n");
            printf("Frames Received : %d\n", frames_received);
            printf("Frames Dropped  : %d\n", frames_dropped);
            printf("Write Errors    : %d\n", write_errors);
            printf("---------------------------\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

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

    twai_mask_filter_config_t j1939_filter;
    j1939_filter = twai_make_dual_filter(
        0x0CF0, // ID 1 
        0xFFFF, // Mask 1 
        0x0000, // ID 2 
        0x0000, // Mask 2 
        true    // is_ext = true (for J1939)
    );

    ESP_ERROR_CHECK(twai_node_config_mask_filter(twai_node, 0, &j1939_filter));
    ESP_LOGI(TAG, "Filter enabled for ID: 0x%03X Mask: 0x%03X", j1939_filter.id, j1939_filter.mask);
    
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
        sd_card_not_found = true;
        sdmmc_card_print_info(stdout, sd_card); 
    } else {
        sd_card = NULL; 
        sd_card_not_found = false;
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
    init_twai();
    init_sdmmc();
    init_ui_and_uart();

    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx_task", 3072, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(log_writer_task, "log_writer_task", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(monitor_status_task, "status_monitor_t", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(led_status_task, "led_status_task", 2048, NULL, 2, NULL, 1);
}