#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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


#define SDMMC_CLK_PIN   GPIO_NUM_12
#define SDMMC_CMD_PIN   GPIO_NUM_11
#define SDMMC_D0_PIN    GPIO_NUM_13
#define SDMMC_D1_PIN    GPIO_NUM_14
#define SDMMC_D2_PIN    GPIO_NUM_9
#define SDMMC_D3_PIN    GPIO_NUM_10

#define TWAI_TX_PIN     GPIO_NUM_4
#define TWAI_RX_PIN     GPIO_NUM_5

#define USER_BUTTON_PIN GPIO_NUM_1
#define STATUS_LED_PIN  GPIO_NUM_6

#define UART_TX         GPIO_NUM_17
#define UART_RX         GPIO_NUM_18

#define UART_NUM        UART_NUM_1
#define UART_BUF_SIZE   1024


sdmmc_card_t *sd_card = NULL;
const char mount_point[] = "/sdcard";

static const char *TAG = "TWAI_CAN";
twai_node_handle_t twai_node = NULL;


static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void init_twai(void) {
    ESP_LOGI(TAG, "Initializing On-Chip TWAI Node with Advanced Timing...");

    
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = (gpio_num_t)TWAI_TX_PIN,
        .io_cfg.rx = (gpio_num_t)TWAI_RX_PIN,
        .bit_timing.bitrate = CONFIG_TWAI_BITRATE_VALUE, 
        .tx_queue_depth = 5,
        .flags.enable_listen_only = true 
    };

    
    esp_err_t ret = twai_new_node_onchip(&node_config, &twai_node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(ret));
        return;
    }

    twai_timing_advanced_config_t timing_cfg;

    if (CONFIG_TWAI_BITRATE_VALUE == 500000) {
        // Konfigurasi untuk 500 kbps (Sample Point ~80%)
        timing_cfg.brp = 8;
        timing_cfg.tseg_1 = 15;
        timing_cfg.tseg_2 = 4;
        timing_cfg.sjw = 3;
        ESP_LOGI(TAG, "Applying Advanced Timing: 500kbps (BRP:8, Tseg1:15, Tseg2:4)");
    } else {
        timing_cfg.brp = 16;
        timing_cfg.tseg_1 = 15;
        timing_cfg.tseg_2 = 4;
        timing_cfg.sjw = 3;
        ESP_LOGI(TAG, "Applying Advanced Timing: 250kbps (BRP:16, Tseg1:15, Tseg2:4)");
    }

    ret = twai_node_reconfig_timing(twai_node, &timing_cfg, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TWAI advanced timing: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = twai_node_enable(twai_node);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TWAI Node with Advanced Timing Successfully Operated.");
    } else {
        ESP_LOGE(TAG, "Failed to activate TWAI node: %s", esp_err_to_name(ret));
    }
}

static void init_sdmmc(void)
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = SDMMC_CLK_PIN;
    slot_config.cmd = SDMMC_CMD_PIN;
    slot_config.d0  = SDMMC_D0_PIN;
    slot_config.d1  = SDMMC_D1_PIN;
    slot_config.d2  = SDMMC_D2_PIN;
    slot_config.d3  = SDMMC_D3_PIN;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &sd_card);
    
    if (ret != ESP_OK) {
        printf("Failed to initialize SDMMC: %s\n", esp_err_to_name(ret));
        sd_card = NULL; 
        return;
    }
    printf("SDMMC successfully mounted globally.\n");
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
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .source_clk = UART_SCLK_DEFAULT
    };

    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver installation FAILED: %s", esp_err_to_name(ret));
        uart_success = false;
    }

    if (uart_success) {
        ret = uart_param_config(UART_NUM, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "UART parameter configuration FAILED: %s", esp_err_to_name(ret));
            uart_success = false;
        }
    }

    if (uart_success) {
        ret = uart_set_pin(UART_NUM, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "UART pin setting FAILED: %s", esp_err_to_name(ret));
            uart_success = false;
        }
    }

    // Cetak Status Akhir UART
    if (uart_success) {
        ESP_LOGI(TAG, "UART Shell Initialization (UART %d): SUCCESS", UART_NUM);
    } else {
        ESP_LOGE(TAG, "UART Shell initialization (UART %d): Overall FAILED", UART_NUM);
    }
}


void app_main(void)
{
    init_twai();
    init_sdmmc();
    init_ui_and_uart();        
}