#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "PMW3901.h"

static const char *TAG = "MAIN";

// 硬件引脚
#define SPI_SCLK_GPIO 7
#define SPI_MOSI_GPIO 15
#define SPI_MISO_GPIO 16
#define PMW3901_CS_GPIO 6

void app_main(void)
{
    esp_err_t ret;
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = SPI_MISO_GPIO,
        .sclk_io_num = SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ret = spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "SPI bus OK (SCK=%d, MOSI=%d, MISO=%d)",
             SPI_SCLK_GPIO, SPI_MOSI_GPIO, SPI_MISO_GPIO);

    spi_device_interface_config_t pmw3901_dev_cfg = {
        .mode = 3,
        .clock_speed_hz = 2 * 1000 * 1000,
        .spics_io_num = PMW3901_CS_GPIO,
        .queue_size = 1,
        .cs_ena_pretrans = 200,
        .cs_ena_posttrans = 200,
    };
    spi_device_handle_t pmw3901_spi_dev = NULL;
    ret = spi_bus_add_device(SPI2_HOST, &pmw3901_dev_cfg, &pmw3901_spi_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "PMW3901 SPI device OK (CS=%d)", PMW3901_CS_GPIO);
    ret = pmw3901_init(pmw3901_spi_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize PMW3901: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "PMW3901 OK");
    bool pmw_ok = (ret == ESP_OK);
    int16_t dx = 0, dy = 0;
    uint8_t squal = 0;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (pmw_ok)
        {
            pmw3901_read_motion(&dx, &dy);
            ESP_LOGI(TAG, "dx = %d, dy = %d\n", dx, dy);
            pmw3901_read_squal(&squal);
            ESP_LOGI(TAG, "squal = %d\n", squal);
        }
    }
}
