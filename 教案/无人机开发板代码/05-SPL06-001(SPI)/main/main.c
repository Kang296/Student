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
#include "spl06001.h"

static const char *TAG = "MAIN";

// 硬件引脚
#define SPI_SCLK_GPIO 7
#define SPI_MOSI_GPIO 15
#define SPI_MISO_GPIO 16
#define SPL_CS_GPIO 8

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

    spi_device_interface_config_t spl06001_dev_cfg = {
        .mode = 3,
        .clock_speed_hz = 1 * 1000 * 1000,
        .spics_io_num = SPL_CS_GPIO,
        .queue_size = 1,
    };
    spi_device_handle_t spl06001_spi_dev = NULL;
    ret = spi_bus_add_device(SPI2_HOST, &spl06001_dev_cfg, &spl06001_spi_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "spl06001 SPI device OK (CS=%d)", SPL_CS_GPIO);
    ret = spl06_init(spl06001_spi_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize spl06: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "spl06001 OK");
    bool spl_ok = (ret == ESP_OK);
    spl06_data_t data;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (spl_ok)
        {
            spl06_read(&data);
            ESP_LOGI(TAG, "temperature_c = %.2f, pressure_pa = %.2f, pressure_hpa = %.2f, altitude_m = %.2f\n", data.temperature_c, data.pressure_pa, data.pressure_hpa, data.altitude_m);
            float altitude_m = spl06_pressure_to_altitude(data.pressure_pa);
            ESP_LOGI(TAG, "altitude_m = %.2f\n", altitude_m);
        }
    }
}
