#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "PMW3901.h"
#include "VL53L0X.h"
static const char *TAG = "MAIN ";

// I2C 引脚
#define I2C_SDA_GPIO 41
#define I2C_SCL_GPIO 42
#define I2C_FREQ_HZ 400000

#define SPI_SCLK_GPIO 7
#define SPI_MOSI_GPIO 15
#define SPI_MISO_GPIO 16
#define PMW3901_CS_GPIO 6

#define VL53L0X_ADDR 0x29

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

    i2c_master_bus_config_t i2c_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    ret = i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C bus OK (SDA=%d, SCL=%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);
    i2c_device_config_t VL53L0X_dev_cfg =
        {
            .dev_addr_length = I2C_ADDR_BIT_7,
            .device_address = VL53L0X_ADDR,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
    i2c_master_dev_handle_t vl53l0x_i2c_dev = NULL;
    ret = i2c_master_bus_add_device(i2c_bus_handle, &VL53L0X_dev_cfg, &vl53l0x_i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C device OK (SDA=%d, SCL=%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);

    vl53l0x_config_t tof_config;
    vl53l0x_init_default_config(&tof_config);
    tof_config.i2c_address = VL53L0X_ADDR;
    tof_config.timeout_ms = 500;
    tof_config.io_2v8 = true;
    tof_config.interrupt_pin = 1;

    vl53l0x_t tof_dev;
    ret = vl53l0x_init(&tof_dev, vl53l0x_i2c_dev, &tof_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "VL53L0X init failed: %s", esp_err_to_name(ret));
    }
    else
    {
        vl53l0x_set_range_mode(&tof_dev, 2);
        ESP_LOGI(TAG, "VL53L0X OK (long-range)");
    }

    bool tof_ok = (ret == ESP_OK);
    uint16_t dist = 0;
    bool dist_ok = false;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (pmw_ok)
        {
            pmw3901_read_motion(&dx, &dy);
            ESP_LOGI(TAG, "dx = %d, dy = %d\n", dx, dy);
            pmw3901_read_squal(&squal);
            ESP_LOGI(TAG, "squal = %d\n", squal);
        }
        if (tof_ok)
        {
            dist = vl53l0x_read_range_single_mm(&tof_dev);
            dist_ok = !vl53l0x_timeout_occurred(&tof_dev) && dist < 8000 && dist >= 30;
        }
        if (dist_ok && tof_ok)
        {
            ESP_LOGI(TAG, "Distance: %d mm", dist);
        }
        else
        {
            ESP_LOGW(TAG, "Distance read failed");
        }
    }
}