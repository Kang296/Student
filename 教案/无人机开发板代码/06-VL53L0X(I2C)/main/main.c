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
#include "VL53L0X.h"

static const char *TAG = "MAIN";

#define I2C_SDA_GPIO 41
#define I2C_SCL_GPIO 42
#define I2C_FREQ_HZ 400000

#define VL53L0X_ADDR 0x29

void app_main(void)
{
    esp_err_t ret;
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
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
