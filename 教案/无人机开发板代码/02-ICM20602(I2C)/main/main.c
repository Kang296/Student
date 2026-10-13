#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "icm20602.h"

static const char *TAG = "MAIN";

// I2C 引脚
#define I2C_SDA_GPIO 41
#define I2C_SCL_GPIO 42
#define I2C_FREQ_HZ 400000

// 测试打印周期
#define TEST_PERIOD_MS 100

void app_main(void)
{
    esp_err_t ret;

    //  1. 初始化 I2C 总线
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    ret = i2c_new_master_bus(&bus_cfg, &bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C bus OK (SDA=%d, SCL=%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);

    //  2. 挂载 ICM20602 设备
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = ICM20602_ADDR_LOW, // 0x68, SA0 接地
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t i2c_dev = NULL;
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Add ICM20602 device failed: %s", esp_err_to_name(ret));
        return;
    }

    //  3. 初始化传感器（内部会做 WHO_AM_I 校验 + 软复位 + 量程配置）
    icm20602_config_t cfg;
    icm20602_init_default_config(&cfg);
    // 可按需修改量程，例如：
    // cfg.acc_range  = ICM20602_ACC_2G;
    // cfg.gyro_range = ICM20602_GYRO_1000DPS;

    icm20602_t imu;
    ret = icm20602_init(&imu, i2c_dev, &cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ICM20602 init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ICM20602 init OK");

    //  4. 循环读取并打印
    icm20602_raw_data_t raw;
    icm20602_data_t data;
    uint32_t sample = 0;

    while (1)
    {
        ret = icm20602_read_raw(&imu, &raw);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "read raw failed: %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG,
                     "[%lu] RAW  ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d | temp=%6d",
                     (unsigned long)sample,
                     raw.acc_x, raw.acc_y, raw.acc_z,
                     raw.gyro_x, raw.gyro_y, raw.gyro_z,
                     raw.temp);
        }

        ret = icm20602_read_data(&imu, &data);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "read data failed: %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG,
                     "[%lu] PHY  ax=%+.3fg ay=%+.3fg az=%+.3fg | "
                     "gx=%+7.2f gy=%+7.2f gz=%+7.2f dps | temp=%+.1f C",
                     (unsigned long)sample,
                     data.acc_x_g, data.acc_y_g, data.acc_z_g,
                     data.gyro_x_dps, data.gyro_y_dps, data.gyro_z_dps,
                     data.temp_c);
        }

        sample++;
        vTaskDelay(pdMS_TO_TICKS(TEST_PERIOD_MS));
    }
}
