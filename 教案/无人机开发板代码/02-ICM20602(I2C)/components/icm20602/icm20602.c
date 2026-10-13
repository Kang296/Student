#include "icm20602.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ICM20602";

// 寄存器
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C

// I2C 底层
static esp_err_t write_reg(icm20602_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev->i2c_dev, buf, 2, 100);
}

static esp_err_t read_regs(icm20602_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, len, 100);
}

static int16_t make_int16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

// 量程
static float acc_scale_from_range(icm20602_acc_range_t r)
{
    switch (r)
    {
    case ICM20602_ACC_2G:
        return 16384.0f;
    case ICM20602_ACC_4G:
        return 8192.0f;
    case ICM20602_ACC_8G:
        return 4096.0f;
    case ICM20602_ACC_16G:
        return 2048.0f;
    default:
        return 4096.0f;
    }
}

static float gyro_scale_from_range(icm20602_gyro_range_t r)
{
    switch (r)
    {
    case ICM20602_GYRO_250DPS:
        return 131.0f;
    case ICM20602_GYRO_500DPS:
        return 65.5f;
    case ICM20602_GYRO_1000DPS:
        return 32.8f;
    case ICM20602_GYRO_2000DPS:
        return 16.4f;
    default:
        return 16.4f;
    }
}

// 公开 API
void icm20602_init_default_config(icm20602_config_t *config)
{
    config->i2c_address = ICM20602_ADDR_LOW;
    config->acc_range = ICM20602_ACC_8G;
    config->gyro_range = ICM20602_GYRO_2000DPS;
    config->sample_rate_div = 9; // ~100Hz
}

esp_err_t icm20602_init(icm20602_t *dev, i2c_master_dev_handle_t i2c_dev,
                        const icm20602_config_t *config)
{
    if (!dev || !i2c_dev || !config)
        return ESP_ERR_INVALID_ARG;

    memset(dev, 0, sizeof(*dev));
    dev->i2c_dev = i2c_dev;
    dev->i2c_address = config->i2c_address;

    vTaskDelay(pdMS_TO_TICKS(100));

    //  WHO_AM_I 检查
    uint8_t who = 0;
    esp_err_t ret = read_regs(dev, ICM20602_WHO_AM_I_REG, &who, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expect 0x%02X)", who, ICM20602_WHO_AM_I_VAL);

    if (who != ICM20602_WHO_AM_I_VAL)
    {
        ESP_LOGE(TAG, "WHO_AM_I mismatch! Check wiring/address.");
        return ESP_ERR_NOT_FOUND;
    }

    //  软件复位
    ret = write_reg(dev, REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    //  退出睡眠 + 时钟源 = PLL
    ret = write_reg(dev, REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    //  开启加速度计 XYZ + 陀螺仪 XYZ
    ret = write_reg(dev, REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK)
        return ret;

    //  采样率分频
    write_reg(dev, REG_SMPLRT_DIV, config->sample_rate_div);

    //  低通滤波
    write_reg(dev, REG_CONFIG, 0x03);

    //  陀螺仪量程
    write_reg(dev, REG_GYRO_CONFIG, (uint8_t)config->gyro_range);

    //  加速度计量程
    write_reg(dev, REG_ACCEL_CONFIG, (uint8_t)config->acc_range);

    //  加速度计低通滤波
    write_reg(dev, REG_ACCEL_CONFIG2, 0x03);

    //  计算比例因子
    dev->acc_scale = acc_scale_from_range(config->acc_range);
    dev->gyro_scale = gyro_scale_from_range(config->gyro_range);

    dev->initialized = true;
    ESP_LOGI(TAG, "Init OK (acc=%.0f LSB/g, gyro=%.1f LSB/dps)",
             dev->acc_scale, dev->gyro_scale);
    return ESP_OK;
}

esp_err_t icm20602_read_raw(icm20602_t *dev, icm20602_raw_data_t *raw)
{
    if (!dev || !dev->initialized || !raw)
        return ESP_ERR_INVALID_ARG;

    uint8_t buf[14] = {0};
    esp_err_t ret = read_regs(dev, REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "read raw failed");
        return ret;
    }

    raw->acc_x = make_int16(buf[0], buf[1]);
    raw->acc_y = make_int16(buf[2], buf[3]);
    raw->acc_z = make_int16(buf[4], buf[5]);
    raw->temp = make_int16(buf[6], buf[7]);
    raw->gyro_x = make_int16(buf[8], buf[9]);
    raw->gyro_y = make_int16(buf[10], buf[11]);
    raw->gyro_z = make_int16(buf[12], buf[13]);

    return ESP_OK;
}

esp_err_t icm20602_read_data(icm20602_t *dev, icm20602_data_t *data)
{
    icm20602_raw_data_t raw;
    esp_err_t ret = icm20602_read_raw(dev, &raw);
    if (ret != ESP_OK)
        return ret;

    data->acc_x_g = raw.acc_x / dev->acc_scale;
    data->acc_y_g = raw.acc_y / dev->acc_scale;
    data->acc_z_g = raw.acc_z / dev->acc_scale;
    data->gyro_x_dps = raw.gyro_x / dev->gyro_scale;
    data->gyro_y_dps = raw.gyro_y / dev->gyro_scale;
    data->gyro_z_dps = raw.gyro_z / dev->gyro_scale;
    data->temp_c = raw.temp / 326.8f + 25.0f;

    return ESP_OK;
}
