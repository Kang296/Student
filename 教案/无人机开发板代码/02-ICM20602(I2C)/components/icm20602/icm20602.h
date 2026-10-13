#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

// I2C 地址
#define ICM20602_ADDR_LOW     0x68    // SA0=GND
#define ICM20602_ADDR_HIGH    0x69    // SA0=VDD

// WHO_AM_I 期望值
#define ICM20602_WHO_AM_I_REG 0x75
#define ICM20602_WHO_AM_I_VAL 0x12

  
// 原始数据 (ADC 值)
typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20602_raw_data_t;

  
// 物理量数据
typedef struct {
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float temp_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} icm20602_data_t;

  
// 量程枚举
typedef enum {
    ICM20602_ACC_2G  = 0x00,   // ±2g
    ICM20602_ACC_4G  = 0x08,   // ±4g
    ICM20602_ACC_8G  = 0x10,   // ±8g
    ICM20602_ACC_16G = 0x18,   // ±16g
} icm20602_acc_range_t;

typedef enum {
    ICM20602_GYRO_250DPS  = 0x00,   // ±250 dps
    ICM20602_GYRO_500DPS  = 0x08,   // ±500 dps
    ICM20602_GYRO_1000DPS = 0x10,   // ±1000 dps
    ICM20602_GYRO_2000DPS = 0x18,   // ±2000 dps
} icm20602_gyro_range_t;

  
// 配置
typedef struct {
    uint8_t             i2c_address;    // 默认 0x68
    icm20602_acc_range_t  acc_range;    // 默认 ±8g
    icm20602_gyro_range_t gyro_range;   // 默认 ±2000dps
    uint8_t             sample_rate_div; // 默认 9 → ~100Hz
} icm20602_config_t;

  
// 设备句柄
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t                 i2c_address;
    float                   acc_scale;   // LSB/g
    float                   gyro_scale;  // LSB/(dps)
    bool                    initialized;
} icm20602_t;

// API

void icm20602_init_default_config(icm20602_config_t *config);

esp_err_t icm20602_init(icm20602_t *dev, i2c_master_dev_handle_t i2c_dev,
                        const icm20602_config_t *config);

esp_err_t icm20602_read_raw(icm20602_t *dev, icm20602_raw_data_t *raw);
esp_err_t icm20602_read_data(icm20602_t *dev, icm20602_data_t *data);

#ifdef __cplusplus
}
#endif
