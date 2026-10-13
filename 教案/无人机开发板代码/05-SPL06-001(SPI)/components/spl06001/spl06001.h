#ifndef SPL06001_H
#define SPL06001_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"

typedef struct // spl06_data_t是SPL专用数据存储载体
{
    int32_t raw_temperature; // 传感器读取到的的原始温度ADC采样值
    int32_t raw_pressure;    // 存储传感器输出的原始气压ADC裸数据

    float temperature_c; // 精确过后得到  单位：摄氏度℃
    float pressure_pa;   // 原始气压换算为标准帕斯卡（Pa）
    float pressure_hpa;  // 气压换算成百帕hPa
    float altitude_m;    // 海拔
} spl06_data_t;          // spl06_data_t(结构体名称)是SPL专用数据存储载体里面有原始的，经过计算的温度，气压的数据

/**
 * @brief 初始化 SPL06 气压计
 * @param spi_dev  SPI 设备句柄 (用户预先创建, CS/模式/速率由用户配置)
 */
esp_err_t spl06_init(spi_device_handle_t spi_dev);
esp_err_t spl06_read(spl06_data_t *data);

float spl06_pressure_to_altitude(float pressure_pa); // 传入参数，由当前的气压参数算出海拔

#endif
