// 用法:
//   spi_device_handle_t spi_dev;
//   spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev);  // 用户在 main.c 配置引脚
//   pmw3901_init(spi_dev);

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 PMW3901 光流传感器
 * @param spi_dev  SPI 设备句柄 (用户预先创建, CS/模式/速率由用户配置)
 * @return ESP_OK 成功
 */
esp_err_t pmw3901_init(spi_device_handle_t spi_dev);

/**
 * @brief 读取运动增量 (自动清除内部计数器)
 *
 * @param dx  X 方向位移
 * @param dy  Y 方向位移
 * @return ESP_OK 成功
 */
esp_err_t pmw3901_read_motion(int16_t *dx, int16_t *dy);

/**
 * @brief 读取图像质量 SQUAL (表面质量, 0~255, 越大说明表面纹理越丰富、成像越好)
 *
 * @param squal 输出图像质量值
 * @return ESP_OK 成功
 */
esp_err_t pmw3901_read_squal(uint8_t *squal);

/**
 * @brief 开启/关闭 LED 照明
 */
esp_err_t pmw3901_set_led(bool led_on);

/**
 * @brief 使能帧缓冲区捕获模式
 */
esp_err_t pmw3901_enable_frame_buffer(void);

/**
 * @brief 读取一帧图像 (35x35 = 1225 像素灰度)
 *
 * 调用前需要先 pmw3901_enable_frame_buffer()
 *
 * @param buffer 至少 1225 字节
 * @return ESP_OK 成功
 */
esp_err_t pmw3901_read_frame_buffer(uint8_t *buffer);

#ifdef __cplusplus
}
#endif
