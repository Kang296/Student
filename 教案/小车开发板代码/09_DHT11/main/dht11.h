#ifndef __DHT11_H
#define __DHT11_H

#include "driver/gpio.h"
#include "esp_log.h"

/* 引脚定义 */
#define DHT11_DQ_GPIO_PIN GPIO_NUM_42

/* IO操作 */
#define DHT11_DQ_IN gpio_get_level(DHT11_DQ_GPIO_PIN)

/* DHT11端口定义 */
#define DHT11_DQ_OUT(x)                                                              \
  do                                                                                 \
  {                                                                                  \
    x ? gpio_set_level(DHT11_DQ_GPIO_PIN, 1) : gpio_set_level(DHT11_DQ_GPIO_PIN, 0); \
  } while (0)

/* 函数声明 */
void dht11_reset(void);                            /* 复位DHT11 */
uint8_t dht11_init(void);                          /* 初始化DHT11 */
uint8_t dht11_check(void);                         /* 等待DHT11的回应 */
uint8_t dht11_read_data(short *temp, short *humi); /* 读取温湿度 */

#endif