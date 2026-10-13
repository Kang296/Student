#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 该芯片的I2C 地址 (7-bit)
#define VL53L0X_I2C_ADDRESS_DEFAULT  0x29   // 7-bit address

// 芯片 ID
#define VL53L0X_CHIP_ID              0xEE  

// 寄存器地址 
typedef enum {
    VL53L0X_REG_SYSRANGE_START                              = 0x00,
    VL53L0X_REG_SYSTEM_THRESH_HIGH                          = 0x0C,
    VL53L0X_REG_SYSTEM_THRESH_LOW                           = 0x0E,
    VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    VL53L0X_REG_SYSTEM_RANGE_CONFIG                         = 0x09,
    VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,
    VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,
    VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,
    VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                      = 0x0B,
    VL53L0X_REG_RESULT_INTERRUPT_STATUS                     = 0x13,
    VL53L0X_REG_RESULT_RANGE_STATUS                         = 0x14,
    VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
    VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
    VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
    VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
    VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,
    VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,
    VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,
    VL53L0X_REG_MSRC_CONFIG_CONTROL                         = 0x60,
    VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
    VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
    VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
    VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,
    VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
    VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
    VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
    VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
    VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,
    VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
    VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,
    VL53L0X_REG_SYSTEM_HISTOGRAM_BIN                        = 0x81,
    VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
    VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,
    VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
    VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
    VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,
    VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,
    VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
    VL53L0X_REG_IDENTIFICATION_MODEL_ID                     = 0xC0,
    VL53L0X_REG_IDENTIFICATION_REVISION_ID                  = 0xC2,
    VL53L0X_REG_OSC_CALIBRATE_VAL                           = 0xF8,
    VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
    VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,
    VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
    VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,
    VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,
    VL53L0X_REG_ALGO_PHASECAL_LIM                           = 0x30,
    VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
} vl53l0x_reg_t;

// VCSEL 脉冲周期类型
typedef enum {
    VL53L0X_VCSEL_PERIOD_PRE_RANGE   = 0,
    VL53L0X_VCSEL_PERIOD_FINAL_RANGE = 1,
} vl53l0x_vcsel_period_type_t;

// 配置的结构体 
typedef struct {
    uint8_t  i2c_address;    // I2C 从机地址 (7-bit)，默认 0x29
    uint32_t timeout_ms;     // I2C 超时时间 (ms)，默认 100
    bool     io_2v8;         // 使用 2.8V I/O 模式，默认 true (3.3V 总线)
    int      interrupt_pin;  // 中断引脚 (GPIO1)，-1 表示不配置
} vl53l0x_config_t;

// 设备句柄结构体
typedef struct {
    i2c_master_dev_handle_t i2c_dev;     // I2C 设备句柄
    uint8_t                 i2c_address; // 当前 I2C 地址
    uint32_t                timeout_ms;  // I2C 超时
    esp_err_t               last_status; // 最后一次 I2C 传输状态

    //来自传感器的状态
    uint8_t  stop_variable;
    uint32_t measurement_timing_budget_us;

    // 超时跟踪
    uint16_t timeout_ms_io;    // I/O 操作超时 (0 = 不限)
    bool     did_timeout;
    uint32_t timeout_start_ms;

    // 初始化标记
    bool     initialized;
} vl53l0x_t;

// API 函数
/**
 * @brief 填充默认配置
 */
void vl53l0x_init_default_config(vl53l0x_config_t *config);

/**
 * @brief 初始化 VL53L0X 传感器
 *
 * 调用前需要先初始化 I2C 总线并创建 dev_handle (通过 i2c_master_probe 或
 * i2c_master_bus_add_device)
 *
 * @param dev       设备句柄指针
 * @param i2c_dev   I2C 设备句柄 (由 i2c_master_bus_add_device 创建)
 * @param config    配置参数
 * @return ESP_OK 成功，否则失败
 */
esp_err_t vl53l0x_init(vl53l0x_t *dev, i2c_master_dev_handle_t i2c_dev,
                       const vl53l0x_config_t *config);

/**
 * @brief 去初始化
 */
esp_err_t vl53l0x_deinit(vl53l0x_t *dev);

/**
 * @brief 修改 I2C 地址 (同时写入传感器寄存器)
 */
esp_err_t vl53l0x_set_address(vl53l0x_t *dev, uint8_t new_addr);

/**
 * @brief 获取当前 I2C 地址
 */
uint8_t vl53l0x_get_address(vl53l0x_t *dev);

// ---- 寄存器级读写 ----

void vl53l0x_write_reg(vl53l0x_t *dev, uint8_t reg, uint8_t value);
void vl53l0x_write_reg16(vl53l0x_t *dev, uint8_t reg, uint16_t value);
void vl53l0x_write_reg32(vl53l0x_t *dev, uint8_t reg, uint32_t value);
uint8_t  vl53l0x_read_reg(vl53l0x_t *dev, uint8_t reg);
uint16_t vl53l0x_read_reg16(vl53l0x_t *dev, uint8_t reg);
uint32_t vl53l0x_read_reg32(vl53l0x_t *dev, uint8_t reg);
void vl53l0x_write_multi(vl53l0x_t *dev, uint8_t reg, const uint8_t *src, uint8_t count);
void vl53l0x_read_multi(vl53l0x_t *dev, uint8_t reg, uint8_t *dst, uint8_t count);

// ---- 高级参数设置 ----

bool vl53l0x_set_signal_rate_limit(vl53l0x_t *dev, float limit_mcps);
float vl53l0x_get_signal_rate_limit(vl53l0x_t *dev);

bool vl53l0x_set_measurement_timing_budget(vl53l0x_t *dev, uint32_t budget_us);
uint32_t vl53l0x_get_measurement_timing_budget(vl53l0x_t *dev);

bool vl53l0x_set_vcsel_pulse_period(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type,
                                    uint8_t period_pclks);
uint8_t vl53l0x_get_vcsel_pulse_period(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type);

// ---- 测距模式 ----

/**
 * @brief 切换测距模式
 *
 * @param mode  0=默认, 1=高速, 2=长距离
 * @return ESP_OK 成功
 */
esp_err_t vl53l0x_set_range_mode(vl53l0x_t *dev, uint8_t mode);

// ---- 测距操作 ----

void vl53l0x_start_continuous(vl53l0x_t *dev, uint32_t period_ms);
void vl53l0x_stop_continuous(vl53l0x_t *dev);
uint16_t vl53l0x_read_range_continuous_mm(vl53l0x_t *dev);
uint16_t vl53l0x_read_range_single_mm(vl53l0x_t *dev);

// ---- 超时 ----

void vl53l0x_set_timeout(vl53l0x_t *dev, uint16_t timeout_ms);
uint16_t vl53l0x_get_timeout(vl53l0x_t *dev);
bool vl53l0x_timeout_occurred(vl53l0x_t *dev);

// ---- 中断引脚配置 (GPIO1 → ESP32 GPIO1) ----

/**
 * @brief 配置 ESP32 侧 GPIO 中断引脚
 *
 * VL53L0X 的 GPIO1 设为"新采样就绪"模式 (active low)
 * ESP32 GPIO1 配置为输入上拉 + 下降沿中断
 */
esp_err_t vl53l0x_setup_interrupt_pin(int gpio_num);

#ifdef __cplusplus
}
#endif
