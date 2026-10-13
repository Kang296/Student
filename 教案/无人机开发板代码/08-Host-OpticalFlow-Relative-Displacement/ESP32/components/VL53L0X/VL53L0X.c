#include "VL53L0X.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "VL53L0X";

// 内部宏
// millis() 等效 — 返回启动后毫秒数 (uint32_t, ~49 天回绕)
#define vl53l0x_millis() ((uint32_t)(esp_timer_get_time() / 1000))

// 启动超时计时
#define vl53l0x_start_timeout(dev) ((dev)->timeout_start_ms = vl53l0x_millis())

// 检查超时是否到期
#define vl53l0x_check_timeout(dev) \
    ((dev)->timeout_ms_io > 0 &&   \
     ((uint32_t)(vl53l0x_millis() - (dev)->timeout_start_ms) > (dev)->timeout_ms_io))

// VCSEL 周期编解码
// decodeVcselPeriod: 寄存器值 → PCLK 周期
#define vl53l0x_decode_vcsel_period(reg_val) (((reg_val) + 1) << 1)
// encodeVcselPeriod: PCLK 周期 → 寄存器值
#define vl53l0x_encode_vcsel_period(pclks) (((pclks) >> 1) - 1)

// 宏周期计算 (纳秒)
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define vl53l0x_calc_macro_period_ns(vcsel_pclks) \
    ((((uint32_t)2304 * (vcsel_pclks) * (uint32_t)1655) + 500) / 1000)

// 内部结构体
typedef struct
{
    bool tcc;
    bool msrc;
    bool dss;
    bool pre_range;
    bool final_range;
} sequence_step_enables_t;

typedef struct
{
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
} sequence_step_timeouts_t;

// 内部函数声明
static bool get_spad_info(vl53l0x_t *dev, uint8_t *count, bool *type_is_aperture);
static void get_sequence_step_enables(vl53l0x_t *dev, sequence_step_enables_t *enables);
static void get_sequence_step_timeouts(vl53l0x_t *dev,
                                       const sequence_step_enables_t *enables,
                                       sequence_step_timeouts_t *timeouts);
static bool perform_single_ref_calibration(vl53l0x_t *dev, uint8_t vhv_init_byte);
static uint16_t decode_timeout(uint16_t reg_val);
static uint16_t encode_timeout(uint32_t timeout_mclks);
static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks, uint8_t vcsel_pclks);
static uint32_t timeout_us_to_mclks(uint32_t timeout_us, uint8_t vcsel_pclks);

// I2C 底层通信

static inline void write_reg(vl53l0x_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    dev->last_status = i2c_master_transmit(dev->i2c_dev, buf, 2, (int)dev->timeout_ms);
}

static inline void write_reg16(vl53l0x_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    dev->last_status = i2c_master_transmit(dev->i2c_dev, buf, 3, (int)dev->timeout_ms);
}

static inline void write_reg32(vl53l0x_t *dev, uint8_t reg, uint32_t value)
{
    uint8_t buf[5] = {reg,
                      (uint8_t)(value >> 24),
                      (uint8_t)(value >> 16),
                      (uint8_t)(value >> 8),
                      (uint8_t)(value & 0xFF)};
    dev->last_status = i2c_master_transmit(dev->i2c_dev, buf, 5, (int)dev->timeout_ms);
}

static inline uint8_t read_reg(vl53l0x_t *dev, uint8_t reg)
{
    uint8_t value = 0;
    dev->last_status = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1,
                                                   &value, 1, (int)dev->timeout_ms);
    return value;
}

static inline uint16_t read_reg16(vl53l0x_t *dev, uint8_t reg)
{
    uint8_t buf[2] = {0};
    dev->last_status = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1,
                                                   buf, 2, (int)dev->timeout_ms);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static inline uint32_t read_reg32(vl53l0x_t *dev, uint8_t reg)
{
    uint8_t buf[4] = {0};
    dev->last_status = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1,
                                                   buf, 4, (int)dev->timeout_ms);
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
}

static void write_multi(vl53l0x_t *dev, uint8_t reg, const uint8_t *src, uint8_t count)
{
    // 堆栈上拼装: 寄存器地址 + 数据
    uint8_t buf[32];
    buf[0] = reg;
    if (count > 31)
        count = 31;
    memcpy(buf + 1, src, count);
    dev->last_status = i2c_master_transmit(dev->i2c_dev, buf, (size_t)count + 1,
                                           (int)dev->timeout_ms);
}

static void read_multi(vl53l0x_t *dev, uint8_t reg, uint8_t *dst, uint8_t count)
{
    dev->last_status = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1,
                                                   dst, count, (int)dev->timeout_ms);
}

// 公开 API — 配置
void vl53l0x_init_default_config(vl53l0x_config_t *config)
{
    config->i2c_address = VL53L0X_I2C_ADDRESS_DEFAULT;
    config->timeout_ms = 100;
    config->io_2v8 = true;
    config->interrupt_pin = -1;
}

// 公开 API — 初始化 / 去初始化
esp_err_t vl53l0x_init(vl53l0x_t *dev, i2c_master_dev_handle_t i2c_dev,
                       const vl53l0x_config_t *config)
{
    if (dev == NULL || i2c_dev == NULL || config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 清零设备结构体
    memset(dev, 0, sizeof(vl53l0x_t));

    dev->i2c_dev = i2c_dev;
    dev->i2c_address = config->i2c_address;
    dev->timeout_ms = config->timeout_ms;
    dev->timeout_ms_io = 0; // 无超时
    dev->did_timeout = false;

    // ---- 检查芯片型号 ID ----
    if (read_reg(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID) != VL53L0X_CHIP_ID)
    {
        ESP_LOGE(TAG, "Chip ID mismatch! Expected 0x%02X", VL53L0X_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }

    // ---- VL53L0X_DataInit() ----

    // 如果使用 2.8V I/O 模式 (3.3V 总线需要)
    if (config->io_2v8)
    {
        uint8_t v = read_reg(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
        write_reg(dev, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, v | 0x01);
    }

    // "Set I2C standard mode"
    write_reg(dev, 0x88, 0x00);

    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    dev->stop_variable = read_reg(dev, 0x91);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    // 禁用 SIGNAL_RATE_MSRC (bit 1) 和 SIGNAL_RATE_PRE_RANGE (bit 4) 限值检测
    write_reg(dev, VL53L0X_REG_MSRC_CONFIG_CONTROL,
              read_reg(dev, VL53L0X_REG_MSRC_CONFIG_CONTROL) | 0x12);

    // 设置 final range signal rate limit = 0.25 MCPS
    vl53l0x_set_signal_rate_limit(dev, 0.25f);

    write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // ---- VL53L0X_StaticInit() ----

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!get_spad_info(dev, &spad_count, &spad_type_is_aperture))
    {
        ESP_LOGE(TAG, "Failed to get SPAD info");
        return ESP_ERR_TIMEOUT;
    }

    // 读取 SPAD map
    uint8_t ref_spad_map[6];
    read_multi(dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() --
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_reg(dev, VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad || spads_enabled == spad_count)
        {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }
    write_multi(dev, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_load_tuning_settings() — DefaultTuningSettings --

    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x09, 0x00);
    write_reg(dev, 0x10, 0x00);
    write_reg(dev, 0x11, 0x00);
    write_reg(dev, 0x24, 0x01);
    write_reg(dev, 0x25, 0xFF);
    write_reg(dev, 0x75, 0x00);

    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x4E, 0x2C);
    write_reg(dev, 0x48, 0x00);
    write_reg(dev, 0x30, 0x20);

    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x30, 0x09);
    write_reg(dev, 0x54, 0x00);
    write_reg(dev, 0x31, 0x04);
    write_reg(dev, 0x32, 0x03);
    write_reg(dev, 0x40, 0x83);
    write_reg(dev, 0x46, 0x25);
    write_reg(dev, 0x60, 0x00);
    write_reg(dev, 0x27, 0x00);
    write_reg(dev, 0x50, 0x06);
    write_reg(dev, 0x51, 0x00);
    write_reg(dev, 0x52, 0x96);
    write_reg(dev, 0x56, 0x08);
    write_reg(dev, 0x57, 0x30);
    write_reg(dev, 0x61, 0x00);
    write_reg(dev, 0x62, 0x00);
    write_reg(dev, 0x64, 0x00);
    write_reg(dev, 0x65, 0x00);
    write_reg(dev, 0x66, 0xA0);

    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x22, 0x32);
    write_reg(dev, 0x47, 0x14);
    write_reg(dev, 0x49, 0xFF);
    write_reg(dev, 0x4A, 0x00);

    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x7A, 0x0A);
    write_reg(dev, 0x7B, 0x00);
    write_reg(dev, 0x78, 0x21);

    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x23, 0x34);
    write_reg(dev, 0x42, 0x00);
    write_reg(dev, 0x44, 0xFF);
    write_reg(dev, 0x45, 0x26);
    write_reg(dev, 0x46, 0x05);
    write_reg(dev, 0x40, 0x40);
    write_reg(dev, 0x0E, 0x06);
    write_reg(dev, 0x20, 0x1A);
    write_reg(dev, 0x43, 0x40);

    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x34, 0x03);
    write_reg(dev, 0x35, 0x44);

    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x31, 0x04);
    write_reg(dev, 0x4B, 0x09);
    write_reg(dev, 0x4C, 0x05);
    write_reg(dev, 0x4D, 0x04);

    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x44, 0x00);
    write_reg(dev, 0x45, 0x20);
    write_reg(dev, 0x47, 0x08);
    write_reg(dev, 0x48, 0x28);
    write_reg(dev, 0x67, 0x00);
    write_reg(dev, 0x70, 0x04);
    write_reg(dev, 0x71, 0x01);
    write_reg(dev, 0x72, 0xFE);
    write_reg(dev, 0x76, 0x00);
    write_reg(dev, 0x77, 0x00);

    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x0D, 0x01);

    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0x01, 0xF8);

    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x8E, 0x01);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    // -- VL53L0X_SetGpioConfig() — 新采样就绪中断, active low --
    write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_reg(dev, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH,
              read_reg(dev, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->measurement_timing_budget_us = vl53l0x_get_measurement_timing_budget(dev);

    // 禁用 MSRC 和 TCC
    write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // 重新计算 timing budget
    vl53l0x_set_measurement_timing_budget(dev, dev->measurement_timing_budget_us);

    // ---- VL53L0X_PerformRefCalibration() ----

    // VHV 校准
    write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!perform_single_ref_calibration(dev, 0x40))
    {
        ESP_LOGE(TAG, "VHV calibration timeout");
        return ESP_ERR_TIMEOUT;
    }

    // 相位校准
    write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!perform_single_ref_calibration(dev, 0x00))
    {
        ESP_LOGE(TAG, "Phase calibration timeout");
        return ESP_ERR_TIMEOUT;
    }

    // 恢复 Sequence Config
    write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // ---- 配置 ESP32 侧 GPIO 中断引脚 ----
    if (config->interrupt_pin >= 0)
    {
        esp_err_t ret = vl53l0x_setup_interrupt_pin(config->interrupt_pin);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Interrupt pin setup failed: %d", ret);
        }
    }

    dev->initialized = true;
    ESP_LOGI(TAG, "VL53L0X initialized successfully");
    return ESP_OK;
}

esp_err_t vl53l0x_deinit(vl53l0x_t *dev)
{
    if (dev == NULL)
        return ESP_ERR_INVALID_ARG;
    dev->initialized = false;
    dev->i2c_dev = NULL;
    return ESP_OK;
}

// 公开 API — I2C 地址
esp_err_t vl53l0x_set_address(vl53l0x_t *dev, uint8_t new_addr)
{
    if (dev == NULL || !dev->initialized)
        return ESP_ERR_INVALID_STATE;
    // VL53L0X 只使用 7-bit 地址
    write_reg(dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    if (dev->last_status != ESP_OK)
        return dev->last_status;
    dev->i2c_address = new_addr;
    return ESP_OK;
}

uint8_t vl53l0x_get_address(vl53l0x_t *dev)
{
    if (dev == NULL)
        return 0;
    return dev->i2c_address;
}

// 公开 API — 寄存器读写 (薄封装，与原始命名对应)
void vl53l0x_write_reg(vl53l0x_t *dev, uint8_t reg, uint8_t value) { write_reg(dev, reg, value); }
void vl53l0x_write_reg16(vl53l0x_t *dev, uint8_t reg, uint16_t value) { write_reg16(dev, reg, value); }
void vl53l0x_write_reg32(vl53l0x_t *dev, uint8_t reg, uint32_t value) { write_reg32(dev, reg, value); }
uint8_t vl53l0x_read_reg(vl53l0x_t *dev, uint8_t reg) { return read_reg(dev, reg); }
uint16_t vl53l0x_read_reg16(vl53l0x_t *dev, uint8_t reg) { return read_reg16(dev, reg); }
uint32_t vl53l0x_read_reg32(vl53l0x_t *dev, uint8_t reg) { return read_reg32(dev, reg); }
void vl53l0x_write_multi(vl53l0x_t *dev, uint8_t reg, const uint8_t *s, uint8_t c) { write_multi(dev, reg, s, c); }
void vl53l0x_read_multi(vl53l0x_t *dev, uint8_t reg, uint8_t *d, uint8_t c) { read_multi(dev, reg, d, c); }

// 公开 API — 信号速率限值
bool vl53l0x_set_signal_rate_limit(vl53l0x_t *dev, float limit_mcps)
{
    if (limit_mcps < 0.0f || limit_mcps > 511.99f)
        return false;
    // Q9.7 定点格式 (9 整数位, 7 小数位)
    write_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                (uint16_t)(limit_mcps * (1 << 7)));
    return true;
}

float vl53l0x_get_signal_rate_limit(vl53l0x_t *dev)
{
    return (float)read_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (float)(1 << 7);
}

// 公开 API — 测量时序预算
bool vl53l0x_set_measurement_timing_budget(vl53l0x_t *dev, uint32_t budget_us)
{
    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;

    const uint16_t StartOverhead = 1910;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(dev, &enables);
    get_sequence_step_timeouts(dev, &enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;
        if (used_budget_us > budget_us)
        {
            return false; // 请求的时序预算太小
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint32_t final_range_timeout_mclks =
            timeout_us_to_mclks(final_range_timeout_us,
                                timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        write_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    encode_timeout(final_range_timeout_mclks));

        dev->measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t vl53l0x_get_measurement_timing_budget(vl53l0x_t *dev)
{
    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;

    const uint16_t StartOverhead = 1910;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    get_sequence_step_enables(dev, &enables);
    get_sequence_step_timeouts(dev, &enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us;
    return budget_us;
}

// 公开 API — VCSEL 脉冲周期
bool vl53l0x_set_vcsel_pulse_period(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type,
                                    uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = vl53l0x_encode_vcsel_period(period_pclks);
    sequence_step_enables_t enables;
    sequence_step_timeouts_t timeouts;

    get_sequence_step_enables(dev, &enables);
    get_sequence_step_timeouts(dev, &enables, &timeouts);

    if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE)
    {
        switch (period_pclks)
        {
        case 12:
            write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
            break;
        case 14:
            write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
            break;
        case 16:
            write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
            break;
        case 18:
            write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
            break;
        default:
            return false;
        }
        write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        write_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_pre_range_timeout_mclks =
            timeout_us_to_mclks(timeouts.pre_range_us, period_pclks);
        write_reg16(dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    encode_timeout(new_pre_range_timeout_mclks));

        uint16_t new_msrc_timeout_mclks =
            timeout_us_to_mclks(timeouts.msrc_dss_tcc_us, period_pclks);
        write_reg(dev, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                  (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
    }
    else if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE)
    {
        switch (period_pclks)
        {
        case 8:
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
            write_reg(dev, 0xFF, 0x01);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x30);
            write_reg(dev, 0xFF, 0x00);
            break;
        case 10:
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
            write_reg(dev, 0xFF, 0x01);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
            write_reg(dev, 0xFF, 0x00);
            break;
        case 12:
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
            write_reg(dev, 0xFF, 0x01);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
            write_reg(dev, 0xFF, 0x00);
            break;
        case 14:
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            write_reg(dev, VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
            write_reg(dev, 0xFF, 0x01);
            write_reg(dev, VL53L0X_REG_ALGO_PHASECAL_LIM, 0x20);
            write_reg(dev, 0xFF, 0x00);
            break;
        default:
            return false;
        }
        write_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_final_range_timeout_mclks =
            timeout_us_to_mclks(timeouts.final_range_us, period_pclks);
        if (enables.pre_range)
        {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }
        write_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    encode_timeout(new_final_range_timeout_mclks));
    }
    else
    {
        return false;
    }

    vl53l0x_set_measurement_timing_budget(dev, dev->measurement_timing_budget_us);

    // 更改 VCSEL 周期后需要相位校准
    uint8_t seq_config = read_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);
    write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    perform_single_ref_calibration(dev, 0x00);
    write_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, seq_config);

    return true;
}

uint8_t vl53l0x_get_vcsel_pulse_period(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type)
{
    if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE)
    {
        return vl53l0x_decode_vcsel_period(
            read_reg(dev, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));
    }
    else if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE)
    {
        return vl53l0x_decode_vcsel_period(
            read_reg(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    return 255;
}

// 公开 API — 测距模式
esp_err_t vl53l0x_set_range_mode(vl53l0x_t *dev, uint8_t mode)
{
    if (dev == NULL || !dev->initialized)
        return ESP_ERR_INVALID_STATE;

    switch (mode)
    {
    case 0: // 默认模式
        vl53l0x_set_signal_rate_limit(dev, 0.25f);
        vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
        vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
        vl53l0x_set_measurement_timing_budget(dev, 33000);
        vl53l0x_set_timeout(dev, 500);
        break;

    case 1: // 高速模式
        vl53l0x_set_signal_rate_limit(dev, 0.25f);
        vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 14);
        vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 10);
        vl53l0x_set_measurement_timing_budget(dev, 20000);
        vl53l0x_set_timeout(dev, 500);
        break;

    case 2: // 长距离模式
        vl53l0x_set_signal_rate_limit(dev, 0.10f);
        vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
        vl53l0x_set_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
        vl53l0x_set_measurement_timing_budget(dev, 200000);
        vl53l0x_set_timeout(dev, 1000);
        break;

    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

// 公开 API — 测距
void vl53l0x_start_continuous(vl53l0x_t *dev, uint32_t period_ms)
{
    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    write_reg(dev, 0x91, dev->stop_variable);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    if (period_ms != 0)
    {
        // 连续定时模式
        uint16_t osc_cal = read_reg16(dev, VL53L0X_REG_OSC_CALIBRATE_VAL);
        uint32_t period = period_ms;
        if (osc_cal != 0)
        {
            period *= osc_cal;
        }
        write_reg32(dev, VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, period);
        write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x04); // TIMED
    }
    else
    {
        // 连续背靠背模式
        write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x02); // BACKTOBACK
    }
}

void vl53l0x_stop_continuous(vl53l0x_t *dev)
{
    write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x01); // SINGLESHOT
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    write_reg(dev, 0x91, 0x00);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
}

uint16_t vl53l0x_read_range_continuous_mm(vl53l0x_t *dev)
{
    vl53l0x_start_timeout(dev);
    while ((read_reg(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (vl53l0x_check_timeout(dev))
        {
            dev->did_timeout = true;
            return 65535;
        }
    }

    // 假设线性校正增益 = 1000 (默认), 不使用分数测距
    uint16_t range = read_reg16(dev, VL53L0X_REG_RESULT_RANGE_STATUS + 10);

    write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

uint16_t vl53l0x_read_range_single_mm(vl53l0x_t *dev)
{
    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);
    write_reg(dev, 0x91, dev->stop_variable);
    write_reg(dev, 0x00, 0x01);
    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x01);

    // 等待 start bit 清零
    vl53l0x_start_timeout(dev);
    while (read_reg(dev, VL53L0X_REG_SYSRANGE_START) & 0x01)
    {
        if (vl53l0x_check_timeout(dev))
        {
            dev->did_timeout = true;
            return 65535;
        }
    }

    return vl53l0x_read_range_continuous_mm(dev);
}

// 公开 API — 超时
void vl53l0x_set_timeout(vl53l0x_t *dev, uint16_t timeout_ms)
{
    dev->timeout_ms_io = timeout_ms;
}

uint16_t vl53l0x_get_timeout(vl53l0x_t *dev)
{
    return dev->timeout_ms_io;
}

bool vl53l0x_timeout_occurred(vl53l0x_t *dev)
{
    bool tmp = dev->did_timeout;
    dev->did_timeout = false;
    return tmp;
}

// 公开 API — 中断引脚
esp_err_t vl53l0x_setup_interrupt_pin(int gpio_num)
{
    // VL53L0X GPIO1: 新采样就绪, active low
    // ESP32 侧: 输入 + 上拉, 下降沿检测由用户自行配置 ISR
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, // 下降沿 — 新数据就绪
    };
    return gpio_config(&io_conf);
}

// 内部函数实现
static bool get_spad_info(vl53l0x_t *dev, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;

    write_reg(dev, 0x80, 0x01);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x00);

    write_reg(dev, 0xFF, 0x06);
    write_reg(dev, 0x83, read_reg(dev, 0x83) | 0x04);
    write_reg(dev, 0xFF, 0x07);
    write_reg(dev, 0x81, 0x01);

    write_reg(dev, 0x80, 0x01);

    write_reg(dev, 0x94, 0x6b);
    write_reg(dev, 0x83, 0x00);

    vl53l0x_start_timeout(dev);
    while (read_reg(dev, 0x83) == 0x00)
    {
        if (vl53l0x_check_timeout(dev))
            return false;
    }

    write_reg(dev, 0x83, 0x01);
    tmp = read_reg(dev, 0x92);

    *count = tmp & 0x7F;
    *type_is_aperture = (tmp >> 7) & 0x01;

    write_reg(dev, 0x81, 0x00);
    write_reg(dev, 0xFF, 0x06);
    write_reg(dev, 0x83, read_reg(dev, 0x83) & ~0x04);
    write_reg(dev, 0xFF, 0x01);
    write_reg(dev, 0x00, 0x01);

    write_reg(dev, 0xFF, 0x00);
    write_reg(dev, 0x80, 0x00);

    return true;
}

static void get_sequence_step_enables(vl53l0x_t *dev, sequence_step_enables_t *enables)
{
    uint8_t seq_config = read_reg(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);
    enables->tcc = (seq_config >> 4) & 0x1;
    enables->dss = (seq_config >> 3) & 0x1;
    enables->msrc = (seq_config >> 2) & 0x1;
    enables->pre_range = (seq_config >> 6) & 0x1;
    enables->final_range = (seq_config >> 7) & 0x1;
}

static void get_sequence_step_timeouts(vl53l0x_t *dev,
                                       const sequence_step_enables_t *enables,
                                       sequence_step_timeouts_t *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks =
        vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE);

    timeouts->msrc_dss_tcc_mclks = read_reg(dev, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
        timeout_mclks_to_us(timeouts->msrc_dss_tcc_mclks,
                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks =
        decode_timeout(read_reg16(dev, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
        timeout_mclks_to_us(timeouts->pre_range_mclks,
                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks =
        vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE);

    timeouts->final_range_mclks =
        decode_timeout(read_reg16(dev, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
        timeout_mclks_to_us(timeouts->final_range_mclks,
                            timeouts->final_range_vcsel_period_pclks);
}

static bool perform_single_ref_calibration(vl53l0x_t *dev, uint8_t vhv_init_byte)
{
    write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x01 | vhv_init_byte);

    vl53l0x_start_timeout(dev);
    while ((read_reg(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (vl53l0x_check_timeout(dev))
            return false;
    }

    write_reg(dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_reg(dev, VL53L0X_REG_SYSRANGE_START, 0x00);

    return true;
}

// 内部函数 — 超时编解码
static uint16_t decode_timeout(uint16_t reg_val)
{
    // 格式: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    // 格式: "(LSByte * 2^MSByte) + 1"
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFF));
    }
    return 0;
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks, uint8_t vcsel_pclks)
{
    uint32_t macro_period_ns = vl53l0x_calc_macro_period_ns(vcsel_pclks);
    return ((timeout_mclks * macro_period_ns) + 500) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_us, uint8_t vcsel_pclks)
{
    uint32_t macro_period_ns = vl53l0x_calc_macro_period_ns(vcsel_pclks);
    return (((timeout_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}
