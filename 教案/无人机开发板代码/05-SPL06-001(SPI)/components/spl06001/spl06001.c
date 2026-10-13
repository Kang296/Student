// SPL06 气压计 — SPI 驱动

#include "spl06001.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"

static const char *TAG = "SPL06";

#define SPL06_REG_PRS_B2 0x00
#define SPL06_REG_TMP_B2 0x03
#define SPL06_REG_PRS_CFG 0x06
#define SPL06_REG_TMP_CFG 0x07
#define SPL06_REG_MEAS_CFG 0x08
#define SPL06_REG_CFG_REG 0x09
#define SPL06_REG_RESET 0x0C
#define SPL06_REG_ID 0x0D
#define SPL06_REG_COEF 0x10

#define SPL06_ID_EXPECTED 0x10
#define SPL06_PRS_CFG_VALUE   0x03
#define SPL06_TMP_CFG_VALUE   0x83
#define SPL06_MEAS_CFG_VALUE  0x07

#define SPL06_SCALE_FACTOR_8X 7864320.0f

static spi_device_handle_t spl06_spi = NULL;

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06_coef_t;

static spl06_coef_t g_coef;

static esp_err_t spl06_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = { reg & 0x7F, value };
    spi_transaction_t trans = {
        .length    = 8 * sizeof(tx_data),
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(spl06_spi, &trans);
}

static esp_err_t spl06_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len == 0 || len > 32) return ESP_ERR_INVALID_ARG;

    uint8_t tx_data[33] = {0};
    uint8_t rx_data[33] = {0};
    tx_data[0] = reg | 0x80;

    spi_transaction_t trans = {
        .length    = 8 * (len + 1),
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_transmit(spl06_spi, &trans);
    if (ret != ESP_OK) return ret;

    memcpy(data, &rx_data[1], len);
    return ESP_OK;
}

static esp_err_t spl06_read_reg(uint8_t reg, uint8_t *value)
{
    return spl06_read_regs(reg, value, 1);
}

static int32_t sign_extend(uint32_t raw, uint8_t bits)
{
    uint32_t sign_bit = 1U << (bits - 1);
    if (raw & sign_bit) raw |= ~((1U << bits) - 1);
    return (int32_t)raw;
}

static esp_err_t spl06_read_raw24(uint8_t start_reg, int32_t *raw)
{
    uint8_t buf[3] = {0};
    esp_err_t ret = spl06_read_regs(start_reg, buf, 3);
    if (ret != ESP_OK) return ret;

    uint32_t value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    *raw = sign_extend(value, 24);
    return ESP_OK;
}

static esp_err_t spl06_read_calibration_data(void)
{
    uint8_t coef[18] = {0};
    esp_err_t ret = spl06_read_regs(SPL06_REG_COEF, coef, 18);
    if (ret != ESP_OK) return ret;

    g_coef.c0  = sign_extend(((uint32_t)coef[0] << 4) | ((uint32_t)coef[1] >> 4), 12);
    g_coef.c1  = sign_extend((((uint32_t)coef[1] & 0x0F) << 8) | coef[2], 12);
    g_coef.c00 = sign_extend(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | ((uint32_t)coef[5] >> 4), 20);
    g_coef.c10 = sign_extend((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | coef[7], 20);
    g_coef.c01 = sign_extend(((uint32_t)coef[8] << 8) | coef[9], 16);
    g_coef.c11 = sign_extend(((uint32_t)coef[10] << 8) | coef[11], 16);
    g_coef.c20 = sign_extend(((uint32_t)coef[12] << 8) | coef[13], 16);
    g_coef.c21 = sign_extend(((uint32_t)coef[14] << 8) | coef[15], 16);
    g_coef.c30 = sign_extend(((uint32_t)coef[16] << 8) | coef[17], 16);

    ESP_LOGI(TAG, "Calibration data loaded");
    ESP_LOGI(TAG, "c0=%d, c1=%d, c00=%ld, c10=%ld",
             g_coef.c0, g_coef.c1, g_coef.c00, g_coef.c10);
    return ESP_OK;
}

static esp_err_t spl06_wait_ready(void)
{
    for (int i = 0; i < 50; i++) {
        uint8_t status = 0;
        esp_err_t ret = spl06_read_reg(SPL06_REG_MEAS_CFG, &status);
        if (ret != ESP_OK) return ret;

        if ((status & 0xC0) == 0xC0) {
            ESP_LOGI(TAG, "Sensor ready, MEAS_CFG=0x%02X", status);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "Sensor ready timeout");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t spl06_wait_data_ready(void)
{
    for (int i = 0; i < 50; i++) {
        uint8_t status = 0;
        esp_err_t ret = spl06_read_reg(SPL06_REG_MEAS_CFG, &status);
        if (ret != ESP_OK) return ret;

        if ((status & 0x30) == 0x30) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "Data ready timeout");
    return ESP_ERR_TIMEOUT;
}

esp_err_t spl06_init(spi_device_handle_t spi_dev)
{
    if (!spi_dev) return ESP_ERR_INVALID_ARG;
    spl06_spi = spi_dev;

    esp_err_t ret;

    uint8_t id = 0;
    ret = spl06_read_reg(SPL06_REG_ID, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read ID failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPL06 ID = 0x%02X", id);

    if (id != SPL06_ID_EXPECTED) {
        ESP_LOGW(TAG, "ID mismatch: got 0x%02X, expected 0x10. Will try to use anyway.", id);
    }

    ret = spl06_write_reg(SPL06_REG_RESET, 0x09);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    ret = spl06_wait_ready();
    if (ret != ESP_OK) return ret;

    ret = spl06_read_calibration_data();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read calibration data failed");
        return ret;
    }

    spl06_write_reg(SPL06_REG_PRS_CFG, SPL06_PRS_CFG_VALUE);
    spl06_write_reg(SPL06_REG_TMP_CFG, SPL06_TMP_CFG_VALUE);
    spl06_write_reg(SPL06_REG_CFG_REG, 0x00);
    spl06_write_reg(SPL06_REG_MEAS_CFG, SPL06_MEAS_CFG_VALUE);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "SPL06 init done");
    return ESP_OK;
}

esp_err_t spl06_read(spl06_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = spl06_wait_data_ready();
    if (ret != ESP_OK) return ret;

    int32_t raw_temp = 0, raw_pressure = 0;
    ret = spl06_read_raw24(SPL06_REG_TMP_B2, &raw_temp);
    if (ret != ESP_OK) return ret;
    ret = spl06_read_raw24(SPL06_REG_PRS_B2, &raw_pressure);
    if (ret != ESP_OK) return ret;

    float Traw_sc = raw_temp / SPL06_SCALE_FACTOR_8X;
    float Praw_sc = raw_pressure / SPL06_SCALE_FACTOR_8X;

    float temperature_c = g_coef.c0 * 0.5f + g_coef.c1 * Traw_sc;
    float pressure_pa =
        g_coef.c00 +
        Praw_sc * (g_coef.c10 + Praw_sc * (g_coef.c20 + Praw_sc * g_coef.c30)) +
        Traw_sc * g_coef.c01 +
        Traw_sc * Praw_sc * (g_coef.c11 + Praw_sc * g_coef.c21);

    data->raw_temperature = raw_temp;
    data->raw_pressure    = raw_pressure;
    data->temperature_c   = temperature_c;
    data->pressure_pa     = pressure_pa;
    data->pressure_hpa    = pressure_pa / 100.0f;
    data->altitude_m      = spl06_pressure_to_altitude(pressure_pa);

    return ESP_OK;
}

float spl06_pressure_to_altitude(float pressure_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.19029495f));
}
