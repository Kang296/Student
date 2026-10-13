#include "PMW3901.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static const char *TAG = "PMW3901";

#define FRAME_TIMEOUT_US 100000 // 帧捕获等待超时 100ms

// 全局状态
static spi_device_handle_t g_spi_dev = NULL;
static bool g_initialized = false;

// SPI 底层读写
static esp_err_t register_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg | 0x80u, value};
    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx,
    };
    esp_err_t ret = spi_device_transmit(g_spi_dev, &trans);
    ets_delay_us(200); // 寄存器间最小间隔
    return ret;
}

static uint8_t register_read(uint8_t reg)
{
    uint8_t tx[2] = {reg & ~0x80u, 0x00};
    uint8_t rx[2] = {0};
    spi_transaction_t trans = {
        .length = 16,
        .rxlength = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(g_spi_dev, &trans);
    ets_delay_us(200);
    return rx[1];
}

// 性能优化寄存器初始化
static void init_registers(void)
{
    register_write(0x7F, 0x00);
    register_write(0x61, 0xAD);
    register_write(0x7F, 0x03);
    register_write(0x40, 0x00);
    register_write(0x7F, 0x05);
    register_write(0x41, 0xB3);
    register_write(0x43, 0xF1);
    register_write(0x45, 0x14);
    register_write(0x5B, 0x32);
    register_write(0x5F, 0x34);
    register_write(0x7B, 0x08);
    register_write(0x7F, 0x06);
    register_write(0x44, 0x1B);
    register_write(0x40, 0xBF);
    register_write(0x4E, 0x3F);
    register_write(0x7F, 0x08);
    register_write(0x65, 0x20);
    register_write(0x6A, 0x18);
    register_write(0x7F, 0x09);
    register_write(0x4F, 0xAF);
    register_write(0x5F, 0x40);
    register_write(0x48, 0x80);
    register_write(0x49, 0x80);
    register_write(0x57, 0x77);
    register_write(0x60, 0x78);
    register_write(0x61, 0x78);
    register_write(0x62, 0x08);
    register_write(0x63, 0x50);
    register_write(0x7F, 0x0A);
    register_write(0x45, 0x60);
    register_write(0x7F, 0x00);
    register_write(0x4D, 0x11);
    register_write(0x55, 0x80);
    register_write(0x74, 0x1F);
    register_write(0x75, 0x1F);
    register_write(0x4A, 0x78);
    register_write(0x4B, 0x78);
    register_write(0x44, 0x08);
    register_write(0x45, 0x50);
    register_write(0x64, 0xFF);
    register_write(0x65, 0x1F);
    register_write(0x7F, 0x14);
    register_write(0x65, 0x60);
    register_write(0x66, 0x08);
    register_write(0x63, 0x78);
    register_write(0x7F, 0x15);
    register_write(0x48, 0x58);
    register_write(0x7F, 0x07);
    register_write(0x41, 0x0D);
    register_write(0x43, 0x14);
    register_write(0x4B, 0x0E);
    register_write(0x45, 0x0F);
    register_write(0x44, 0x42);
    register_write(0x4C, 0x80);
    register_write(0x7F, 0x10);
    register_write(0x5B, 0x02);
    register_write(0x7F, 0x07);
    register_write(0x40, 0x41);
    register_write(0x70, 0x00);

    vTaskDelay(pdMS_TO_TICKS(100));

    register_write(0x32, 0x44);
    register_write(0x7F, 0x07);
    register_write(0x40, 0x40);
    register_write(0x7F, 0x06);
    register_write(0x62, 0xF0);
    register_write(0x63, 0x00);
    register_write(0x7F, 0x0D);
    register_write(0x48, 0xC0);
    register_write(0x6F, 0xD5);
    register_write(0x7F, 0x00);
    register_write(0x5B, 0xA0);
    register_write(0x4E, 0xA8);
    register_write(0x5A, 0x50);
    register_write(0x40, 0x80);
}

// 公开 API
esp_err_t pmw3901_init(spi_device_handle_t spi_dev)
{
    if (g_initialized)
        return ESP_OK;
    if (!spi_dev)
        return ESP_ERR_INVALID_ARG;

    g_spi_dev = spi_dev;

    //  SPI 总线复位时序 (原库: CS toggle)
    {
        uint8_t dummy = 0;
        spi_transaction_t trans = {
            .length = 8,
            .tx_buffer = &dummy,
        };
        spi_device_transmit(g_spi_dev, &trans);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    //  上电复位
    esp_err_t ret = register_write(0x3A, 0x5A);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Power-on reset write failed");
        g_spi_dev = NULL;
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    //  芯片 ID 检查
    uint8_t chip_id = register_read(0x00);
    uint8_t inv_id = register_read(0x5F);

    ESP_LOGI(TAG, "Chip ID: 0x%02X (expect 0x49), Inverse: 0x%02X (~0x49=0xB6)",
             chip_id, inv_id);

    if (chip_id != 0x49 && inv_id != 0xB8)
    {
        ESP_LOGE(TAG, "Chip ID mismatch! Check wiring.");
        g_spi_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    //  预读 Motion 寄存器 (清空残留数据)
    register_read(0x02);
    register_read(0x03);
    register_read(0x04);
    register_read(0x05);
    register_read(0x06);
    vTaskDelay(pdMS_TO_TICKS(1));

    //  性能优化寄存器
    init_registers();

    g_initialized = true;
    ESP_LOGI(TAG, "PMW3901 init OK");
    return ESP_OK;
}

esp_err_t pmw3901_read_motion(int16_t *dx, int16_t *dy)
{
    if (!g_initialized || !dx || !dy)
        return ESP_ERR_INVALID_STATE;

    // 读 0x02 触发 motion 寄存器锁存 + 清零
    register_read(0x02);

    // 读 Delta_X (0x03 low, 0x04 high)
    uint8_t xh = register_read(0x04);
    uint8_t xl = register_read(0x03);
    *dx = (int16_t)(((uint16_t)xh << 8) | xl);

    // 读 Delta_Y (0x05 low, 0x06 high)
    uint8_t yh = register_read(0x06);
    uint8_t yl = register_read(0x05);
    *dy = (int16_t)(((uint16_t)yh << 8) | yl);

    return ESP_OK;
}

esp_err_t pmw3901_read_squal(uint8_t *squal)
{
    if (!g_initialized || !squal)
        return ESP_ERR_INVALID_STATE;

    // 0x55 = Surface_Quality (SQUAL), 图像质量 0~255
    *squal = register_read(0x07);
    return ESP_OK;
}

esp_err_t pmw3901_set_led(bool led_on)
{
    if (!g_initialized)
        return ESP_ERR_INVALID_STATE;

    vTaskDelay(pdMS_TO_TICKS(200));
    register_write(0x7F, 0x14);
    register_write(0x6F, led_on ? 0x1C : 0x00);
    register_write(0x7F, 0x00);

    return ESP_OK;
}

esp_err_t pmw3901_enable_frame_buffer(void)
{
    if (!g_initialized)
        return ESP_ERR_INVALID_STATE;

    register_write(0x7F, 0x07);
    register_write(0x41, 0x1D);
    register_write(0x4C, 0x00);
    register_write(0x7F, 0x08);
    register_write(0x6A, 0x38);
    register_write(0x7F, 0x00);
    register_write(0x55, 0x04);
    register_write(0x40, 0x80);
    register_write(0x4D, 0x11);
    register_write(0x70, 0x00);
    register_write(0x58, 0xFF);

    // 等待帧捕获就绪
    int check;
    do
    {
        check = register_read(0x58) >> 6;
    } while (check == 0x03);

    ets_delay_us(50);
    return ESP_OK;
}

esp_err_t pmw3901_read_frame_buffer(uint8_t *buffer)
{
    if (!g_initialized || !buffer)
        return ESP_ERR_INVALID_STATE;

    int count = 0;
    uint8_t mask = 0x0C;

    for (int i = 0; i < 1225; i++)
    {
        uint8_t a, hold;
        // 等待有效数据
        do
        {
            a = register_read(0x58);
            hold = a >> 6;
        } while (hold == 0x00 || hold == 0x03);

        if (hold == 0x01)
        {
            uint8_t b = register_read(0x58);
            uint8_t pixel = (uint8_t)(a << 2) | (b & mask);
            buffer[count++] = pixel;
        }
    }

    // 恢复并等待下一次就绪
    register_write(0x70, 0x00);
    register_write(0x58, 0xFF);

    int check;
    do
    {
        check = register_read(0x58) >> 6;
    } while (check == 0x03);

    return ESP_OK;
}
