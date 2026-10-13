#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"

static const char *TAG = "BLE_UART";
static uint16_t conn_handle = 0;
static const char *DEVICE_NAME = "ESP32-S3-BLE";

static void ble_advertise(void);
static void ble_on_sync(void);
static void ble_on_reset(int reason);

static int gatt_svr_rx_access_cb(uint16_t conn, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt == NULL || ctxt->om == NULL)
    {
        return 0;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        ESP_LOGI(TAG, "recv %d bytes: %.*s",
                 ctxt->om->om_len,
                 ctxt->om->om_len,
                 ctxt->om->om_data);
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID128_DECLARE(
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x11),
                .access_cb = gatt_svr_rx_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {0}},
    },
    {0}};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected");
        }
        else
        {
            conn_handle = 0;
            ble_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = 0;
        ESP_LOGI(TAG, "BLE disconnected");
        ble_advertise();
        break;
    default:
        break;
    }
    return 0;
}

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv = {0};
    struct ble_hs_adv_fields adv_fields = {0};

    // 设置广播名称（扫描响应数据包）
    adv_fields.name = (uint8_t *)DEVICE_NAME;
    adv_fields.name_len = strlen(DEVICE_NAME);
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    ble_gap_adv_set_fields(&adv_fields);

    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    // 合法广播间隔：100ms ~150ms
    adv.itvl_min = BLE_GAP_ADV_ITVL_MS(100);
    adv.itvl_max = BLE_GAP_ADV_ITVL_MS(150);

    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv, ble_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "adv start failed,rc=%d", rc);
    }
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "Nimble host reset, reason=%d", reason);
}

static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "Nimble host synced,start advertise");
    ble_advertise();
}

static void nimble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_LOGI(TAG, "app_main start");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed rc=%d", rc);
        return;
    }

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "BLE UART ready");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
