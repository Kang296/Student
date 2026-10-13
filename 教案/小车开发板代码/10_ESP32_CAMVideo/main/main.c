#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_pins.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"

/* ========== WiFi 配置（根据实际网络修改） ========== */
#define WIFI_SSID "pura70pro+"
#define WIFI_PASS "12345678"
#define WIFI_MAX_RETRY 5

static int s_retry_num = 0;

/* ========== MJPEG 流参数 ========== */
#define STREAM_BOUNDARY "jpgeofr"
#define STREAM_CONTENT "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY

/* ========== 摄像头初始化 ========== */
esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn = CAM_PWDN,
        .pin_reset = CAM_RESET,
        .pin_xclk = CAM_XCLK,
        .pin_sccb_sda = CAM_SIOD,
        .pin_sccb_scl = CAM_SIOC,
        .pin_d7 = CAM_D7,
        .pin_d6 = CAM_D6,
        .pin_d5 = CAM_D5,
        .pin_d4 = CAM_D4,
        .pin_d3 = CAM_D3,
        .pin_d2 = CAM_D2,
        .pin_d1 = CAM_D1,
        .pin_d0 = CAM_D0,
        .pin_vsync = CAM_VSYNC,
        .pin_href = CAM_HREF,
        .pin_pclk = CAM_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12, /* 更高画质 */
        .fb_count = 5,
        .fb_location = CAMERA_FB_IN_PSRAM, /* 帧缓冲放在 PSRAM */
        .grab_mode = CAMERA_GRAB_LATEST,   /* 始终拿最新帧 */
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK)
        printf("摄像头初始化失败: %d\n", err);
    else
        printf("摄像头初始化成功\n");
    return err;
}

/* ========== WiFi 事件处理 ========== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
        printf("WiFi: 正在连接 %s...\n", WIFI_SSID);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            printf("WiFi: 重连 %d/%d\n", s_retry_num, WIFI_MAX_RETRY);
        }
        else
        {
            printf("WiFi: 连接失败，请检查密码\n");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        printf("WiFi: 连接成功！IP 地址: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
    }
}

/* ========== WiFi 初始化 ========== */
void wifi_init(void)
{
    /* NVS 初始化 (WiFi 需要) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ========== 网页源码（视频观看页） ========== */
static const char *STREAM_HTML =
    "<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32-S3-CAM 实时画面</title>"
    "<style>"
    "body{background:#111;color:#fff;text-align:center;font-family:sans-serif;margin:0;padding:20px}"
    "h1{font-size:24px;margin:10px 0}"
    "img{max-width:100%;border:2px solid #333;border-radius:8px}"
    ".info{color:#888;font-size:14px;margin-top:10px}"
    "</style></head>"
    "<body>"
    "<h1>📷 ESP32-S3-CAM 实时画面</h1>"
    "<img src='/stream'>"
    "<div class='info'>MJPEG 实时流</div>"
    "</body></html>";

/* 根路径 / → 返回 HTML 页面 */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, STREAM_HTML, strlen(STREAM_HTML));
    return ESP_OK;
}

/* /stream → MJPEG 实时流 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT);
    if (res != ESP_OK)
        return res;

    printf("MJPEG 客户端已连接\n");

    while (true)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            printf("stream: 取帧失败\n");
            httpd_resp_sendstr_chunk(req, "--" STREAM_BOUNDARY "--\r\n");
            break;
        }

        char hdr[160];
        int hdr_len = snprintf(hdr, sizeof(hdr),
                               "--" STREAM_BOUNDARY "\r\n"
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: %u\r\n\r\n",
                               fb->len);

        if (httpd_resp_send_chunk(req, hdr, hdr_len) != ESP_OK || httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK)
        {
            esp_camera_fb_return(fb);
            break;
        }
        esp_camera_fb_return(fb);
    }

    httpd_resp_sendstr_chunk(req, NULL);
    printf("MJPEG 客户端断开\n");
    return ESP_OK;
}

/* ========== 启动 Web 服务器 ========== */
void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;

    if (httpd_start(&server, &config) != ESP_OK)
    {
        printf("HTTP 服务器启动失败\n");
        return;
    }

    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    httpd_uri_t str = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &str);

    printf("HTTP 服务器已启动，请在浏览器访问 http://<IP 地址>/\n");
}

void app_main(void)
{
    /* 初始化摄像头 */
    camera_init();

    /* 联网并启动视频服务器 */
    wifi_init();
    start_webserver();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}