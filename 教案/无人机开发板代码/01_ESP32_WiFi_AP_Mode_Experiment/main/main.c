#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi_types.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

static const char *TAG = "ESP32_AP";

/* AP 配置 */
#define EXAMPLE_ESP_WIFI_SSID "ESP32_AP"
#define EXAMPLE_ESP_WIFI_PASS "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL 1
#define EXAMPLE_MAX_STA_CONN 4

// UDP服务器端口
#define UDP_SERVER_PORT 8081
// TCP服务器端口
#define TCP_SERVER_PORT 8080

/* WiFi 事件处理函数 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event =
            (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station 连接: MAC: %02X:%02X:%02X:%02X:%02X:%02X, AID=%d",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5],
                 event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event =
            (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Station 断开: MAC: %02X:%02X:%02X:%02X:%02X:%02X, AID=%d",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5],
                 event->aid);
    }
}

/**
 * @brief UDP服务器任务
 */
static void udp_server_task(void *pvParameters)
{
    int server_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char rx_buffer[128];

    // 1. 创建socket（UDP）
    server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_fd < 0)
    {
        ESP_LOGE(TAG, "socket 创建失败");
        vTaskDelete(NULL);
        return;
    }

    // 2. 绑定端口
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(UDP_SERVER_PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        ESP_LOGE(TAG, "bind 端口 %d 失败", UDP_SERVER_PORT);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP服务器已启动，端口：%d", UDP_SERVER_PORT);

    while (1)
    {
        // 3. 阻塞接收数据，并记录发送方地址
        int len = recvfrom(server_fd, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&client_addr, &client_len);
        if (len > 0)
        {
            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "收到客户端消息(%s:%d): %s",
                     inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), rx_buffer);
            // 回发数据给发送方
            sendto(server_fd, "OK\r\n", 4, 0,
                   (struct sockaddr *)&client_addr, client_len);
        }
        else if (len < 0)
        {
            ESP_LOGE(TAG, "接收数据失败");
        }
    }
    close(server_fd);
    vTaskDelete(NULL);
}

/**
 * @brief TCP服务器任务
 */
static void tcp_server_task(void *pvParameters)
{
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char rx_buffer[128];

    // 1. 创建socket（TCP）
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        ESP_LOGE(TAG, "socket 创建失败");
        vTaskDelete(NULL);
        return;
    }

    // 2. 绑定端口
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_SERVER_PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        ESP_LOGE(TAG, "bind 端口 %d 失败", TCP_SERVER_PORT);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    // 3. 开启监听，等待客户端连接
    listen(server_fd, 1);
    ESP_LOGI(TAG, "TCP服务器已启动，端口：%d", TCP_SERVER_PORT);

    while (1)
    {
        // 4. 阻塞等待客户端接入
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd >= 0)
        {
            ESP_LOGI(TAG, "客户端已接入: %s:%d",
                     inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
            while (1)
            {
                // 接收电脑发来的数据
                int len = recv(client_fd, rx_buffer, sizeof(rx_buffer) - 1, 0);
                if (len > 0)
                {
                    rx_buffer[len] = '\0';
                    ESP_LOGI(TAG, "收到客户端消息: %s", rx_buffer);
                    // 回发数据给电脑
                    send(client_fd, "OK\r\n", 4, 0);
                }
                else if (len == 0 || len < 0)
                {
                    ESP_LOGI(TAG, "客户端断开连接");
                    close(client_fd);
                    break;
                }
            }
        }
    }
    close(server_fd);
    vTaskDelete(NULL);
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP 启动完成");
    ESP_LOGI(TAG, "SSID: %s", EXAMPLE_ESP_WIFI_SSID);
    ESP_LOGI(TAG, "Password: %s", EXAMPLE_ESP_WIFI_PASS);

    // 创建UDP服务器任务
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    // 创建TCP服务器任务
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 AP WiFi 实验启动");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
}
