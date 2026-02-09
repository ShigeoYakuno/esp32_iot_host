#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"  // IP2STRマクロ用
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include <arpa/inet.h>  // inet_ntop用
#include "wifi_task.h"
#include "web_server_task.h"  // temp_sens_data_t定義用、Webサーバーへのデータ送信用
#include "log_task.h"
#include "flash_data.h"  // SSID番号取得用
#include "cJSON.h" 

// ==== マクロ定義 ====
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// データ送信先設定（APモード用）
#define DATA_SERVER_HOST   "192.168.4.2"    // 送信先クライアントIPアドレス（APモードで接続したデバイス）
#define DATA_SERVER_PORT   8080             // 送信先ポート番号
#define DATA_QUEUE_SIZE    10               // データキューのサイズ

// APモード設定（仕様書準拠）
#define AP_SSID_PREFIX     "PE_IOT_GATEWAY_"  // SSIDプレフィックス
#define AP_PASSWORD        "12345678"       // APのパスワード（8文字以上）
#define AP_CHANNEL         1                 // APのチャンネル
#define AP_MAX_CONNECTIONS 8                 // 最大接続数（子機4台 + PC等）
#define AP_IP              "192.168.4.1"    // APのIPアドレス
#define AP_GATEWAY         "192.168.4.1"    // ゲートウェイ
#define AP_NETMASK         "255.255.255.0"  // サブネットマスク

// UDP受信設定
#define UDP_RECV_PORT      50000            // UDP受信ポート
#define MAX_PAYLOAD_SIZE   256              // 最大ペイロードサイズ
#define MAX_CHILD_NODES    4                // 最大子機数
#define STALE_TIMEOUT_MS   5000             // STALE判定タイムアウト（5秒）

// ==== 内部シンボル ====
static EventGroupHandle_t s_wifi_event_group;
static TaskHandle_t mainTaskHandle_ = NULL;
static QueueHandle_t s_data_queue = NULL;
static TaskHandle_t s_data_send_task = NULL;
static TaskHandle_t s_udp_recv_task = NULL;
static TaskHandle_t s_child_monitor_task = NULL;

// ==== 子機テーブル定義 ====
typedef enum {
    CHILD_STATE_ACTIVE = 0,
    CHILD_STATE_STALE
} child_state_t;

typedef struct {
    uint8_t child_no;                    // 子機No（1-4）
    char source_ip[16];                  // 送信元IPアドレス（文字列）
    uint32_t last_recv_time_ms;          // 最終受信時刻（ms）
    char latest_payload[MAX_PAYLOAD_SIZE]; // 最新受信ペイロード
    child_state_t state;                  // 状態（ACTIVE/STALE）
    bool is_valid;                        // エントリが有効かどうか
} child_node_t;

static child_node_t s_child_table[MAX_CHILD_NODES];
static SemaphoreHandle_t s_child_table_mutex = NULL;

// ==== MACアドレス表示用マクロ ====
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

// ==== 子機テーブル管理関数 ====
static void child_table_init(void)
{
    s_child_table_mutex = xSemaphoreCreateMutex();
    configASSERT(s_child_table_mutex);
    
    for (int i = 0; i < MAX_CHILD_NODES; i++) {
        s_child_table[i].is_valid = false;
        s_child_table[i].child_no = 0;
        s_child_table[i].source_ip[0] = '\0';
        s_child_table[i].last_recv_time_ms = 0;
        s_child_table[i].latest_payload[0] = '\0';
        s_child_table[i].state = CHILD_STATE_STALE;
    }
}

static child_node_t* child_table_find_by_no(uint8_t child_no)
{
    for (int i = 0; i < MAX_CHILD_NODES; i++) {
        if (s_child_table[i].is_valid && s_child_table[i].child_no == child_no) {
            return &s_child_table[i];
        }
    }
    return NULL;
}

static child_node_t* child_table_find_free_slot(void)
{
    for (int i = 0; i < MAX_CHILD_NODES; i++) {
        if (!s_child_table[i].is_valid) {
            return &s_child_table[i];
        }
    }
    return NULL;
}

static void child_table_update(uint8_t child_no, const char* source_ip, const char* payload, uint32_t current_time_ms)
{
    xSemaphoreTake(s_child_table_mutex, portMAX_DELAY);
    
    child_node_t* node = child_table_find_by_no(child_no);
    if (node == NULL) {
        // 新規登録
        node = child_table_find_free_slot();
        if (node == NULL) {
            syslog(WARN, "Child table full, cannot register child_no=%d", child_no);
            xSemaphoreGive(s_child_table_mutex);
            return;
        }
        node->is_valid = true;
        node->child_no = child_no;
        strncpy(node->source_ip, source_ip, sizeof(node->source_ip) - 1);
        node->source_ip[sizeof(node->source_ip) - 1] = '\0';
    }
    
    // 更新
    node->last_recv_time_ms = current_time_ms;
    strncpy(node->latest_payload, payload, sizeof(node->latest_payload) - 1);
    node->latest_payload[sizeof(node->latest_payload) - 1] = '\0';
    node->state = CHILD_STATE_ACTIVE;
    
    xSemaphoreGive(s_child_table_mutex);
}

// ==== 子機No抽出関数 ====
// ペイロード形式: "N=<子機No>,<現行フォーマット>"
static int extract_child_no(const char* payload, uint8_t* child_no)
{
    if (payload == NULL || strlen(payload) < 3) {
        return -1;
    }
    
    // "N="で始まるか確認
    if (payload[0] != 'N' || payload[1] != '=') {
        return -1;
    }
    
    // 数値を抽出
    int no = 0;
    int i = 2;
    while (payload[i] >= '0' && payload[i] <= '9') {
        no = no * 10 + (payload[i] - '0');
        i++;
        if (no > MAX_CHILD_NODES) {
            return -1; // 範囲外
        }
    }
    
    // カンマが続くか確認
    if (payload[i] != ',') {
        return -1;
    }
    
    if (no >= 1 && no <= MAX_CHILD_NODES) {
        *child_no = (uint8_t)no;
        return 0;
    }
    
    return -1;
}

// ==== 子機ACTIVE状態確認関数 ====
bool wifi_has_active_child(void)
{
    if (s_child_table_mutex == NULL) {
        return false;  // まだ初期化されていない
    }
    
    xSemaphoreTake(s_child_table_mutex, portMAX_DELAY);
    
    bool has_active = false;
    for (int i = 0; i < MAX_CHILD_NODES; i++) {
        if (s_child_table[i].is_valid && s_child_table[i].state == CHILD_STATE_ACTIVE) {
            has_active = true;
            break;
        }
    }
    
    xSemaphoreGive(s_child_table_mutex);
    
    return has_active;
}

// ==== SSID生成関数 ====
static void generate_ssid(char *ssid_buf, size_t buf_size)
{
    uint32_t ssid_no = getSsidNo();
    snprintf(ssid_buf, buf_size, "%s%lu", AP_SSID_PREFIX, (unsigned long)ssid_no);
}

// ==== Wi-Fiイベントハンドラ（APモード用） ====
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        char ssid[32];
        generate_ssid(ssid, sizeof(ssid));
        syslog(INFO, "Wi-Fi AP started (SSID: %s)", ssid);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
        syslog(INFO, "Wi-Fi AP stopped");
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        syslog(INFO, "Station " MACSTR " joined, AID=%d (子機接続)",
               MAC2STR(event->mac), event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        syslog(INFO, "Station " MACSTR " left, AID=%d (子機切断)",
               MAC2STR(event->mac), event->aid);
    }
}

// ==== UDP受信タスク ====
static void udp_recv_task(void *pvParameters)
{
    syslog(INFO, "UDP receive task started on port %d", UDP_RECV_PORT);
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        syslog(ERR, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_RECV_PORT);
    
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        syslog(ERR, "Failed to bind UDP socket to port %d", UDP_RECV_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    syslog(INFO, "UDP socket bound to port %d", UDP_RECV_PORT);
    
    char recv_buf[MAX_PAYLOAD_SIZE + 1];
    struct sockaddr_in source_addr;
    socklen_t addr_len = sizeof(source_addr);
    
    while (1) {
        // WiFi接続確認
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        if (!(bits & WIFI_CONNECTED_BIT)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // 受信タイムアウト設定（1秒）
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        
        int len = recvfrom(sock, recv_buf, MAX_PAYLOAD_SIZE, 0,
                          (struct sockaddr *)&source_addr, &addr_len);
        
        if (len > 0) {
            recv_buf[len] = '\0'; // 文字列終端
            
            // 送信元IPアドレスを文字列に変換
            char source_ip_str[16];
            inet_ntop(AF_INET, &source_addr.sin_addr, source_ip_str, sizeof(source_ip_str));
            
            uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // まずJSON形式のデータを試す（子機からのデータはJSON形式）
            cJSON *json = cJSON_Parse(recv_buf);
            if (json != NULL) {
                cJSON *child_no_item = cJSON_GetObjectItem(json, "child_no");
                cJSON *aht_t01_item = cJSON_GetObjectItem(json, "aht_t01");
                cJSON *aht_rh01_item = cJSON_GetObjectItem(json, "aht_rh01");
                cJSON *bmp_t01_item = cJSON_GetObjectItem(json, "bmp_t01");
                cJSON *bmp_p01_item = cJSON_GetObjectItem(json, "bmp_p01");
                cJSON *aht_ok_item = cJSON_GetObjectItem(json, "aht_ok");
                cJSON *bmp_ok_item = cJSON_GetObjectItem(json, "bmp_ok");
                cJSON *seq_item = cJSON_GetObjectItem(json, "seq");
                cJSON *rssi_item = cJSON_GetObjectItem(json, "rssi");
                
                if (child_no_item && aht_t01_item && aht_rh01_item && 
                    bmp_t01_item && bmp_p01_item && aht_ok_item && 
                    bmp_ok_item && seq_item) {
                    // JSONからセンサーデータを抽出
                    uint8_t child_no = (uint8_t)cJSON_GetNumberValue(child_no_item);
                    temp_sens_data_t sensor_data = {0};
                    sensor_data.aht_t01 = (int16_t)cJSON_GetNumberValue(aht_t01_item);
                    sensor_data.aht_rh01 = (uint16_t)cJSON_GetNumberValue(aht_rh01_item);
                    sensor_data.bmp_t01 = (int16_t)cJSON_GetNumberValue(bmp_t01_item);
                    sensor_data.bmp_p01 = (uint32_t)cJSON_GetNumberValue(bmp_p01_item);
                    sensor_data.aht_ok = cJSON_IsTrue(aht_ok_item);
                    sensor_data.bmp_ok = cJSON_IsTrue(bmp_ok_item);
                    sensor_data.seq = (uint32_t)cJSON_GetNumberValue(seq_item);
                    // RSSIはオプショナル（存在しない場合は0）
                    if (rssi_item) {
                        sensor_data.rssi = (int)cJSON_GetNumberValue(rssi_item);
                    } else {
                        sensor_data.rssi = 0;
                    }
                    
                    // 子機テーブルを更新
                    child_table_update(child_no, source_ip_str, recv_buf, current_time_ms);
                    
                    // Webサーバーに最新データを送信（子機番号付き）
                    web_server_update_sensor_data_with_child_no(child_no, &sensor_data);
                    
                    syslog(INFO, "[RX] JSON N=%d IP=%s AHT T=%.1fC RH=%.1f%% BMP T=%.1fC P=%.1fhPa RSSI=%d dBm seq=%lu",
                           child_no, source_ip_str,
                           (float)sensor_data.aht_t01 / 10.0f,
                           (float)sensor_data.aht_rh01 / 10.0f,
                           (float)sensor_data.bmp_t01 / 10.0f,
                           (float)sensor_data.bmp_p01 / 10.0f,
                           sensor_data.rssi,
                           (unsigned long)sensor_data.seq);
                } else {
                    syslog(WARN, "[RX] JSON parse OK but missing fields IP=%s", source_ip_str);
                }
                cJSON_Delete(json);
            } else {
                // JSONパース失敗、旧形式 "N=1,..." を試す
                uint8_t child_no = 0;
                int extract_result = extract_child_no(recv_buf, &child_no);
                if (extract_result == 0) {
                    child_table_update(child_no, source_ip_str, recv_buf, current_time_ms);
                    syslog(INFO, "[RX] N=%d IP=%s payload=%s", child_no, source_ip_str, recv_buf);
                } else {
                    syslog(WARN, "[RX] UNKNOWN format IP=%s payload=%s", source_ip_str, recv_buf);
                }
            }
        } else if (len < 0) {
            // タイムアウトまたはエラー（正常動作）
            // エラーログは出力しない（タイムアウトは正常）
        }
    }
}

// ==== 子機監視タスク（STALE判定） ====
static void child_monitor_task(void *pvParameters)
{
    syslog(INFO, "Child monitor task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒ごとにチェック
        
        // WiFi接続確認
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        if (!(bits & WIFI_CONNECTED_BIT)) {
            continue;
        }
        
        uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        xSemaphoreTake(s_child_table_mutex, portMAX_DELAY);
        
        for (int i = 0; i < MAX_CHILD_NODES; i++) {
            if (!s_child_table[i].is_valid) {
                continue;
            }
            
            uint32_t elapsed_ms = current_time_ms - s_child_table[i].last_recv_time_ms;
            
            // STALE判定（5秒経過）
            if (elapsed_ms >= STALE_TIMEOUT_MS) {
                if (s_child_table[i].state == CHILD_STATE_ACTIVE) {
                    // ACTIVE → STALE に遷移
                    s_child_table[i].state = CHILD_STATE_STALE;
                    syslog(WARN, "[STALE] N=%d last_seen=%lums", 
                           s_child_table[i].child_no, elapsed_ms);
                }
            }
        }
        
        xSemaphoreGive(s_child_table_mutex);
    }
}

// ==== データ送信タスク ====
// スタック使用量を削減するため、大きな変数を静的変数に
static int s_sock = -1;
static bool s_connected = false;
static struct sockaddr_in s_server_addr;

static void data_send_task(void *pvParameters)
{
    syslog(DEBUG_WIFI, "Data send task started");
    
    while (1) {
        temp_sens_data_t data;
        
        // キューからデータを受信（タイムアウト: 1秒）
        if (xQueueReceive(s_data_queue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            
            // WiFi接続確認
            EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
            if (!(bits & WIFI_CONNECTED_BIT)) {
                syslog(DEBUG_WIFI, "WiFi not connected, skipping data send");
                if (s_sock >= 0) {
                    close(s_sock);
                    s_sock = -1;
                    s_connected = false;
                }
                continue;
            }
            
            // ソケット未接続の場合は接続を試みる
            if (!s_connected || s_sock < 0) {
                s_sock = socket(AF_INET, SOCK_STREAM, 0);
                if (s_sock < 0) {
                    syslog(DEBUG_WIFI, "Failed to create socket");
                    continue;
                }
                
                memset(&s_server_addr, 0, sizeof(s_server_addr));
                s_server_addr.sin_family = AF_INET;
                s_server_addr.sin_port = htons(DATA_SERVER_PORT);
                inet_pton(AF_INET, DATA_SERVER_HOST, &s_server_addr.sin_addr);
                
                // 接続タイムアウト設定
                struct timeval timeout;
                timeout.tv_sec = 3;
                timeout.tv_usec = 0;
                setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
                setsockopt(s_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
                
                if (connect(s_sock, (struct sockaddr *)&s_server_addr, sizeof(s_server_addr)) < 0) {
                    syslog(DEBUG_WIFI, "Failed to connect to %s:%d", DATA_SERVER_HOST, DATA_SERVER_PORT);
                    close(s_sock);
                    s_sock = -1;
                    continue;
                }
                
                s_connected = true;
                syslog(DEBUG_WIFI, "Connected to %s:%d", DATA_SERVER_HOST, DATA_SERVER_PORT);
            }
            
            // データ送信（JSON形式）
            // 固定形式: {"aht_t01":253,"aht_rh01":482,"bmp_t01":251,"bmp_p01":100845,"aht_ok":true,"bmp_ok":true,"seq":123}
            char send_buf[128];
            int len = snprintf(send_buf, sizeof(send_buf),
                "{\"aht_t01\":%d,\"aht_rh01\":%u,\"bmp_t01\":%d,\"bmp_p01\":%lu,\"aht_ok\":%s,\"bmp_ok\":%s,\"seq\":%lu}\n",
                (int)data.aht_t01,
                (unsigned int)data.aht_rh01,
                (int)data.bmp_t01,
                (unsigned long)data.bmp_p01,
                data.aht_ok ? "true" : "false",
                data.bmp_ok ? "true" : "false",
                (unsigned long)data.seq);
            
            int sent = send(s_sock, send_buf, len, 0);
            if (sent < 0) {
                syslog(DEBUG_WIFI, "Failed to send data, closing socket");
                close(s_sock);
                s_sock = -1;
                s_connected = false;
            } else {
                syslog(DEBUG_WIFI, "Sent temp sens data: seq=%lu", (unsigned long)data.seq);
            }
        }
        
        // 接続が長時間使われていない場合は閉じる（5秒間データが来ない場合）
        // これは実装の簡略化のため。必要に応じて調整可能。
    }
}

// ==== Wi-Fiタスク本体（APモード） ====
static void wifi_task(void *pvParameters)
{
    syslog(DEBUG_WIFI, "Wi-Fi AP init sequence start");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // AP用のネットインターフェースを作成
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        syslog(DEBUG_WIFI, "Failed to create AP netif");
        vTaskDelete(NULL);
        return;
    }

    // 固定IPアドレスを設定
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    esp_netif_dns_info_t dns_info;
    
    inet_pton(AF_INET, AP_IP, &ip_info.ip);
    inet_pton(AF_INET, AP_GATEWAY, &ip_info.gw);
    inet_pton(AF_INET, AP_NETMASK, &ip_info.netmask);
    inet_pton(AF_INET, AP_GATEWAY, &dns_info.ip.u_addr.ip4);
    
    // IPアドレス設定
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns_info));
    // 注意：ESP32のAPモードでは、クライアント接続のためにDHCPサーバーを起動する必要がある
    // ただし、子機側では静的IP（192.168.4.11-14）を使用する
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
    
    syslog(DEBUG_WIFI, "AP IP: " IPSTR ", Gateway: " IPSTR ", Netmask: " IPSTR,
           IP2STR(&ip_info.ip), IP2STR(&ip_info.gw), IP2STR(&ip_info.netmask));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    // データ送信用キューを作成（構造体サイズ）
    s_data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(temp_sens_data_t));
    if (s_data_queue == NULL) {
        syslog(DEBUG_WIFI, "Failed to create data queue");
        vTaskDelete(NULL);
        return;
    }

    // APモード設定
    char ssid[32];
    generate_ssid(ssid, sizeof(ssid));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ssid),
            .password = AP_PASSWORD,
            .channel = AP_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = AP_MAX_CONNECTIONS,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid[sizeof(wifi_config.ap.ssid) - 1] = '\0';

    // パスワードが空の場合はオープンモード
    if (strlen(AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    syslog(DEBUG_WIFI, "Starting AP with SSID: %s", wifi_config.ap.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // AP起動待機（5秒）
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(5000)
    );

    if (bits & WIFI_CONNECTED_BIT) {
        syslog(DEBUG_WIFI, "Wi-Fi AP started successfully");
        
        if (mainTaskHandle_ != NULL) {
            xTaskNotify(mainTaskHandle_, 0x1234, eSetValueWithOverwrite);
        }
        
        // 子機テーブル初期化
        child_table_init();
        
        // データ送信タスクを起動（既存機能維持）
        xTaskCreate(data_send_task, "DataSendTask", 2048, NULL, 2, &s_data_send_task);
        syslog(DEBUG_WIFI, "Data send task started");
        
        // UDP受信タスクを起動
        xTaskCreate(udp_recv_task, "UdpRecvTask", 4096, NULL, 5, &s_udp_recv_task);
        syslog(DEBUG_WIFI, "UDP receive task started");
        
        // 子機監視タスクを起動
        xTaskCreate(child_monitor_task, "ChildMonitorTask", 2048, NULL, 3, &s_child_monitor_task);
        syslog(DEBUG_WIFI, "Child monitor task started");
    } else {
        syslog(DEBUG_WIFI, "Wi-Fi AP start timeout");
    }

    syslog(DEBUG_WIFI, "WiFi AP task running on core %d", xPortGetCoreID());

    // メインループ（AP維持）
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10秒ごとに状態を確認
        
        bits = xEventGroupGetBits(s_wifi_event_group);
        if (bits & WIFI_CONNECTED_BIT) {
            // APは起動中
        } else {
            syslog(DEBUG_WIFI, "AP not running");
        }
    }
}

// ==== 公開関数 ====
void start_wifi_task(TaskHandle_t mainTaskHandle)
{
    mainTaskHandle_ = mainTaskHandle;
    // スタックサイズ削減: 8192 → 4096（IRAM節約）
    xTaskCreate(wifi_task, "WiFiTask", 4096, NULL, 3, NULL);
}

QueueHandle_t wifi_get_data_queue(void)
{
    return s_data_queue;
}

// 既存互換性用（非推奨、temp_sens_task使用時は使用しない）
bool wifi_send_data(uint32_t data)
{
    // 互換性のため、空実装（temp_sens_data_t構造体を使用すること）
    (void)data;
    return false;
}

// 新規構造体データ送信関数
bool wifi_send_temp_sens_data(const temp_sens_data_t *data)
{
    if (s_data_queue == NULL || data == NULL) {
        return false;
    }
    
    // キューにデータを送信（非ブロッキング）
    if (xQueueSend(s_data_queue, data, 0) == pdTRUE) {
        return true;
    } else {
        // キューが満杯の場合は古いデータを上書き
        temp_sens_data_t dummy;
        xQueueReceive(s_data_queue, &dummy, 0);
        return xQueueSend(s_data_queue, data, 0) == pdTRUE;
    }
}
