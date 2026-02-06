#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "web_server_task.h"
#include "log_task.h"
#include "wifi_task.h"
#include "child_led.h"
#include "kio800.h"

static httpd_handle_t s_server = NULL;

// 各子機のセンサデータを保持（1-4）
typedef struct {
    temp_sens_data_t data;
    bool is_valid;
    uint32_t last_update_ms;
} child_sensor_data_t;

static child_sensor_data_t s_child_sensor_data[4] = {0};  // 子機1-4
static SemaphoreHandle_t s_sensor_data_mutex = NULL;

// ==== タイムアウト設定 ====
#define CHILD_DATA_TIMEOUT_MS  10000  // 10秒でタイムアウト

// HTMLページ（4子機対応版）
static const char html_page[] = 
"<!DOCTYPE html>"
"<html lang='ja'>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>ESP32 制御パネル</title>"
"<style>"
"body{font-family:sans-serif;margin:0;padding:20px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh}"
".header{text-align:center;color:white;margin-bottom:20px}"
".led-section{text-align:center;margin-bottom:30px}"
".led-button{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;border:none;padding:15px 30px;font-size:20px;font-weight:bold;border-radius:50px;cursor:pointer;transition:all 0.3s ease;box-shadow:0 4px 15px rgba(102,126,234,0.4);margin-right:10px}"
".led-button:hover{transform:translateY(-2px);box-shadow:0 6px 20px rgba(102,126,234,0.6)}"
".log-button{background:linear-gradient(135deg,#28a745 0%,#20c997 100%);color:white;border:none;padding:15px 30px;font-size:20px;font-weight:bold;border-radius:50px;cursor:pointer;transition:all 0.3s ease;box-shadow:0 4px 15px rgba(40,167,69,0.4);margin-right:10px}"
".log-button:hover{transform:translateY(-2px);box-shadow:0 6px 20px rgba(40,167,69,0.6)}"
".log-button.logging{background:linear-gradient(135deg,#dc3545 0%,#c82333 100%);box-shadow:0 4px 15px rgba(220,53,69,0.4)}"
".log-button.logging:hover{box-shadow:0 6px 20px rgba(220,53,69,0.6)}"
".status{display:inline-block;margin-left:15px;padding:10px 20px;background:rgba(255,255,255,0.2);border-radius:10px;font-size:16px;color:white}"
".status.on{background:rgba(40,167,69,0.8)}"
".status.off{background:rgba(220,53,69,0.8)}"
".children-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:20px;max-width:1200px;margin:0 auto}"
".child-card{background:white;padding:20px;border-radius:15px;box-shadow:0 5px 20px rgba(0,0,0,0.2)}"
".child-header{font-size:20px;font-weight:bold;color:#333;margin-bottom:15px;padding-bottom:10px;border-bottom:2px solid #667eea}"
".child-header.inactive{color:#999;border-bottom-color:#ccc}"
".sensor-item{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #dee2e6}"
".sensor-item:last-child{border-bottom:none}"
".sensor-label{font-weight:bold;color:#495057}"
".sensor-value{color:#212529}"
".sensor-value.na{color:#999;font-style:italic}"
"</style>"
"</head>"
"<body>"
"<div class='header'><h1>LUPIOS 環境モニタ</h1></div>"
"<div class='led-section'>"
"<button id='log-button' class='log-button' onclick='toggleLogging()'>ロギングON</button>"
"</div>"
"<div class='children-grid'>"
"<div class='child-card'><div class='child-header' id='child1-header'>子機1</div><div class='sensor-item'><span class='sensor-label'>温度:</span><span id='child1-temp' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>湿度:</span><span id='child1-rh' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>気圧:</span><span id='child1-pressure' class='sensor-value na'>--</span></div></div>"
"<div class='child-card'><div class='child-header' id='child2-header'>子機2</div><div class='sensor-item'><span class='sensor-label'>温度:</span><span id='child2-temp' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>湿度:</span><span id='child2-rh' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>気圧:</span><span id='child2-pressure' class='sensor-value na'>--</span></div></div>"
"<div class='child-card'><div class='child-header' id='child3-header'>子機3</div><div class='sensor-item'><span class='sensor-label'>温度:</span><span id='child3-temp' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>湿度:</span><span id='child3-rh' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>気圧:</span><span id='child3-pressure' class='sensor-value na'>--</span></div></div>"
"<div class='child-card'><div class='child-header' id='child4-header'>子機4</div><div class='sensor-item'><span class='sensor-label'>温度:</span><span id='child4-temp' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>湿度:</span><span id='child4-rh' class='sensor-value na'>--</span></div><div class='sensor-item'><span class='sensor-label'>気圧:</span><span id='child4-pressure' class='sensor-value na'>--</span></div></div>"
"</div>"
"<script>"
"let loggingState=false;"
"let logData=[];"
"function toggleLogging(){"
"if(!loggingState){"
"loggingState=true;"
"logData=[];"
"const btn=document.getElementById('log-button');"
"btn.textContent='ロギングOFF';"
"btn.classList.add('logging');"
"}else{"
"loggingState=false;"
"const btn=document.getElementById('log-button');"
"btn.textContent='ロギングON';"
"btn.classList.remove('logging');"
"if(logData.length>0){"
"downloadLog();"
"}"
"}"
"}"
"function formatTimestamp(date){"
"const month=(date.getMonth()+1).toString().padStart(2,'0');"
"const day=date.getDate().toString().padStart(2,'0');"
"const hours=date.getHours().toString().padStart(2,'0');"
"const minutes=date.getMinutes().toString().padStart(2,'0');"
"const seconds=date.getSeconds().toString().padStart(2,'0');"
"return month+day+hours+minutes+seconds;"
"}"
"function downloadLog(){"
"if(logData.length===0){"
"alert('ログデータがありません');"
"return;"
"}"
"let csv='';"
"for(let i=0;i<logData.length;i++){"
"const entry=logData[i];"
"csv+=entry.timestamp+','+entry.childNo+','+entry.temp+','+entry.rh+','+entry.pressure+'\\n';"
"}"
"const blob=new Blob([csv],{type:'text/plain;charset=utf-8'});"
"const url=URL.createObjectURL(blob);"
"const a=document.createElement('a');"
"a.href=url;"
"const now=new Date();"
"const filename='sensor_log_'+formatTimestamp(now)+'.txt';"
"a.download=filename;"
"document.body.appendChild(a);"
"a.click();"
"document.body.removeChild(a);"
"URL.revokeObjectURL(url);"
"logData=[];"
"}"
"function updateSensorData(){"
"fetch('/sensor/data')"
".then(r=>{"
"if(!r.ok)throw new Error('HTTP '+r.status);"
"return r.text();"
"})"
".then(t=>{"
"try{"
"const d=JSON.parse(t);"
"const now=new Date();"
"const timestamp=formatTimestamp(now);"
"if(d&&typeof d==='object'&&Array.isArray(d.children)){"
"for(let i=0;i<4;i++){"
"const child=d.children[i];"
"const idx=i+1;"
"const header=document.getElementById('child'+idx+'-header');"
"if(child&&child.valid){"
"header.classList.remove('inactive');"
"const temp=(child.aht_t01/10).toFixed(1);"
"const rh=(child.aht_rh01/10).toFixed(1);"
"const pressure=(child.bmp_p01/10).toFixed(1);"
"document.getElementById('child'+idx+'-temp').textContent=temp+' ℃';"
"document.getElementById('child'+idx+'-temp').classList.remove('na');"
"document.getElementById('child'+idx+'-rh').textContent=rh+' %';"
"document.getElementById('child'+idx+'-rh').classList.remove('na');"
"document.getElementById('child'+idx+'-pressure').textContent=pressure+' hPa';"
"document.getElementById('child'+idx+'-pressure').classList.remove('na');"
"if(loggingState){"
"logData.push({timestamp:timestamp,childNo:idx,temp:temp,rh:rh,pressure:pressure});"
"}"
"}else{"
"header.classList.add('inactive');"
"document.getElementById('child'+idx+'-temp').textContent='--';"
"document.getElementById('child'+idx+'-temp').classList.add('na');"
"document.getElementById('child'+idx+'-rh').textContent='--';"
"document.getElementById('child'+idx+'-rh').classList.add('na');"
"document.getElementById('child'+idx+'-pressure').textContent='--';"
"document.getElementById('child'+idx+'-pressure').classList.add('na');"
"}"
"}"
"}"
"}catch(e){console.error('JSON parse error:',e);}"
"})"
".catch(e=>console.error('Fetch error:',e));"
"}"
"updateSensorData();"
"setInterval(updateSensorData,2000);"
"</script>"
"</body>"
"</html>";

// ルートハンドラ: メインページ
static esp_err_t root_handler(httpd_req_t *req)
{
    syslog(INFO, "root_handler: request received");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    size_t html_len = sizeof(html_page) - 1;  // null終端を除く
    syslog(INFO, "root_handler: sending HTML page, length=%zu", html_len);
    esp_err_t ret = httpd_resp_send(req, html_page, html_len);
    if (ret != ESP_OK) {
        syslog(ERR, "root_handler: failed to send HTML page, ret=%d", ret);
    }
    return ESP_OK;
}

// ルートハンドラ: favicon（404エラーを防ぐ）
static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}


// ルートハンドラ: センサデータ取得（全子機のデータを返す）
static esp_err_t sensor_data_handler(httpd_req_t *req)
{
    syslog(INFO, "sensor_data_handler: request received");
    char response[1024];  // 4子機分のデータ用に拡張
    
    if (s_sensor_data_mutex == NULL) {
        // ミューテックスが初期化されていない場合、空の配列を返す
        snprintf(response, sizeof(response), "{\"children\":[null,null,null,null]}");
        syslog(WARN, "sensor_data_handler: mutex not initialized");
    } else {
        xSemaphoreTake(s_sensor_data_mutex, portMAX_DELAY);
        uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // JSON配列を構築
        char *p = response;
        int len = snprintf(p, sizeof(response), "{\"children\":[");
        p += len;
        
        for (int i = 0; i < 4; i++) {
            if (i > 0) {
                len = snprintf(p, sizeof(response) - (p - response), ",");
                p += len;
            }
            
            // タイムアウトチェック（10秒以上更新がない場合は無効化）
            bool is_valid = s_child_sensor_data[i].is_valid;
            if (is_valid) {
                uint32_t elapsed_ms = current_ms - s_child_sensor_data[i].last_update_ms;
                if (elapsed_ms >= CHILD_DATA_TIMEOUT_MS) {
                    // タイムアウト：データを0にクリア
                    memset(&s_child_sensor_data[i].data, 0, sizeof(temp_sens_data_t));
                    s_child_sensor_data[i].is_valid = false;
                    is_valid = false;
                }
            }
            
            if (is_valid) {
                len = snprintf(p, sizeof(response) - (p - response),
                    "{\"valid\":true,\"aht_t01\":%d,\"aht_rh01\":%u,\"bmp_t01\":%d,\"bmp_p01\":%lu,\"aht_ok\":%s,\"bmp_ok\":%s,\"seq\":%lu}",
                    (int)s_child_sensor_data[i].data.aht_t01,
                    (unsigned int)s_child_sensor_data[i].data.aht_rh01,
                    (int)s_child_sensor_data[i].data.bmp_t01,
                    (unsigned long)s_child_sensor_data[i].data.bmp_p01,
                    s_child_sensor_data[i].data.aht_ok ? "true" : "false",
                    s_child_sensor_data[i].data.bmp_ok ? "true" : "false",
                    (unsigned long)s_child_sensor_data[i].data.seq);
            } else {
                len = snprintf(p, sizeof(response) - (p - response), "null");
            }
            p += len;
        }
        
        len = snprintf(p, sizeof(response) - (p - response), "]}");
        p += len;
        
        xSemaphoreGive(s_sensor_data_mutex);
        syslog(DEBUG, "sensor_data_handler: returning data for 4 children");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// センサデータ更新関数（UDP受信タスクから呼び出される、子機番号付き）
void web_server_update_sensor_data_with_child_no(uint8_t child_no, const temp_sens_data_t *data)
{
    if (data == NULL) {
        syslog(WARN, "web_server_update_sensor_data: data is NULL");
        return;
    }
    
    if (child_no < 1 || child_no > 4) {
        syslog(WARN, "web_server_update_sensor_data: invalid child_no=%d", child_no);
        return;
    }
    
    if (s_sensor_data_mutex == NULL) {
        syslog(WARN, "web_server_update_sensor_data: mutex not initialized");
        return;
    }
    
    xSemaphoreTake(s_sensor_data_mutex, portMAX_DELAY);
    int idx = child_no - 1;  // 0-3に変換
    memcpy(&s_child_sensor_data[idx].data, data, sizeof(temp_sens_data_t));
    s_child_sensor_data[idx].is_valid = true;
    s_child_sensor_data[idx].last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(s_sensor_data_mutex);
    
    // 子機LEDをトグル
    toggle_child_led(child_no);
    
    // KIO800にデータ送信（データが有効な場合のみ）
    if (data->aht_ok && data->bmp_ok) {
        kio800_send_sensor_data(child_no, data->aht_t01, data->aht_rh01, data->bmp_p01);
    }
    
    syslog(INFO, "Web server sensor data updated: Child %d AHT T=%.1fC RH=%.1f%% BMP T=%.1fC P=%.1fhPa seq=%lu",
           child_no,
           (float)data->aht_t01 / 10.0f,
           (float)data->aht_rh01 / 10.0f,
           (float)data->bmp_t01 / 10.0f,
           (float)data->bmp_p01 / 10.0f,
           (unsigned long)data->seq);
}

// 後方互換性のため（既存コード用）
void web_server_update_sensor_data(const temp_sens_data_t *data)
{
    // 子機番号が不明な場合は、最初の有効な子機または子機1に保存
    web_server_update_sensor_data_with_child_no(1, data);
}

// 子機センサーデータ取得（SSD1306表示用）
bool web_server_get_child_sensor_data(uint8_t child_no, temp_sens_data_t *data)
{
    if (data == NULL) {
        return false;
    }
    
    if (child_no < 1 || child_no > 4) {
        memset(data, 0, sizeof(temp_sens_data_t));
        return false;
    }
    
    if (s_sensor_data_mutex == NULL) {
        memset(data, 0, sizeof(temp_sens_data_t));
        return false;
    }
    
    xSemaphoreTake(s_sensor_data_mutex, portMAX_DELAY);
    int idx = child_no - 1;  // 0-3に変換
    uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // タイムアウトチェック（10秒以上更新がない場合は0を返す）
    if (s_child_sensor_data[idx].is_valid) {
        uint32_t elapsed_ms = current_ms - s_child_sensor_data[idx].last_update_ms;
        if (elapsed_ms >= CHILD_DATA_TIMEOUT_MS) {
            // タイムアウト：データを0にクリア
            memset(data, 0, sizeof(temp_sens_data_t));
            s_child_sensor_data[idx].is_valid = false;
            xSemaphoreGive(s_sensor_data_mutex);
            return false;
        } else {
            // 有効なデータ
            memcpy(data, &s_child_sensor_data[idx].data, sizeof(temp_sens_data_t));
            xSemaphoreGive(s_sensor_data_mutex);
            return true;
        }
    } else {
        // 無効なデータ
        memset(data, 0, sizeof(temp_sens_data_t));
        xSemaphoreGive(s_sensor_data_mutex);
        return false;
    }
}


// HTTPサーバー開始
static void start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 10;
    
    syslog(INFO, "Starting web server on port %d", config.server_port);
    
    if (httpd_start(&s_server, &config) == ESP_OK) {
        // ルートハンドラ登録
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(s_server, &root_uri);
        
        // faviconハンドラ
        httpd_uri_t favicon_uri = {
            .uri       = "/favicon.ico",
            .method    = HTTP_GET,
            .handler   = favicon_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(s_server, &favicon_uri);
        
        // センサデータ取得ハンドラ
        httpd_uri_t sensor_data_uri = {
            .uri       = "/sensor/data",
            .method    = HTTP_GET,
            .handler   = sensor_data_handler,
            .user_ctx  = NULL
        };
        esp_err_t ret = httpd_register_uri_handler(s_server, &sensor_data_uri);
        if (ret == ESP_OK) {
            syslog(INFO, "Sensor data handler registered: /sensor/data");
        } else {
            syslog(ERR, "Failed to register sensor data handler: %d", ret);
        }
        
        syslog(INFO, "Web server started successfully");
    } else {
        syslog(ERR, "Failed to start web server");
    }
}

// Webサーバータスク
static void web_server_task(void *pvParameters)
{
    // ミューテックス作成
    s_sensor_data_mutex = xSemaphoreCreateMutex();
    if (s_sensor_data_mutex == NULL) {
        syslog(ERR, "Failed to create sensor data mutex");
        vTaskDelete(NULL);
        return;
    }
    syslog(INFO, "Web server task: sensor data mutex created");
    
    // 子機LED初期化
    init_child_leds();
    
    // KIO800初期化
    kio800_init();
    
    // 初期データを0で初期化（表示用）
    memset(s_child_sensor_data, 0, sizeof(s_child_sensor_data));
    syslog(INFO, "Web server task: sensor data initialized for 4 children");
    
    // WiFi接続待機
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    start_web_server();
    
    syslog(INFO, "Web server task: running, waiting for sensor data...");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// 公開関数: Webサーバータスク開始
void start_web_server_task(void)
{
    xTaskCreate(web_server_task, "WebServerTask", 4096, NULL, 5, NULL);
    syslog(INFO, "Web server task created");
}
