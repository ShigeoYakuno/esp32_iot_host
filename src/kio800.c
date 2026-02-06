// src/kio800.c
// KIO800外部基板へのUART2通信

#include "kio800.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "log_task.h"
#include <string.h>
#include <stdint.h>

// ==== UART2設定 ====
#define KIO800_UART_NUM     UART_NUM_2
#define KIO800_UART_TX      GPIO_NUM_17
#define KIO800_UART_RX      GPIO_NUM_16
#define KIO800_UART_BAUD    115200
#define KIO800_BUF_SIZE     256  // 参考ファイルに合わせて256に変更

// ==== プロトコル定義 ====
#define KIO800_STX          0x02
#define KIO800_ETX          0x03

static const char *TAG = "kio800";
static bool s_uart_initialized = false;

// ==== UART2初期化 ====
void kio800_init(void)
{
    if (s_uart_initialized) {
        return;
    }

    uart_config_t uart_config = {
        .baud_rate = KIO800_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // UART2ドライバをインストール（参考ファイルと同じ設定）
    // 既にインストールされている場合はエラーになるので、ESP_ERR_INVALID_STATEを無視
    esp_err_t err = uart_driver_install(KIO800_UART_NUM, KIO800_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        syslog(ERR, "KIO800: Failed to install UART driver: %s", esp_err_to_name(err));
        return;
    }
    // ESP_ERR_INVALID_STATEの場合は既にインストール済みなので続行
    
    // UART2パラメータを設定
    ESP_ERROR_CHECK(uart_param_config(KIO800_UART_NUM, &uart_config));
    
    // UART2ピンを設定
    ESP_ERROR_CHECK(uart_set_pin(KIO800_UART_NUM, KIO800_UART_TX, KIO800_UART_RX, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_uart_initialized = true;
    syslog(INFO, "KIO800 UART2 initialized (TX=GPIO%d, RX=GPIO%d, %d baud)", 
           KIO800_UART_TX, KIO800_UART_RX, KIO800_UART_BAUD);
}

// ==== センサーデータ送信 ====
// フォーマット: STX(0x02) + 子機番号(1byte) + 温度(2byte) + 湿度(2byte) + 気圧(2byte) + ETX(0x03) = 9byte
// 温度: 23.1℃ → 0.1℃単位で231 → 0x00E7 (2byte, リトルエンディアン)
// 湿度: 43.1% → 0.1%単位で431 → 0x01AF (2byte, リトルエンディアン)
// 気圧: 1013.5hPa → 小数点切り捨てで1013 → 0x03F5 (2byte, リトルエンディアン)
void kio800_send_sensor_data(uint8_t child_no, int16_t temp_01c, uint16_t rh_01, uint32_t pressure_01hpa)
{
    if (!s_uart_initialized) {
        return;
    }

    // データ変換
    // 温度: 0.1℃単位 → 整数値（例: 23.1℃ = 231）
    uint16_t temp_value = (uint16_t)(temp_01c >= 0 ? temp_01c : 0);
    
    // 湿度: 0.1%単位 → 整数値（例: 43.1% = 431）
    uint16_t rh_value = rh_01;
    
    // 気圧: 0.1hPa単位 → 小数点切り捨てで整数値（例: 1013.5hPa = 1013）
    uint16_t pressure_value = (uint16_t)(pressure_01hpa / 10);

    // 9バイトのパケットを構築
    uint8_t packet[9];
    packet[0] = KIO800_STX;                    // STX
    packet[1] = child_no;                       // 子機番号
    packet[2] = (uint8_t)(temp_value & 0xFF);  // 温度下位バイト
    packet[3] = (uint8_t)(temp_value >> 8);    // 温度上位バイト
    packet[4] = (uint8_t)(rh_value & 0xFF);    // 湿度下位バイト
    packet[5] = (uint8_t)(rh_value >> 8);      // 湿度上位バイト
    packet[6] = (uint8_t)(pressure_value & 0xFF);  // 気圧下位バイト
    packet[7] = (uint8_t)(pressure_value >> 8);    // 気圧上位バイト
    packet[8] = KIO800_ETX;                    // ETX

    // UART2に送信（タイムアウト付き）
    int len = uart_write_bytes(KIO800_UART_NUM, packet, sizeof(packet));
    if (len == sizeof(packet)) {
        syslog(DEBUG, "KIO800: Sent data for child %d (T=%d, RH=%u, P=%u)", 
               child_no, temp_value, rh_value, pressure_value);
    } else if (len < 0) {
        // エラーコードを取得
        esp_err_t err = (esp_err_t)len;
        syslog(WARN, "KIO800: UART write error: %s (0x%x)", esp_err_to_name(err), err);
        // エラー時は再初期化を試みる
        s_uart_initialized = false;
        kio800_init();
    } else {
        syslog(WARN, "KIO800: Failed to send data (sent %d/%d bytes)", len, sizeof(packet));
    }
}
