// src/ssd1306_task.c
// ESP32 SSD1306 I2C OLED ディスプレイタスク

#include "ssd1306_task.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_attr.h"  // IRAM_ATTR用
#include "log_task.h"
#include "flash_data.h"
#include "user_common.h"
#include <string.h>
#include <stdint.h>

// ==== ボタンイベント定義 ====
typedef enum {
    BUTTON_EVENT_SW1,
    BUTTON_EVENT_SW2,
} button_event_t;

// ==== ハードウェア設定 ====
#ifndef SSD1306_I2C_PORT
#define SSD1306_I2C_PORT      I2C_NUM_1
#endif
#ifndef SSD1306_I2C_SDA
#define SSD1306_I2C_SDA       32         // GPIO32
#endif
#ifndef SSD1306_I2C_SCL
#define SSD1306_I2C_SCL       33         // GPIO33
#endif
#ifndef SSD1306_I2C_FREQ
#define SSD1306_I2C_FREQ      400000     // 400kHz
#endif

// ==== SSD1306 I2Cアドレス ====
// 0x3Cまたは0x3Dのいずれか（7bitアドレス）
// 0x3Cを試して、失敗したら0x3Dを試す
#define SSD1306_ADDR_1        0x3C
#define SSD1306_ADDR_2        0x3D

// ==== SSD1306 コマンド定義 ====
#define SSD1306_CMD_MODE      0x00
#define SSD1306_DATA_MODE     0x40

// コマンド
#define SSD1306_CMD_DISPLAY_OFF           0xAE
#define SSD1306_CMD_DISPLAY_ON            0xAF
#define SSD1306_CMD_SET_DISPLAY_CLOCK     0xD5
#define SSD1306_CMD_SET_MULTIPLEX         0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET    0xD3
#define SSD1306_CMD_SET_START_LINE        0x40
#define SSD1306_CMD_CHARGE_PUMP           0x8D
#define SSD1306_CMD_MEMORY_MODE           0x20
#define SSD1306_CMD_SEG_REMAP             0xA1
#define SSD1306_CMD_COM_SCAN_DEC          0xC8
#define SSD1306_CMD_COM_SCAN_INC          0xC0
#define SSD1306_CMD_SET_COM_PINS          0xDA
#define SSD1306_CMD_SET_CONTRAST          0x81
#define SSD1306_CMD_SET_PRECHARGE         0xD9
#define SSD1306_CMD_SET_VCOM_DETECT       0xDB
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_CMD_NORMAL_DISPLAY        0xA6
#define SSD1306_CMD_INVERT_DISPLAY        0xA7
#define SSD1306_CMD_COLUMN_ADDR           0x21
#define SSD1306_CMD_PAGE_ADDR             0x22
#define SSD1306_CMD_ACTIVATE_SCROLL       0x2F
#define SSD1306_CMD_DEACTIVATE_SCROLL     0x2E

static const char *TAG = "ssd1306Task";

static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_i2c_mutex = NULL;
static uint8_t s_i2c_addr = 0;
static uint8_t s_display_buffer[SSD1306_WIDTH * SSD1306_PAGES];
static bool s_display_inverted = false;
static bool s_i2c_initialized = false;

// ==== 表示モード管理 ====
static ssd1306_display_mode_t s_display_mode = SSD1306_MODE_SENSOR;
static TickType_t s_mode_timer = 0;  // モードのタイマー（0=無効、10秒でモード0に戻る）
static QueueHandle_t s_button_queue = NULL;  // ボタンイベントキュー
static bool s_executing = false;  // 実行中フラグ
static TickType_t s_result_timer = 0;  // 実行結果表示タイマー（0=無効）
static char s_result_message[64] = "";  // 実行結果メッセージ

// ==== デバウンス管理 ====
static TickType_t s_sw1_last_press = 0;  // SW1最後に押された時刻（0=未押下）
static TickType_t s_sw2_last_press = 0;  // SW2最後に押された時刻（0=未押下）
#define DEBOUNCE_TIME_MS  500  // デバウンス時間（0.5秒）

// ==== 5x7フォントデータ（ASCII 32-127） ====
// 各文字は5ピクセル幅、7ピクセル高さ
static const uint8_t font_5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32: スペース
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33: !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34: "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35: #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36: $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37: %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38: &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39: '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40: (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41: )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42: *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43: +
    {0x00, 0x00, 0xA0, 0x60, 0x00}, // 44: ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45: -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46: .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47: /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48: 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49: 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50: 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51: 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52: 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53: 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54: 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55: 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56: 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57: 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58: :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59: ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60: <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61: =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62: >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63: ?
    {0x32, 0x49, 0x59, 0x51, 0x3E}, // 64: @
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, // 65: A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66: B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67: C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68: D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69: E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70: F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71: G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72: H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73: I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74: J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75: K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76: L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 77: M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78: N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79: O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80: P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81: Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82: R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83: S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84: T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85: U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86: V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 87: W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88: X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89: Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90: Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 91: [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92: backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 93: ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94: ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95: _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 96: `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97: a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 98: b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99: c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 100: d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101: e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 102: f
    {0x18, 0xA4, 0xA4, 0xA4, 0x7C}, // 103: g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 104: h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 105: i
    {0x40, 0x80, 0x84, 0x7D, 0x00}, // 106: j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 107: k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 108: l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 109: m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 110: n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111: o
    {0xFC, 0x24, 0x24, 0x24, 0x18}, // 112: p
    {0x18, 0x24, 0x24, 0x18, 0xFC}, // 113: q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 114: r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115: s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 116: t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 117: u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 118: v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 119: w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120: x
    {0x1C, 0xA0, 0xA0, 0xA0, 0x7C}, // 121: y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 122: z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123: {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 124: |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125: }
    {0x10, 0x08, 0x08, 0x10, 0x08}, // 126: ~
    {0x78, 0x46, 0x41, 0x46, 0x78}, // 127: DEL
};

// ==== 前方宣言 ====
static esp_err_t i2c_bus_init(void);
static esp_err_t ssd1306_send_command(uint8_t cmd);
static esp_err_t ssd1306_send_data(const uint8_t *data, size_t len);
static esp_err_t ssd1306_init_hw(void);
static esp_err_t ssd1306_detect_address(void);
static void ssd1306_task_entry(void *arg);

// ==== I2C初期化 ====
static esp_err_t i2c_bus_init(void)
{
    if (s_i2c_initialized) {
        return ESP_OK;
    }

    i2c_config_t c = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SSD1306_I2C_SDA,
        .scl_io_num = SSD1306_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,  // 外部10kΩプルアップあり
        .scl_pullup_en = GPIO_PULLUP_DISABLE,  // 外部10kΩプルアップあり
        .master.clk_speed = SSD1306_I2C_FREQ,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(SSD1306_I2C_PORT, &c);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(SSD1306_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        // 既にインストール済みの場合はOK（他のタスクが初期化済み）
        err = ESP_OK;
    }
    
    if (err == ESP_OK) {
        s_i2c_initialized = true;
    }
    
    return err;
}

// ==== I2C Mutex管理 ====
static bool take_i2c_mutex(TickType_t timeout)
{
    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
    }
    return (xSemaphoreTake(s_i2c_mutex, timeout) == pdTRUE);
}

static void give_i2c_mutex(void)
{
    if (s_i2c_mutex) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

// ==== SSD1306 I2C通信 ====
static esp_err_t ssd1306_send_command(uint8_t cmd)
{
    if (!take_i2c_mutex(pdMS_TO_TICKS(100))) {
        return ESP_ERR_TIMEOUT;
    }

    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (s_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, SSD1306_CMD_MODE, true);
    i2c_master_write_byte(i2c_cmd, cmd, true);
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(SSD1306_I2C_PORT, i2c_cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(i2c_cmd);

    give_i2c_mutex();
    return ret;
}

static esp_err_t ssd1306_send_data(const uint8_t *data, size_t len)
{
    if (!take_i2c_mutex(pdMS_TO_TICKS(100))) {
        return ESP_ERR_TIMEOUT;
    }

    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (s_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(i2c_cmd, SSD1306_DATA_MODE, true);
    i2c_master_write(i2c_cmd, data, len, true);
    i2c_master_stop(i2c_cmd);
    esp_err_t ret = i2c_master_cmd_begin(SSD1306_I2C_PORT, i2c_cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(i2c_cmd);

    give_i2c_mutex();
    return ret;
}

// ==== I2Cアドレス検出 ====
static esp_err_t ssd1306_detect_address(void)
{
    // 0x3Cを試す
    s_i2c_addr = SSD1306_ADDR_1;
    if (take_i2c_mutex(pdMS_TO_TICKS(100))) {
        i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
        i2c_master_start(i2c_cmd);
        i2c_master_write_byte(i2c_cmd, (s_i2c_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(i2c_cmd);
        esp_err_t ret = i2c_master_cmd_begin(SSD1306_I2C_PORT, i2c_cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(i2c_cmd);
        give_i2c_mutex();

        if (ret == ESP_OK) {
            syslog(INFO, "SSD1306 detected at address 0x%02X", s_i2c_addr);
            return ESP_OK;
        }
    }

    // 0x3Dを試す
    s_i2c_addr = SSD1306_ADDR_2;
    if (take_i2c_mutex(pdMS_TO_TICKS(100))) {
        i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
        i2c_master_start(i2c_cmd);
        i2c_master_write_byte(i2c_cmd, (s_i2c_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(i2c_cmd);
        esp_err_t ret = i2c_master_cmd_begin(SSD1306_I2C_PORT, i2c_cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(i2c_cmd);
        give_i2c_mutex();

        if (ret == ESP_OK) {
            syslog(INFO, "SSD1306 detected at address 0x%02X", s_i2c_addr);
            return ESP_OK;
        }
    }

    syslog(ERR, "SSD1306 not detected at 0x%02X or 0x%02X", SSD1306_ADDR_1, SSD1306_ADDR_2);
    return ESP_FAIL;
}

// ==== SSD1306初期化 ====
static esp_err_t ssd1306_init_hw(void)
{
    // I2Cアドレス検出
    if (ssd1306_detect_address() != ESP_OK) {
        return ESP_FAIL;
    }

    // 初期化シーケンス
    ssd1306_send_command(SSD1306_CMD_DISPLAY_OFF);
    vTaskDelay(pdMS_TO_TICKS(10));

    ssd1306_send_command(SSD1306_CMD_SET_DISPLAY_CLOCK);
    ssd1306_send_command(0x80);  // クロック設定

    ssd1306_send_command(SSD1306_CMD_SET_MULTIPLEX);
    ssd1306_send_command(SSD1306_HEIGHT - 1);  // 64-1 = 63

    ssd1306_send_command(SSD1306_CMD_SET_DISPLAY_OFFSET);
    ssd1306_send_command(0x00);  // オフセットなし

    ssd1306_send_command(SSD1306_CMD_SET_START_LINE | 0x00);

    ssd1306_send_command(SSD1306_CMD_CHARGE_PUMP);
    ssd1306_send_command(0x14);  // 内部チャージポンプ有効

    ssd1306_send_command(SSD1306_CMD_MEMORY_MODE);
    ssd1306_send_command(0x00);  // 水平アドレッシングモード

    ssd1306_send_command(SSD1306_CMD_SEG_REMAP | 0x01);  // セグメントリマップ

    ssd1306_send_command(SSD1306_CMD_COM_SCAN_DEC);  // COMスキャン方向

    ssd1306_send_command(SSD1306_CMD_SET_COM_PINS);
    ssd1306_send_command(0x12);  // COMピン設定（128x64の場合）

    ssd1306_send_command(SSD1306_CMD_SET_CONTRAST);
    ssd1306_send_command(0xCF);  // コントラスト設定

    ssd1306_send_command(SSD1306_CMD_SET_PRECHARGE);
    ssd1306_send_command(0xF1);  // プリチャージ設定

    ssd1306_send_command(SSD1306_CMD_SET_VCOM_DETECT);
    ssd1306_send_command(0x40);  // VCOM検出設定

    ssd1306_send_command(SSD1306_CMD_DISPLAY_ALL_ON_RESUME);
    ssd1306_send_command(SSD1306_CMD_NORMAL_DISPLAY);

    ssd1306_send_command(SSD1306_CMD_DEACTIVATE_SCROLL);

    ssd1306_send_command(SSD1306_CMD_DISPLAY_ON);

    vTaskDelay(pdMS_TO_TICKS(100));

    // 画面クリア
    ssd1306_clear();
    ssd1306_display();

    syslog(INFO, "SSD1306 initialized");
    return ESP_OK;
}

// ==== ボタンイベント通知関数（割り込みハンドラから呼び出し） ====
void IRAM_ATTR ssd1306_notify_sw1_press(void)
{
    if (s_button_queue) {
        // デバウンス処理：最後に押された時刻をチェック
        TickType_t current_tick = xTaskGetTickCountFromISR();
        
        // 初回押下、または0.5秒以上経過している場合のみ処理
        if (s_sw1_last_press == 0 || 
            (current_tick - s_sw1_last_press) >= pdMS_TO_TICKS(DEBOUNCE_TIME_MS)) {
            s_sw1_last_press = current_tick;  // 時刻を更新
            
            button_event_t event = BUTTON_EVENT_SW1;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(s_button_queue, &event, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        // 0.5秒以内の場合は無視（チャタリング対策）
    }
}

void IRAM_ATTR ssd1306_notify_sw2_press(void)
{
    if (s_button_queue) {
        // デバウンス処理：最後に押された時刻をチェック
        TickType_t current_tick = xTaskGetTickCountFromISR();
        
        // 初回押下、または0.5秒以上経過している場合のみ処理
        if (s_sw2_last_press == 0 || 
            (current_tick - s_sw2_last_press) >= pdMS_TO_TICKS(DEBOUNCE_TIME_MS)) {
            s_sw2_last_press = current_tick;  // 時刻を更新
            
            button_event_t event = BUTTON_EVENT_SW2;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(s_button_queue, &event, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        // 0.5秒以内の場合は無視（チャタリング対策）
    }
}

// ==== 公開関数 ====
void start_ssd1306_task(void)
{
    if (s_task) {
        return;
    }

    // ボタンイベントキュー作成
    s_button_queue = xQueueCreate(10, sizeof(button_event_t));
    if (s_button_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button queue");
        return;
    }

    // I2C初期化（既に初期化されている場合はスキップ）
    if (i2c_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialization failed");
        return;
    }

    // SSD1306初期化
    if (ssd1306_init_hw() != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 initialization failed");
        return;
    }

    // タスク作成
    xTaskCreatePinnedToCore(ssd1306_task_entry, "ssd1306Task", 4096, NULL, 2, &s_task, 1);
    syslog(INFO, "ssd1306Task started");
}

void ssd1306_clear(void)
{
    memset(s_display_buffer, 0, sizeof(s_display_buffer));
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, bool on)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }

    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    uint16_t index = page * SSD1306_WIDTH + x;

    if (on) {
        s_display_buffer[index] |= (1 << bit);
    } else {
        s_display_buffer[index] &= ~(1 << bit);
    }
}

void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str)
{
    uint8_t cx = x;
    const char *p = str;
    
    while (*p && cx < SSD1306_WIDTH - 6) {
        uint8_t c = (uint8_t)*p;
        if (c >= 32 && c <= 127) {
            const uint8_t *font_data = font_5x7[c - 32];
            uint8_t base_y = y * 8;
            
            // 各文字は5ピクセル幅
            for (uint8_t i = 0; i < 5; i++) {
                uint8_t font_byte = font_data[i];
                for (uint8_t j = 0; j < 7; j++) {
                    if (font_byte & (1 << j)) {
                        ssd1306_draw_pixel(cx + i, base_y + j, true);
                    }
                }
            }
        }
        cx += 6;  // 文字間隔を含めて6ピクセル
        p++;
    }
}

void ssd1306_display(void)
{
    // ページアドレス設定
    ssd1306_send_command(SSD1306_CMD_PAGE_ADDR);
    ssd1306_send_command(0x00);
    ssd1306_send_command(SSD1306_PAGES - 1);

    // カラムアドレス設定
    ssd1306_send_command(SSD1306_CMD_COLUMN_ADDR);
    ssd1306_send_command(0x00);
    ssd1306_send_command(SSD1306_WIDTH - 1);

    // データ送信
    ssd1306_send_data(s_display_buffer, sizeof(s_display_buffer));
}

void ssd1306_invert_display(bool invert)
{
    s_display_inverted = invert;
    if (invert) {
        ssd1306_send_command(SSD1306_CMD_INVERT_DISPLAY);
    } else {
        ssd1306_send_command(SSD1306_CMD_NORMAL_DISPLAY);
    }
}

void ssd1306_set_contrast(uint8_t contrast)
{
    ssd1306_send_command(SSD1306_CMD_SET_CONTRAST);
    ssd1306_send_command(contrast);
}

// ==== モード実行処理 ====
static void execute_mode_action(ssd1306_display_mode_t mode, TickType_t current_tick)
{
    s_executing = true;
    s_result_message[0] = '\0';
    
    switch (mode) {
        case SSD1306_MODE_RESET:
            // モード1: reset?
            snprintf(s_result_message, sizeof(s_result_message), "exec reset");
            s_result_timer = current_tick + pdMS_TO_TICKS(500);  // 0.5秒後にリセット実行
            break;
            
        case SSD1306_MODE_SSID_0:
            // モード2: change SSID No0?
            setSsidNo(0);
            flashdata_save();
            snprintf(s_result_message, sizeof(s_result_message), "change ssid PE_IOT_GATEWAY_0");
            s_result_timer = current_tick + pdMS_TO_TICKS(3000);  // 3秒間表示
            break;
            
        case SSD1306_MODE_SSID_1:
            // モード3: change SSID No1?
            setSsidNo(1);
            flashdata_save();
            snprintf(s_result_message, sizeof(s_result_message), "change ssid PE_IOT_GATEWAY_1");
            s_result_timer = current_tick + pdMS_TO_TICKS(3000);  // 3秒間表示
            break;
            
        case SSD1306_MODE_SSID_2:
            // モード4: change SSID No2?
            setSsidNo(2);
            flashdata_save();
            snprintf(s_result_message, sizeof(s_result_message), "change ssid PE_IOT_GATEWAY_2");
            s_result_timer = current_tick + pdMS_TO_TICKS(3000);  // 3秒間表示
            break;
            
        case SSD1306_MODE_SSID_3:
            // モード5: change SSID No3?
            setSsidNo(3);
            flashdata_save();
            snprintf(s_result_message, sizeof(s_result_message), "change ssid PE_IOT_GATEWAY_3");
            s_result_timer = current_tick + pdMS_TO_TICKS(3000);  // 3秒間表示
            break;
            
        case SSD1306_MODE_SSID_4:
            // モード6: change SSID No4?
            setSsidNo(4);
            flashdata_save();
            snprintf(s_result_message, sizeof(s_result_message), "change ssid PE_IOT_GATEWAY_4");
            s_result_timer = current_tick + pdMS_TO_TICKS(3000);  // 3秒間表示
            break;
            
        case SSD1306_MODE_WIFI_PASS:
            // モード7: forget wifi password?
            snprintf(s_result_message, sizeof(s_result_message), "wifi pass is 12345678");
            s_result_timer = current_tick + pdMS_TO_TICKS(3000);  // 3秒間表示
            break;
            
        default:
            s_executing = false;
            s_result_timer = 0;
            break;
    }
}

// ==== タスクエントリ ====
static void ssd1306_task_entry(void *arg)
{
    char line[64];
    button_event_t button_event;
    TickType_t current_tick;

    while (1) {
        current_tick = xTaskGetTickCount();
        
        // ボタンイベントをチェック（非ブロッキング）
        while (xQueueReceive(s_button_queue, &button_event, 0) == pdTRUE) {
            if (button_event == BUTTON_EVENT_SW1) {
                // SW1押下：モードをインクリメント（0→1→2→...→7→0）
                if (!s_executing) {
                    s_display_mode = (s_display_mode + 1) % 8;
                    s_mode_timer = current_tick + pdMS_TO_TICKS(10000);  // 10秒後にタイムアウト
                    s_result_message[0] = '\0';  // 結果メッセージをクリア
                    syslog(INFO, "SSD1306: Mode changed to %d", s_display_mode);
                }
            } else if (button_event == BUTTON_EVENT_SW2) {
                // SW2押下：モード0以外の時、実行
                if (!s_executing && s_display_mode != SSD1306_MODE_SENSOR) {
                    execute_mode_action(s_display_mode, current_tick);
                    s_mode_timer = current_tick + pdMS_TO_TICKS(10000);  // タイマーリセット
                }
            }
        }
        
        // 実行結果タイマーチェック
        if (s_executing && s_result_timer != 0) {
            if (current_tick >= s_result_timer) {
                if (s_display_mode == SSD1306_MODE_RESET) {
                    // リセットモードの場合はリセット実行
                    exec_soft_reset();  // リセット実行（この後は実行されない）
                } else {
                    // その他のモードは実行フラグをリセット
                    s_executing = false;
                    s_result_timer = 0;
                }
            }
        }
        
        // モードのタイマーチェック（10秒経過でモード0に戻る）
        if (s_display_mode != SSD1306_MODE_SENSOR && s_mode_timer != 0) {
            if (current_tick >= s_mode_timer) {
                s_display_mode = SSD1306_MODE_SENSOR;
                s_mode_timer = 0;
                s_executing = false;
                s_result_timer = 0;
                s_result_message[0] = '\0';
                syslog(INFO, "SSD1306: Mode changed to SENSOR (timeout)");
            }
        }
        
        // 画面クリア
        ssd1306_clear();

        if (s_display_mode == SSD1306_MODE_SENSOR) {
            // モード0：センサ情報表示
            for (uint8_t child_no = 1; child_no <= 4; child_no++) {
                temp_sens_data_t data;
                bool valid = web_server_get_child_sensor_data(child_no, &data);
                
                if (valid && data.aht_ok && data.bmp_ok) {
                    // 有効なデータがある場合
                    float temp = (float)data.aht_t01 / 10.0f;
                    float rh = (float)data.aht_rh01 / 10.0f;
                    float pressure = (float)data.bmp_p01 / 10.0f;
                    
                    snprintf(line, sizeof(line), "%d  %.1fC  %.1f%%  %.1fhPa",
                             child_no, temp, rh, pressure);
                } else {
                    // 無効なデータまたは通信できなかった場合
                    snprintf(line, sizeof(line), "%d  0.0C  0.0%%  0.0hPa", child_no);
                }
                
                // 各行を表示（ページ0-3、各ページは8ピクセル高さ）
                ssd1306_draw_string(0, child_no - 1, line);
            }
        } else {
            // モード1～7：各モードの表示
            if (s_executing && s_result_message[0] != '\0') {
                // 実行結果を表示（最大21文字/行、最大8行）
                const char *msg = s_result_message;
                uint8_t line_num = 0;
                size_t msg_len = strlen(msg);
                
                // メッセージを21文字ずつに分割して表示
                while (msg_len > 0 && line_num < 8) {
                    size_t line_len = (msg_len > 21) ? 21 : msg_len;
                    char line_buf[22];
                    strncpy(line_buf, msg, line_len);
                    line_buf[line_len] = '\0';
                    ssd1306_draw_string(0, line_num, line_buf);
                    msg += line_len;
                    msg_len -= line_len;
                    line_num++;
                }
            } else {
                // 通常のモード表示
                switch (s_display_mode) {
                    case SSD1306_MODE_RESET:
                        snprintf(line, sizeof(line), "reset?");
                        break;
                    case SSD1306_MODE_SSID_0:
                        snprintf(line, sizeof(line), "change SSID No0?");
                        break;
                    case SSD1306_MODE_SSID_1:
                        snprintf(line, sizeof(line), "change SSID No1?");
                        break;
                    case SSD1306_MODE_SSID_2:
                        snprintf(line, sizeof(line), "change SSID No2?");
                        break;
                    case SSD1306_MODE_SSID_3:
                        snprintf(line, sizeof(line), "change SSID No3?");
                        break;
                    case SSD1306_MODE_SSID_4:
                        snprintf(line, sizeof(line), "change SSID No4?");
                        break;
                    case SSD1306_MODE_WIFI_PASS:
                        snprintf(line, sizeof(line), "forget wifi password?");
                        break;
                    default:
                        snprintf(line, sizeof(line), "unknown mode");
                        break;
                }
                ssd1306_draw_string(0, 0, line);
            }
        }
        
        // 画面更新
        ssd1306_display();

        // 100ms待機（応答性向上）
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
