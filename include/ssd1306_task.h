#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "web_server_task.h"  // 子機センサーデータ取得用

// ==== SSD1306設定 ====
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES  (SSD1306_HEIGHT / 8)

// ==== 表示モード ====
typedef enum {
    SSD1306_MODE_SENSOR = 0,      // センサ情報表示（デフォルト）
    SSD1306_MODE_RESET = 1,        // reset? モード
    SSD1306_MODE_SSID_0 = 2,       // change SSID No0? モード
    SSD1306_MODE_SSID_1 = 3,       // change SSID No1? モード
    SSD1306_MODE_SSID_2 = 4,       // change SSID No2? モード
    SSD1306_MODE_SSID_3 = 5,       // change SSID No3? モード
    SSD1306_MODE_SSID_4 = 6,       // change SSID No4? モード
    SSD1306_MODE_WIFI_PASS = 7,    // forget wifi password? モード
} ssd1306_display_mode_t;

// ==== 公開関数 ====
void start_ssd1306_task(void);

// ボタンイベント通知（割り込みハンドラから呼び出し）
// 注: 実装側でIRAM_ATTRが付いているため、ここでは宣言のみ
void ssd1306_notify_sw1_press(void);
void ssd1306_notify_sw2_press(void);

// 画面クリア
void ssd1306_clear(void);

// 文字列表示（x: 0-127, y: 0-7 (ページ単位)）
void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str);

// ピクセル描画（x: 0-127, y: 0-63）
void ssd1306_draw_pixel(uint8_t x, uint8_t y, bool on);

// 画面更新（バッファをディスプレイに送信）
void ssd1306_display(void);

// 反転表示の設定
void ssd1306_invert_display(bool invert);

// コントラスト設定（0-255）
void ssd1306_set_contrast(uint8_t contrast);

#ifdef __cplusplus
}
#endif
