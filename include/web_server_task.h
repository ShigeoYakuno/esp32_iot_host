#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// ==== センサーデータ構造体定義 ====
// 子機からのセンサーデータを受信するために使用
typedef struct temp_sens_data {
    int16_t aht_t01;    // AHT温度（0.1℃単位、例：253 = 25.3℃）
    uint16_t aht_rh01;  // AHT湿度（0.1%単位、例：482 = 48.2%）
    int16_t bmp_t01;    // BMP温度（0.1℃単位、例：251 = 25.1℃）
    uint32_t bmp_p01;   // BMP気圧（0.1hPa単位、例：100845 = 10084.5hPa）
    bool aht_ok;        // 今回のAHT測定成功フラグ
    bool bmp_ok;        // 今回のBMP測定成功フラグ
    uint32_t seq;       // 送信ごとにインクリメントする連番
    int rssi;           // RSSI値（dBm、通常-100～0、取得失敗時は0）
} temp_sens_data_t;

void start_web_server_task(void);
void web_server_update_sensor_data(const temp_sens_data_t *data);  // 後方互換性用
void web_server_update_sensor_data_with_child_no(uint8_t child_no, const temp_sens_data_t *data);

// 子機センサーデータ取得（SSD1306表示用）
// child_no: 1-4
// 戻り値: true=有効データ, false=無効データ
bool web_server_get_child_sensor_data(uint8_t child_no, temp_sens_data_t *data);

#ifdef __cplusplus
}
#endif
