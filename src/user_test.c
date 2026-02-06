#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "log_task.h"
#include "user_test.h"
#include "freertos/portmacro.h"
#include "esp_rom_sys.h"
#include "enet_task.h" 
#include "flash_data.h"
#include "user_common.h"
#include "sd_task.h"
#include <string.h>
#include <stdlib.h>

#define TEST_UART_NUM UART_NUM_0
#define BUF_SIZE      128
#define CMD_BUF_SIZE  64




// 関数は残すがコマンドからは削除
__attribute__((unused)) static void sd_write_test(void)
{
    char line[128];
    snprintf(line, sizeof(line),"SDTEST,%lu",(unsigned long)xTaskGetTickCount());
    bool ok = sd_enqueue_line(line);
    syslog(INFO, ok ? "SD write queued: %s" : "SD queue full", line);
}

// 関数は残すがコマンドからは削除
__attribute__((unused)) static void setBTdeviceNo(void)
{
    syslog(INFO, "Enter BT Device Number (00-99):");

    char buf[4] = {0};
    int i = 0;
    while (i < 2) {
        int ch = fgetc(stdin);
        if (ch >= '0' && ch <= '9') buf[i++] = (char)ch;
        else if (ch == '\r' || ch == '\n') break;
    }
    buf[i] = '\0';
    int n = atoi(buf);

    setBtDevNum(n);

    syslog(INFO, "BT Device name changed to PE_DEV_%02d", n);
}



// ==== コマンド解析と実行 ====
static void execute_command(const char *cmd)
{
    if (cmd == NULL || strlen(cmd) == 0) {
        return;
    }
    
    // コマンド文字列の比較（大文字小文字を区別）
    if (strcmp(cmd, "dump") == 0) {
        flashdata_dump_all();
    }
    else if (strcmp(cmd, "load") == 0) {
        flashdata_load();
    }
    else if (strcmp(cmd, "save") == 0) {
        flashdata_save();
    }
    else if (strcmp(cmd, "reset") == 0) {
        exec_soft_reset();
    }
    else if (strcmp(cmd, "setbtdev1") == 0) {
        setBtDevNum(1);
    }
    else if (strcmp(cmd, "setbtdev2") == 0) {
        setBtDevNum(2);
    }
    else if (strcmp(cmd, "setbtdev3") == 0) {
        setBtDevNum(3);
    }
    else if (strncmp(cmd, "ipchg", 5) == 0) {
        // "ipchg" + 数字の形式
        const char *num_str = cmd + 5;
        int ip_num = atoi(num_str);
        if (ip_num > 0 && ip_num <= 255) {
            enet_set_ip((uint8_t)ip_num);
        } else {
            syslog(INFO, "Invalid IP number: %s (must be 1-255)", num_str);
        }
    }
    else if (strncmp(cmd, "ssidchg", 7) == 0) {
        // "ssidchg" + 数字の形式
        const char *num_str = cmd + 7;
        int ssid_no = atoi(num_str);
        if (ssid_no >= 0 && ssid_no <= 255) {
            setSsidNo((uint32_t)ssid_no);
            flashdata_save();  // フラッシュに保存
            syslog(INFO, "SSID changed to PE_IOT_GATEWAY_%d (reboot required)", ssid_no);
        } else {
            syslog(INFO, "Invalid SSID number: %s (must be 0-255)", num_str);
        }
    }
    else {
        syslog(INFO, "Unknown command: %s", cmd);
    }
}

void test_task(void *pvParameters)
{
    syslog(INFO, "test_task started (UART0 RX=GPIO3, CR/LF command mode)");

    // UART0はESP-IDF標準で初期化済み（ログ出力に使用されるため）
    char cmd_buf[CMD_BUF_SIZE] = {0};
    int cmd_idx = 0;

    while (1) {
        int c = fgetc(stdin);  // UART0のRX(GPIO3)から入力を読む
        if (c != EOF) {
            if (c == '\r' || c == '\n') {
                // CRまたはLFを受信したらコマンドを実行
                if (cmd_idx > 0) {
                    cmd_buf[cmd_idx] = '\0';  // null終端
                    execute_command(cmd_buf);
                    cmd_idx = 0;  // バッファをリセット
                    memset(cmd_buf, 0, sizeof(cmd_buf));
                }
            }
            else if (c >= 32 && c <= 126) {
                // 印字可能文字のみ受け付ける
                if (cmd_idx < CMD_BUF_SIZE - 1) {
                    cmd_buf[cmd_idx++] = (char)c;
                } else {
                    // バッファオーバーフロー
                    syslog(WARN, "Command buffer overflow, resetting");
                    cmd_idx = 0;
                    memset(cmd_buf, 0, sizeof(cmd_buf));
                }
            }
            // その他の制御文字は無視
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // watchdog防止
    }
}

void start_test_task(void)
{
    xTaskCreatePinnedToCore(test_task, "test_task", 4096, NULL, 4, NULL, 1);
}
