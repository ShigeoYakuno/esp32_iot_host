#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "log_task.h"
#include "sd_task.h"

#include "flash_data.h"
#include <arpa/inet.h>  // inet_ntoa, etc.

// ==== GPIO割当（HSPI専用）====
#define SD_MOSI_PIN   13
#define SD_MISO_PIN   35
#define SD_SCLK_PIN   14
#define SD_CS_PIN     17

#define SD_MOUNT_POINT   "/sdcard"
#define SD_QUEUE_DEPTH   64
#define SD_LINE_LEN      128

typedef enum {
    SD_CMD_WRITE_LOG,
    SD_CMD_EXPORT_FLASHDATA,
} SdCommand;

typedef struct {
    SdCommand cmd;
    char line[128];
} SdMsg;


// ==== 内部オブジェクト ====
static const char *TAG = "sdTask";
static QueueHandle_t qSdCmd = NULL;
static QueueHandle_t qAggToSD = NULL;
static SemaphoreHandle_t mtxHSPI = NULL;
static SemaphoreHandle_t mtxSD   = NULL;
static TaskHandle_t sdTaskHandle = NULL;

static bool sd_mounted = false;
static sdmmc_card_t *sd_card = NULL;

// ============================================================
// SDカードのマウント処理
// ============================================================
bool mount_sdcard(void)
{
    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 1000;  // 1MHz
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    // 事前安定化
    gpio_set_direction(SD_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SD_CS_PIN, 1);
    gpio_set_pull_mode(SD_CS_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_MISO_PIN, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(10));

    // バス初期化（初回のみ）
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        syslog(ERR, "spi_bus_initialize failed (%s)", esp_err_to_name(ret));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // ======== ★ここから再試行ループ★ ========
    for (int attempt = 1; attempt <= 3; ++attempt) {
        syslog(INFO, "SD mount attempt %d ...", attempt);
        ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_cfg, &sd_card);
        if (ret == ESP_OK) {
            syslog(INFO, "SD mounted successfully (try=%d): %s", attempt, sd_card->cid.name);
            sd_mounted = true;
            return true;
        }

        syslog(WARN, "SD mount failed (try=%d): %s", attempt, esp_err_to_name(ret));

        // 一旦バス解放→再初期化（カード再認識試行）
        spi_bus_free(HSPI_HOST);
        vTaskDelay(pdMS_TO_TICKS(100));
        spi_bus_initialize(HSPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // ======== ★ここまで★ ========

    syslog(ERR, "SD init failed after 3 retries");
    return false;
}


// ============================================================
// SDカードのアンマウント処理
// ============================================================
static void unmount_sdcard(void)
{
    if (sd_mounted) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
        spi_bus_free(HSPI_HOST);
        syslog(INFO, "SD unmounted");
        sd_mounted = false;
    }
}

// ===================================================
// 外部からSD書き込み要求を出すAPI
// ===================================================
bool sd_enqueue_line(const char *line)
{
    if (!qSdCmd) return false;
    SdMsg msg = { .cmd = SD_CMD_WRITE_LOG };
    strncpy(msg.line, line, sizeof(msg.line) - 1);
    return (xQueueSend(qSdCmd, &msg, 0) == pdPASS);
}

// flashdata.csv出力要求
bool sd_request_flashdata_export(void)
{
    if (!qSdCmd) return false;
    SdMsg msg = { .cmd = SD_CMD_EXPORT_FLASHDATA };
    return (xQueueSend(qSdCmd, &msg, 0) == pdPASS);
}

// ===================================================
// CSV出力（実処理）
// ===================================================
static void sd_export_flashdata_csv(void)
{
    if (!sd_is_mounted()) {
        syslog(ERR, "SD not mounted, cannot export flashdata");
        return;
    }

    // マウント直後の安定待ち（特に初回は重要）
    vTaskDelay(pdMS_TO_TICKS(100));

    xSemaphoreTake(mtxSD, portMAX_DELAY);

    FILE *fp = NULL;
    fp = fopen(SD_MOUNT_POINT "/setdata.csv", "a");//FATの8.3ファイル名制限で8文字以下でないといけない
    if (!fp) {
        syslog(WARN, "Failed to open flashdata.csv");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!fp) {
        syslog(ERR, "Failed to open flashdata.csv after retries");
        xSemaphoreGive(mtxSD);
        return;
    }

    fprintf(fp, "ID,NAME,VALUE_HEX,VALUE_DEC,VALUE_STR\n");

    for (int i = 0; i < E2DATA_MAX; i++) {
        const char *name = NULL;
        UW val = 0;
        char strval[64] = "";

        switch (i) {
            case IP_ADDR: {
                name = "IP_ADDR";  val = getIpAddr();
                snprintf(strval, sizeof(strval), "%lu.%lu.%lu.%lu",
                         (val >> 24) & 0xFF, (val >> 16) & 0xFF,
                         (val >> 8) & 0xFF, val & 0xFF);
                break;
            }
            case BT_DEV_NO:  name = "BT_DEV_NO";  val = getBtDevNum();  break;
            case CRC_CALC:   name = "CRC_CALC";   val = 0;              break;
            default: continue;
        }

        if (strval[0] == '\0')
            snprintf(strval, sizeof(strval), "%lu", val);

        fprintf(fp, "%d,%s,0x%08lX,%lu,%s\n", i, name, val, val, strval);
    }

    fclose(fp);
    xSemaphoreGive(mtxSD);

    syslog(INFO, "flashdata.csv exported to SD");
}






// ============================================================
// SD書込み本体ループ
// ============================================================
void sd_task(void *pv)
{
    syslog(INFO, "sd_task started (HSPI) GPIO14/13/19/17");

    mtxSD = xSemaphoreCreateMutex();
    qSdCmd = xQueueCreate(16, sizeof(SdMsg));

    if (!mount_sdcard()) {
        syslog(ERR, "SD init failed");
    }

    SdMsg msg;
    for (;;) {
        if (xQueueReceive(qSdCmd, &msg, pdMS_TO_TICKS(100))) {
            switch (msg.cmd) {
                case SD_CMD_WRITE_LOG: {
                    xSemaphoreTake(mtxSD, portMAX_DELAY);
                    FILE *fp = fopen(SD_MOUNT_POINT "/log.csv", "a");
                    if (fp) {
                        fprintf(fp, "%s\n", msg.line);
                        fclose(fp);
                        syslog(INFO, "[SD] wrote: %s", msg.line);
                    } else {
                        syslog(ERR, "fopen(log.csv) failed");
                    }
                    xSemaphoreGive(mtxSD);
                    break;
                }
                case SD_CMD_EXPORT_FLASHDATA:
                    sd_export_flashdata_csv();
                    break;
            }
        }
    }
}

// ============================================================
// 外部インタフェース
// ============================================================

// --- sdTask起動 ---
void start_sd_task(void)
{
    if (!qAggToSD) qAggToSD = xQueueCreate(SD_QUEUE_DEPTH, SD_LINE_LEN);
    if (!mtxHSPI)  mtxHSPI  = xSemaphoreCreateMutex();
    if (!mtxSD)    mtxSD    = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(sd_task, "sd_task", 4096, NULL, 5, &sdTaskHandle, 1);
    syslog(INFO, "sd_task created");
}



// --- マウント状態取得 ---
bool sd_is_mounted(void)
{
    return sd_mounted;
}

// --- 強制アンマウント ---
void sd_force_unmount(void)
{
    unmount_sdcard();
}

