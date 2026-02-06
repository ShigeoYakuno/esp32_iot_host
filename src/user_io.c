#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "log_task.h"
#include "user_io.h"
#include "ssd1306_task.h"


// ==== 割り込みハンドラ ====
static void IRAM_ATTR sw_isr_handler(void *arg)
{
    // SSD1306タスクに通知
    ssd1306_notify_sw1_press();
}

static void IRAM_ATTR sw2_isr_handler(void *arg)
{
    // SSD1306タスクに通知
    ssd1306_notify_sw2_press();
}

// ==== 初期化 ====
void init_userIO(void)
{
    //スイッチ入力設定

    gpio_config_t io_conf_sw1 = {
        .pin_bit_mask = 1ULL << USER_SW,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // 外部プルアップ使用
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // 押下でLOWになる想定
    };
    gpio_config(&io_conf_sw1);

    gpio_config_t io_conf_sw2 = {
        .pin_bit_mask = 1ULL << USER_SW2,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // 外部プルアップ使用
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf_sw2);


    // --- 割り込みサービス登録 ---
    gpio_install_isr_service(0);
    gpio_isr_handler_add(USER_SW, sw_isr_handler, NULL);
    gpio_isr_handler_add(USER_SW2, sw2_isr_handler, NULL);

    syslog(DEBUG,"UserIO initialized");
}


// ==== 公開関数 ====
void start_userIO_task(void)
{
    init_userIO();
}
