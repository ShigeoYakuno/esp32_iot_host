// src/child_led.c
// 子機用LED制御

#include "child_led.h"
#include "driver/gpio.h"
#include "log_task.h"
#include <stdint.h>

// ==== LEDピン定義 ====
static const gpio_num_t child_led_pins[4] = {
    CHILD_LED1,  // 子機1
    CHILD_LED2,  // 子機2
    CHILD_LED3,  // 子機3
    CHILD_LED4,  // 子機4
};

// ==== LED状態（トグル用） ====
static bool child_led_states[4] = {false, false, false, false};

// ==== 初期化 ====
void init_child_leds(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(child_led_pins[i]);
        gpio_set_direction(child_led_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(child_led_pins[i], GPIO_FLOATING);
        gpio_set_drive_capability(child_led_pins[i], GPIO_DRIVE_CAP_DEFAULT);
        gpio_set_level(child_led_pins[i], 0);
        child_led_states[i] = false;
    }
    syslog(INFO, "Child LEDs initialized (GPIO2,21,22,15)");
}

// ==== LEDトグル ====
void toggle_child_led(uint8_t child_no)
{
    if (child_no < 1 || child_no > 4) {
        return;
    }
    
    int idx = child_no - 1;  // 0-3に変換
    child_led_states[idx] = !child_led_states[idx];
    gpio_set_level(child_led_pins[idx], child_led_states[idx] ? 1 : 0);
}
