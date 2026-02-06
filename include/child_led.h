#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==== 子機LED定義 ====
#define CHILD_LED1  GPIO_NUM_2   // 子機1
#define CHILD_LED2  GPIO_NUM_21  // 子機2
#define CHILD_LED3  GPIO_NUM_22  // 子機3
#define CHILD_LED4  GPIO_NUM_15  // 子機4

// ==== 公開関数 ====
void init_child_leds(void);
void toggle_child_led(uint8_t child_no);  // child_no: 1-4

#ifdef __cplusplus
}
#endif
