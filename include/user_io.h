#pragma once

#ifdef __cplusplus
extern "C" {
#endif


// ==== 定義 ====
#define USER_SW         GPIO_NUM_4   // SW1
#define USER_SW2        GPIO_NUM_34  // SW2 (入力専用ピン、外部プルアップ必要)




void start_userIO_task(void);
void init_userIO(void);

#ifdef __cplusplus
}
#endif
