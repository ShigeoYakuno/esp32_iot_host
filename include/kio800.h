#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ==== 公開関数 ====
void kio800_init(void);
void kio800_send_sensor_data(uint8_t child_no, int16_t temp_01c, uint16_t rh_01, uint32_t pressure_01hpa);

#ifdef __cplusplus
}
#endif
