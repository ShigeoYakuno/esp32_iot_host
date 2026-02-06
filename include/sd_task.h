#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void start_sd_task(void);
bool sd_enqueue_line(const char *line);
bool sd_is_mounted(void);
void sd_force_unmount(void);

bool sd_request_flashdata_export(void);

#ifdef __cplusplus
}
#endif
