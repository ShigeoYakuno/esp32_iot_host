/**
 * @file enet_rmii_lan8720.h
 * @brief ESP32 + LAN8720 RMII Ethernet 初期化（OSC_EN制御付き）
 *
 * ハード仕様:
 *   - PHY: LAN8720, RMII, アドレス1
 *   - REF_CLK: GPIO0 (外部50MHz入力)
 *   - OSC_EN: GPIO17 (発振器ON/OFF制御、起動後に有効化)
 *   - MDC: GPIO23, MDIO: GPIO18
 */
#ifndef ENET_RMII_LAN8720_H
#define ENET_RMII_LAN8720_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LAN8720 RMII Ethernet タスクを起動
 *
 * 内部で以下を実行:
 *   1. GPIO17 (OSC_EN) を HIGH にして 50MHz 発振器を有効化
 *   2. クロック安定待ち (約50ms)
 *   3. esp_eth + esp_netif による Ethernet 初期化・起動
 */
void start_enet_rmii_lan8720_task(void);

/**
 * @brief IPアドレスを 192.168.1.{host_id} に変更
 * @param host_id 1–200
 * @return true=成功, false=範囲外 or 未初期化
 */
bool enet_set_ip(uint8_t host_id);

#ifdef __cplusplus
}
#endif

#endif /* ENET_RMII_LAN8720_H */
