/**
 * @file enet_rmii_lan8720.c
 * @brief ESP32 + LAN8720 RMII Ethernet（esp_eth ドライバ使用）
 *
 * ピン割当:
 *   MDC=23, MDIO=18, TX_EN=21, TXD0=19, TXD1=22,
 *   RXD0=25, RXD1=26, CRS_DV=27, REF_CLK=GPIO0(入力), OSC_EN=GPIO17
 */
#include "enet_rmii_lan8720.h"
#include "log_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "lwip/ip4_addr.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static const char *TAG = "enet_rmii";

#define OSC_EN_GPIO   17  /* 50MHz発振器制御 (HIGH=有効) */
#define OSC_STABLE_MS 100 /* クロック安定待ち時間 [ms] */

/* PHY/EMAC 設定（固定） */
#define PHY_ADDR      1
#define MDC_GPIO      23
#define MDIO_GPIO     18
#define REF_CLK_GPIO  0   /* RMII REF_CLK 入力 (外部50MHz) */

static esp_netif_t *s_eth_netif = NULL;

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6];
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED: {
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        syslog(INFO, "enet_rmii: ETH LINK UP");
        syslog(INFO, "enet_rmii: MAC %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        eth_speed_t speed = ETH_SPEED_10M;
        eth_duplex_t duplex = ETH_DUPLEX_HALF;
        bool autonego = false;
        esp_eth_ioctl(eth_handle, ETH_CMD_G_SPEED, &speed);
        esp_eth_ioctl(eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex);
        esp_eth_ioctl(eth_handle, ETH_CMD_G_AUTONEGO, &autonego);
        syslog(INFO, "enet_rmii: Speed=%s Duplex=%s AutoNeg=%s",
               speed == ETH_SPEED_100M ? "100M" : "10M",
               duplex == ETH_DUPLEX_FULL ? "FULL" : "HALF",
               autonego ? "ON" : "OFF");
        break;
    }
    case ETHERNET_EVENT_DISCONNECTED:
        syslog(INFO, "enet_rmii: ETH LINK DOWN");
        break;
    case ETHERNET_EVENT_START:
        syslog(INFO, "enet_rmii: ETH STARTED");
        break;
    case ETHERNET_EVENT_STOP:
        syslog(INFO, "enet_rmii: ETH STOPPED");
        break;
    default:
        syslog(INFO, "enet_rmii: ETH event %ld", event_id);
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    syslog(INFO, "enet_rmii: Got IP: " IPSTR " / " IPSTR " gw " IPSTR,
             IP2STR(&ip_info->ip),
             IP2STR(&ip_info->netmask),
             IP2STR(&ip_info->gw));
}

/**
 * @brief 50MHz発振器を有効化 (GPIO17 = OSC_EN)
 */
static void enable_oscillator(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << OSC_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(OSC_EN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(5));   /* 一度LOWを維持 */
    gpio_set_level(OSC_EN_GPIO, 1); /* 発振器ON */
    vTaskDelay(pdMS_TO_TICKS(OSC_STABLE_MS));  /* クロック安定待ち */
    syslog(INFO, "50MHz oscillator enabled (GPIO%d)", OSC_EN_GPIO);
}

/**
 * @brief LAN8720 RMII Ethernet 初期化
 */
static esp_err_t eth_init_lan8720(esp_eth_handle_t *eth_handle_out)
{
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = PHY_ADDR;
    phy_config.reset_gpio_num = -1;
    phy_config.autonego_timeout_ms = 8000;  /* リンク確立まで余裕 */

    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_mdc_gpio_num = MDC_GPIO;
    emac_config.smi_mdio_gpio_num = MDIO_GPIO;
    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;  /* GPIO0入力 */

    syslog(INFO, "enet_rmii: creating MAC...");
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (mac == NULL) {
        ESP_LOGE(TAG, "esp_eth_mac_new_esp32 failed");
        return ESP_FAIL;
    }
    syslog(INFO, "enet_rmii: MAC OK, creating PHY...");

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
    if (phy == NULL) {
        ESP_LOGE(TAG, "esp_eth_phy_new_lan87xx failed");
        mac->del(mac);
        return ESP_FAIL;
    }
    syslog(INFO, "enet_rmii: PHY OK, installing driver...");

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;

    if (esp_eth_driver_install(&eth_config, &eth_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed");
        mac->del(mac);
        phy->del(phy);
        return ESP_FAIL;
    }
    syslog(INFO, "enet_rmii: driver_install OK");

    /* MACアドレス設定（LAA形式） */
    uint8_t base_mac[6];
    if (esp_read_mac(base_mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        uint8_t eth_mac[6] = {
            (uint8_t)(base_mac[0] | 0x02),
            base_mac[1], base_mac[2], base_mac[3], base_mac[4],
            (uint8_t)(base_mac[5] ^ 0x55)
        };
        esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, eth_mac);
    }

    *eth_handle_out = eth_handle;
    return ESP_OK;
}

static void enet_rmii_task(void *arg)
{
    (void)arg;
    syslog(INFO, "enet_rmii_task starting");

    /* 1. 発振器有効化（GPIO0に50MHzが来る前に必ず実行） */
    enable_oscillator();

    /* 2. esp_netif / event loop（未初期化なら） */
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
    }

    /* 3. Ethernet 初期化 */
    syslog(INFO, "enet_rmii: eth_init_lan8720 start");
    esp_eth_handle_t eth_handle = NULL;
    if (eth_init_lan8720(&eth_handle) != ESP_OK) {
        ESP_LOGE(TAG, "eth_init_lan8720 failed");
        vTaskDelete(NULL);
        return;
    }

    /* 4. esp_netif に登録 */
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    if (eth_netif == NULL) {
        syslog(ERR, "enet_rmii: esp_netif_new failed");
        esp_eth_driver_uninstall(eth_handle);
        vTaskDelete(NULL);
        return;
    }
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
    if (glue == NULL) {
        syslog(ERR, "enet_rmii: esp_eth_new_netif_glue failed");
        esp_netif_destroy(eth_netif);
        esp_eth_driver_uninstall(eth_handle);
        vTaskDelete(NULL);
        return;
    }

    if (esp_netif_attach(eth_netif, glue) != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_attach failed");
        esp_netif_destroy(eth_netif);
        esp_eth_del_netif_glue(glue);
        esp_eth_driver_uninstall(eth_handle);
        vTaskDelete(NULL);
        return;
    }

    /* 5. イベントハンドラ登録 */
    esp_event_handler_instance_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL, NULL);

    /* 6. スタティックIP設定（enet_set_ip で変更可能） */
    s_eth_netif = eth_netif;
    esp_netif_dhcpc_stop(eth_netif);
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 1, 50);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
    esp_netif_set_ip_info(eth_netif, &ip_info);

    /* Ethernetをデフォルトnetifに（ping応答等の優先） */
    esp_netif_set_default_netif(eth_netif);
    ESP_LOGI(TAG, "Static IP: 192.168.1.50");

    /* 7. Ethernet 開始 */
    if (esp_eth_start(eth_handle) != ESP_OK) {
        syslog(ERR, "enet_rmii: esp_eth_start failed");
    } else {
        syslog(INFO, "enet_rmii: LAN8720 init OK (PHY=%d)", PHY_ADDR);
    }


    /* TX診断用UDPソケット */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(54321),
        .sin_addr.s_addr = inet_addr("192.168.1.10"),
    };

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_netif_ip_info_t ip;
        esp_netif_get_ip_info(eth_netif, &ip);
        bool netif_up = esp_netif_is_netif_up(eth_netif);
        syslog(INFO, "enet_rmii: netif=%s IP=" IPSTR,
               netif_up ? "UP" : "DOWN",
               IP2STR(&ip.ip));

        if (sock >= 0) {
            const char *msg = "ESP32-ETH-TX-TEST";
            int tx_ret = sendto(sock, msg, strlen(msg), 0,
                             (struct sockaddr *)&dest, sizeof(dest));
            if (tx_ret < 0) {
                syslog(ERR, "enet_rmii: UDP TX FAIL errno=%d", errno);
            } else {
                syslog(INFO, "enet_rmii: UDP TX OK ret=%d", tx_ret);
            }
        }
    }
}

void start_enet_rmii_lan8720_task(void)
{
    xTaskCreate(enet_rmii_task, "enet_rmii", 8192, NULL, 5, NULL);
}

bool enet_set_ip(uint8_t host_id)
{
    if (s_eth_netif == NULL || host_id < 1 || host_id > 200) {
        return false;
    }
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 1, host_id);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
    esp_err_t ret = esp_netif_set_ip_info(s_eth_netif, &ip_info);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "IP set to 192.168.1.%d", host_id);
    }
    return (ret == ESP_OK);
}
