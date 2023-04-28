#ifndef WIFICONNECT_H
#define WIFICONNECT_H

#include <sys/param.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include <string>

#define NETIF_DESC_STA "netif_sta"
#define CONFIG_WIFI_CONN_MAX_RETRY 10

class WifiConnect
{
private:
    static esp_netif_t *s_sta_netif;
    static SemaphoreHandle_t s_semph_get_ip_addrs;
    static int s_retry_num;

    esp_err_t wifi_sta_do_connect(wifi_config_t wifi_config, bool wait);
    esp_err_t wifi_connect(const char *ssid, const char *password);
    static void handler_on_sta_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base, int32_t event_id, void *event_data);
    void wifi_start(void);
    static esp_err_t wifi_sta_do_disconnect(void);
    static void wifi_stop(void);
    static void wifi_shutdown(void);
    void print_all_netif_ips(const char *prefix);
public:
    WifiConnect(/* args */);
    ~WifiConnect();

    esp_err_t connect(const char *ssid, const char *password);
};

#endif
