#include "Network/WifiConnect.h"
#include <cstring>

WifiConnect::WifiConnect(/* args */)
{
}

WifiConnect::~WifiConnect()
{
}

esp_netif_t *WifiConnect::s_sta_netif = NULL;
SemaphoreHandle_t WifiConnect::s_semph_get_ip_addrs = NULL;
int WifiConnect::s_retry_num = 0;

esp_err_t WifiConnect::wifi_sta_do_connect(wifi_config_t wifi_config, bool wait) {
    if (wait) {
        s_semph_get_ip_addrs = xSemaphoreCreateBinary();
        if (s_semph_get_ip_addrs == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_retry_num = 0;
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &handler_on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &handler_on_sta_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &handler_on_wifi_connect, s_sta_netif));

    ESP_LOGI("WifiConnect", "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE("WifiConnect", "WiFi connect failed! ret:%x", ret);
        return ret;
    }
    if (wait) {
        ESP_LOGI("WifiConnect", "Waiting for IP(s)");

        xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);

        if (s_retry_num > CONFIG_WIFI_CONN_MAX_RETRY) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_err_t WifiConnect::wifi_connect(const char *ssid, const char *password) {
    ESP_LOGI("WifiConnect", "Start example_connect.");
    printf("Start example_connect.\r\n");
    wifi_start();
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);

    return wifi_sta_do_connect(wifi_config, true);
}

bool is_our_netif(const char *prefix, esp_netif_t *netif) {
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

void WifiConnect::handler_on_sta_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    s_retry_num = 0;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!is_our_netif(NETIF_DESC_STA, event->esp_netif)) {
        return;
    }
    ESP_LOGI("WifiConnect", "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    if (s_semph_get_ip_addrs) {
        xSemaphoreGive(s_semph_get_ip_addrs);
    } else {
        ESP_LOGI("WifiConnect", "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));
    }
}

void WifiConnect::handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    s_retry_num++;
    if (s_retry_num > CONFIG_WIFI_CONN_MAX_RETRY) {
        ESP_LOGI("WifiConnect", "WiFi Connect failed %d times, stop reconnect.", s_retry_num);
        /* let example_wifi_sta_do_connect() return */
        if (s_semph_get_ip_addrs) {
            xSemaphoreGive(s_semph_get_ip_addrs);
        }
        return;
    }
    ESP_LOGI("WifiConnect", "Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return;
    }
    ESP_ERROR_CHECK(err);
}

void WifiConnect::handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI("WifiConnect", "Connected!");
}

void WifiConnect::wifi_start(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    esp_netif_config.if_desc = NETIF_DESC_STA;
    esp_netif_config.route_prio = 128;
    s_sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t WifiConnect::wifi_sta_do_disconnect(void) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &handler_on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &handler_on_sta_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &handler_on_wifi_connect));

    if (s_semph_get_ip_addrs) {
        vSemaphoreDelete(s_semph_get_ip_addrs);
    }

    return esp_wifi_disconnect();
}

void WifiConnect::wifi_stop(void) {
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(s_sta_netif));
    esp_netif_destroy(s_sta_netif);
    s_sta_netif = NULL;
}

void WifiConnect::wifi_shutdown(void) {
    wifi_sta_do_disconnect();
    wifi_stop();
}

esp_err_t WifiConnect::connect(const char *ssid, const char *password) {
    if (wifi_connect(ssid, password) != ESP_OK) {
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&wifi_shutdown));

    print_all_netif_ips(NETIF_DESC_STA);

    return ESP_OK;
}

void WifiConnect::print_all_netif_ips(const char *prefix) {
    // iterate over active interfaces, and print out IPs of "our" netifs
    esp_netif_t *netif = NULL;
    for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) {
        netif = esp_netif_next(netif);
        if (is_our_netif(prefix, netif)) {
            ESP_LOGI("WifiConnect", "Connected to %s", esp_netif_get_desc(netif));
            esp_netif_ip_info_t ip;
            ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip));

            ESP_LOGI("WifiConnect", "- IPv4 address: " IPSTR ",", IP2STR(&ip.ip));
        }
    }
}

