#include <Arduino.h>

#include <driver/sdmmc_defs.h>
#include <driver/sdmmc_types.h>
#include <driver/sdmmc_host.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <sdmmc_cmd.h>
#include <esp_vfs_fat.h>
#include <nvs_flash.h>
#include <esp_vfs.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "freertos/semphr.h"

#include <Update.h>

// local files
#include "pinmap.h"
#include "MatrixPanel.h"
#include "Network/WifiConnect.h"


#define EXAMPLE_NETIF_DESC_STA "example_netif_sta"
#define CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY 10

#define ARTNET_DATA         0x50
#define ARTNET_PORT         6454
#define ARTNET_HEADER       17

#define ARTNET_POLL         0x2000
#define ARTNET_POLL_REPLY   0x2100
#define ARTNET_DMX          0x5000
#define ARTNET_SYNC         0x5200

static const char *TAG = "espressif"; // TAG for debug

MatrixPanel matrixPanel(32,32);
WifiConnect wifi;

// Mount path for the partition
static sdmmc_card_t *mount_card = NULL;
#define MOUNT_POINT 	"/sdcard"

xQueueHandle interputQueue;

SemaphoreHandle_t Mutex = NULL;

int buttonCount = 0;

std::vector<String> files;
int file_count = 0;
int current_file_idx = 0;

struct art_poll_reply {
    uint8_t id[8];
    uint8_t this_ip[4];
    uint16_t opcode = ARTNET_POLL_REPLY;
    uint16_t port_number = 0x1936;
    uint16_t fw_version = 0;
    uint8_t net_switch = 0;
    uint8_t sub_switch = 0;
    uint16_t oem = 0;
    uint8_t ubea_version = 0;
    uint8_t status1 = 0b11100000;
    uint16_t esta_code = 0;
    uint8_t short_name[18];
    uint8_t long_name[64];
    uint8_t node_report[64];
    uint16_t num_ports = 0;
    uint8_t port_types[4];
    uint8_t good_input[4];
    uint8_t good_output[4];
    uint8_t sw_in[4];
    uint8_t sw_out[4];
    uint8_t style = 0;
    uint8_t mac_addr[6];
    uint8_t bind_ip[4];
    uint8_t bind_idx = 0;
    uint8_t status2 = 0b00001110;
};

float clamp_value(float num, float mini, float maxi){
    return min(max(maxi,num),mini);
}

void init_fat() {
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = (gpio_num_t)PIN_NUM_MOSI,
        .miso_io_num = (gpio_num_t)PIN_NUM_MISO,
        .sclk_io_num = (gpio_num_t)PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)PIN_NUM_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &mount_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    sdmmc_card_print_info(stdout, mount_card);
}

void parseDMX(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data) {
    //printf("universe: %i, length: %i, sequence: %i\r\n", universe, length, sequence);
    for (int j=0; j<7; j++) {
        if (universe == j) {
            for (int i=0; i<32*5; i++) {
                matrixPanel.drawPixel(i%32, 5*j + i/32, data[i*3 + 0], data[i*3 + 1], data[i*3 + 2]);
            }
            
            matrixPanel.drawBuffer();
        }
    }
}

static void udp_server_task(void *pvParameters) {
    uint8_t rx_buffer[530];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(ARTNET_PORT);
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", ARTNET_PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;

        while (1) {
            //ESP_LOGI(TAG, "Waiting for data");
            int len = recvmsg(sock, &msg, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                //inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                /*for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                    if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                        struct in_pktinfo *pktinfo;
                        pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                        ESP_LOGI(TAG, "dest ip: %s\n", inet_ntoa(pktinfo->ipi_addr));
                    }
                }*/
                uint16_t uniSize;

                if ( rx_buffer[0] == 'A' && rx_buffer[1] == 'r' && rx_buffer[2] == 't' && rx_buffer[3] == '-' && rx_buffer[4] == 'N' && rx_buffer[5] == 'e' && rx_buffer[6] == 't') {
                    int opcode = rx_buffer[8] | rx_buffer[9] << 8;

                    if (opcode == ARTNET_DMX)
                    {
                        int sequence = rx_buffer[12];
                        int subUni = rx_buffer[14];
                        int net = rx_buffer[15];
                        int universe = subUni + net<<8;
                        int dmxDataLength = rx_buffer[17] + rx_buffer[16] << 8;
                        parseDMX(subUni, dmxDataLength, sequence, rx_buffer + (ARTNET_HEADER + 1));
                    }

                    if (opcode == ARTNET_POLL)
                    {
                        // https://art-net.org.uk/how-it-works/discovery-packets/artpollreply/
                        printf("poll\r\n");
                        //uint8_t poll_reply[214] = {0};
                        //struct art_poll_reply reply;

                        //memcpy(reply.id, "Art-Net", 8);

                        //uint64_t this_ip_adress = (uint64_t)(struct sockaddr_in *)&dest_addr;
                        //reply.this_ip[0] = (this_ip_adress & 0xFF000000L) >> 24;
                        //reply.this_ip[1] = (this_ip_adress & 0x00FF0000L) >> 16;
                        //reply.this_ip[2] = (this_ip_adress & 0x0000FF00L) >> 8;
                        //reply.this_ip[3] = (this_ip_adress & 0x000000FFL) >> 0;

                        //int err = sendto(sock, poll_reply, sizeof(poll_reply), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                        //if (err < 0) {
                        //    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        //    break;
                        //}
                    }
                    if (opcode == ARTNET_SYNC)
                    {
                        printf("sync\r\n");
                    }
                }

                /*int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }*/
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void setup() {
    //i2c_master_init();

    //acellero_init();

    // Initialize rtc
    //if (getSeconds() == 0) {
    //    uint8_t rtc_config[1] = {0x80};
    //    ESP_ERROR_CHECK(i2c_master_write_slave_reg(I2C_NUM_0, 0x6F, 0x00, rtc_config, 1));
    //}

    //init_fat();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi.connect("TPLink-ciska-alain", "alainloveciska1971");
    
    setCpuFrequencyMhz(240);

    //list_files();

    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
}

void loop() {
    matrixPanel.drawBuffer();
    delay(100);
}