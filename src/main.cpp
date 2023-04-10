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

#include <sys/param.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "freertos/semphr.h"

#include <Update.h>

// local files
#include "pinmap.h"
#include "MatrixPanel.h"
#include "Graphics3D.h"
#include "Lua_libs/Lua_matrix_libs.h"
#include "Lua_libs/Lua_box2d_libs.h"
#include "FastMath.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#define EXAMPLE_NETIF_DESC_STA "example_netif_sta"
#define CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY 10

#define ARTNET_DATA         0x50
#define ARTNET_POLL         0x20
#define ARTNET_POLL_REPLY   0x21
#define ARTNET_PORT         6454
#define ARTNET_HEADER       17

#define ARTNET_POLL         0x2000
#define ARTNET_POLL_REPLY   0x2100
#define ARTNET_DMX          0x5000
#define ARTNET_SYNC         0x5200

const char* ssid = "ssid";
const char* password = "password";

static const char *TAG = "espressif"; // TAG for debug

lua_State *lua_state;
MatrixPanel matrixPanel(32,32);
Graphics3D graphics3D(&matrixPanel);

Lua_matrix_libs lua_matrix_libs(lua_state, &matrixPanel, &graphics3D);
Lua_box2d_libs lua_box2d_libs(lua_state, &matrixPanel);

// Mount path for the partition
static sdmmc_card_t *mount_card = NULL;
#define MOUNT_POINT 	"/sdcard"

xQueueHandle interputQueue;

TaskHandle_t lua_task_handle;

SemaphoreHandle_t Mutex = NULL;

int buttonCount = 0;

std::vector<String> files;
int file_count = 0;
int current_file_idx = 0;

static esp_netif_t *s_example_sta_netif = NULL;
static SemaphoreHandle_t s_semph_get_ip_addrs = NULL;
static int s_retry_num = 0;

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

static int lua_delay(lua_State *lua_state) {
    int a = luaL_checkinteger(lua_state, 1);
    delay(a);
    return 0;
}

/*static int lua_printText(lua_State *lua_state) {
    int textsize = luaL_checkinteger(lua_state, 1);
    int cursorx = luaL_checkinteger(lua_state, 2);
    int cursory = luaL_checkinteger(lua_state, 3);
    const char * text = luaL_checkstring(lua_state, 4);
    uint16_t color = luaL_checkinteger(lua_state, 5);

    matrixPanel.setTextSize(textsize);
    matrixPanel.setCursor(cursorx, cursory);
    matrixPanel.setTextColor(color);
    matrixPanel.print(text);
    return 0;
}*/

void scrollString(String str, int height, uint16_t bg, uint16_t fg) {
    matrixPanel.fillRect(0,height,matrixPanel.getWidth(),7,0x0000);
    matrixPanel.setTextWrap(false);
    matrixPanel.setTextSize(1);
    int charWidth = 6;
    int pxwidth = (str.length()*charWidth);
    for (int32_t x=charWidth; x>=-pxwidth+32; x--) {
        matrixPanel.fillRect(0,height,matrixPanel.getWidth(),7,0x0000);
        //matrixPanel.print(str);
        for (int chr=0; chr<str.length(); chr++) {
            matrixPanel.drawChar(x + charWidth*chr,height,str[chr],fg,bg,1);
        }
        delay(60);
    }
    delay(500);
    for (int chr=0; chr<str.length(); chr++) {
        matrixPanel.drawChar(charWidth*chr,height,str[chr],fg,bg,1);
    }
}

/*static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}*/


/*void show_menu(int selected) {
    matrixPanel.fillScreen(0x0000);
    for (int i=0; i<files.size(); i++) {
        //matrixPanel.setTextWrap(false);
        if (i != selected) {
            matrixPanel.setTextSize(1);
            for (int chr=0; chr<files[i].length(); chr++) {
                matrixPanel.drawChar(6*chr,i*8,files[i][chr],0xffff,0x0000,1);
            }
        }
    }

    scrollString(files[selected], selected*8, 0xffff, 0x0000);
}*/

/*void ButtonA_task(void *params)
{
    int pinNumber = 0;
    while (1) {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            lua_pop(lua_state, 1);
            vTaskDelete(lua_task_handle);
            buttonCount = (buttonCount + 1);
            //printf("hello\n");
            xTaskCreate(&run_lua_task,"lua task button A", 4096*8, &files[buttonCount], 0, &lua_task_handle);
        }
    }
    vTaskDelete(NULL);
}*/
/*
void ButtonB_task(void *params)
{
    int pinNumber, count = 0;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            //
        }
    }
    vTaskDelete(NULL);
}*/

/*void ButtonC_task(void *params)
{
    int pinNumber = 0;
    while (1) {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            lua_pop(lua_state, 1);
            vTaskDelete(lua_task_handle);
            buttonCount = (buttonCount - 1);
            //printf("hello\n");
            xTaskCreate(&run_lua_task,"lua task button C", 4096*8, &files[buttonCount], 0, &lua_task_handle);
        }
    }
    vTaskDelete(NULL);
}*/

/*void setup_interrupt_ISR(gpio_num_t pin, char *name, TaskFunction_t pvTaskCode) {
    gpio_pad_select_gpio(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_pulldown_en(pin);
    gpio_pullup_dis(pin);
    gpio_set_intr_type(pin, GPIO_INTR_POSEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    xTaskCreate(pvTaskCode, name, 2048, &buttonCount, 1, NULL);

    gpio_isr_handler_add(pin, gpio_interrupt_handler, (void *)pin);
}*/

static int lua_sin16(lua_State *lua_state) {
    int theta = luaL_checkinteger(lua_state, 1);

    lua_pushinteger(lua_state, fast_sin16(theta));

    return 1;
}

static int lua_sin8(lua_State *lua_state) {
    int theta = luaL_checkinteger(lua_state, 1)%256;

    lua_pushinteger(lua_state, fast_sin8(theta));

    return 1;
}

//https://github.com/espressif/esp-idf/issues/1807
static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    xSemaphoreTake( Mutex, portMAX_DELAY );

    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), 0x1);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, 0x1);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | I2C_MASTER_READ, 0x1);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t)0x0);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, (i2c_ack_type_t)0x1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive( Mutex );

    return ret;
}//*/

//https://github.com/espressif/esp-idf/issues/1807
static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
    xSemaphoreTake( Mutex, portMAX_DELAY );

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | I2C_MASTER_WRITE, 0x1);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, 0x1);
    // write the data
    i2c_master_write(cmd, data_wr, size, 0x1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive( Mutex );
    
    return ret;
}//*/

//https://github.com/espressif/esp-idf/issues/1807
static void i2c_master_init()
{
    Mutex = xSemaphoreCreateMutex();

    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = PIN_I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    /*printf("scanning the bus...\r\n\r\n");
    int devices_found = 0;
    for(int address = 1; address < 127; address++) {
        // create and execute the command link
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
            printf("-> found device with address 0x%02x\r\n", address);
            devices_found++;
        }
        fflush(stdout);
        i2c_cmd_link_delete(cmd);
    }
    if(devices_found == 0) printf("\r\n-> no devices found\r\n");*/
}//*/

// http://ww1.microchip.com/downloads/en/DeviceDoc/MCP7940N-Battery-Backed-I2C-RTCC-with-SRAM-20005010G.pdf
int getSeconds() {
    uint8_t rx_data[2];
    ESP_ERROR_CHECK(i2c_master_read_slave_reg(I2C_NUM_0, 0x6F, 0x00, rx_data, 1));
    int seconds = (rx_data[0] & 0x0F) + (((rx_data[0] >> 4) & 0x07)*10);
    return seconds;
}

int getMinutes() {
    uint8_t rx_data[2];
    ESP_ERROR_CHECK(i2c_master_read_slave_reg(I2C_NUM_0, 0x6F, 0x01, rx_data, 1));
    return (rx_data[0] & 0x0F) + (((rx_data[0] >> 4) & 0x07)*10);
}

int getHours() {
    uint8_t rx_data[2];
    ESP_ERROR_CHECK(i2c_master_read_slave_reg(I2C_NUM_0, 0x6F, 0x02, rx_data, 1));
    return (rx_data[0] & 0x0F) + (((rx_data[0] >> 0x04) & 1)*10);
}

static int lua_getSeconds(lua_State *lua_state) {
    lua_pushinteger(lua_state, getSeconds());
    return 1;
}

static int lua_getMinutes(lua_State *lua_state) {
    lua_pushinteger(lua_state, getMinutes());
    return 1;
}

static int lua_getHours(lua_State *lua_state) {
    lua_pushinteger(lua_state, getHours());
    return 1;
}

static int lua_getRandom(lua_State *lua_state) {
    lua_pushinteger(lua_state, esp_random());
    return 1;
}

// call a function `run' defined in Lua
double lua_run(lua_State *lua_state) {
    int z;

    // push functions and arguments
    lua_getglobal(lua_state, "run");  // function to be called

    // do the call (0 arguments, 1 result)
    if (lua_pcall(lua_state, 0, 1, 0) != 0) {
        ESP_LOGE(TAG, "error running function `run': %s", lua_tostring(lua_state, -1));
    }

    // retrieve result
    if (!lua_isnumber(lua_state, -1)) {
        ESP_LOGE(TAG, "function `run' must return a number");
    }
    z = lua_tointeger(lua_state, -1);
    lua_pop(lua_state, 1);  // pop returned value
    return z;
}

static int lua_setInterval(lua_State *lua_state) {
    int interval = luaL_checkinteger(lua_state, 1);
    while (true) {
        lua_run(lua_state);
        matrixPanel.drawBuffer();
        vTaskDelay(interval / portTICK_PERIOD_MS);
    }
    return 1;
}

void list_files() {
    // open the directory
    DIR* dir = opendir(MOUNT_POINT"/");
    struct dirent* de;

    // iterate over all files in the directory
    while (de = readdir(dir)) {
        if (!de) {
            break;
        }

        // extract files with .lua extention
        String filename = String(de->d_name);
        String extention = filename.substring(1 + filename.lastIndexOf("."));
        if (extention == "lua") {
            String base = filename.substring(0, filename.lastIndexOf("."));
            files.push_back(base); //add to the list of filenames
            file_count++;
        }
    }
}

// initialize the i2c acellerometer
int acellero_init() {
    uint8_t whoami[1] = { 0 };
    i2c_master_read_slave_reg(I2C_NUM_0, 0x1D,  0b00001111, whoami, 1);
    if (whoami[0] != 65) {
        return 0;
    } else {
        uint8_t CTRL1[1] = { 0b00011111 };
        i2c_master_write_slave_reg(I2C_NUM_0, 0x1D, 0b00100000, CTRL1, 1);
        return 1;
    }  
}

float get_acellero_x() {
    uint8_t data[2] = { 0 };
    i2c_master_read_slave_reg(I2C_NUM_0, 0x1D,  0b00101000, data, 2);
    int intValue = (int16_t)(data[1] << 8) + data[0];
    return intValue/16384.0f;
}

float get_acellero_y() {
    uint8_t data[2] = { 0 };
    i2c_master_read_slave_reg(I2C_NUM_0, 0x1D,  0b00101010, data, 2);
    int intValue = (int16_t)(data[1] << 8) + data[0];
    return -intValue/16384.0f;
}

float get_acellero_z() {
    uint8_t data[2] = { 0 };
    i2c_master_read_slave_reg(I2C_NUM_0, 0x1D,  0b00101100, data, 2);
    int intValue = (int16_t)(data[1] << 8) + data[0];
    return intValue/16384.0f;
}

static int lua_getAcceleration_x(lua_State *lua_state) {
    float acelleration_x = get_acellero_x();
    lua_pushnumber(lua_state, acelleration_x);
    return 1;
}

static int lua_getAcceleration_y(lua_State *lua_state) {
    float acelleration_y = get_acellero_y();
    lua_pushnumber(lua_state, acelleration_y);
    return 1;
}

static int lua_getAcceleration_z(lua_State *lua_state) {
    float acelleration_z = get_acellero_z();
    lua_pushnumber(lua_state, acelleration_z);
    return 1;
}//*/


void run_lua_task(void * param) {
    int script_id = *(int *)param;
    String script = files[script_id];
    String file = (MOUNT_POINT"/" + script + ".lua");

    printf("file name: %i\r\n", (script_id));
    fflush(stdout);

    // install libraries
    lua_state = luaL_newstate();
    luaL_requiref( lua_state, "math", luaopen_math, 1 );
    luaL_requiref( lua_state, "table", luaopen_table, 1 );
    luaL_requiref( lua_state, "base", luaopen_base, 1 );
    luaL_requiref( lua_state, "string", luaopen_string, 1 );
    luaL_requiref( lua_state, "matrix", lua_matrix_libs.luaopen_matrix_lib, 1 );
    luaL_requiref( lua_state, "physics", lua_box2d_libs.luaopen_box2d_lib, 1 );
    lua_pop( lua_state, 1 );

    // install functions
    //lua_register(lua_state, "printText", (const lua_CFunction) &lua_printText);
    lua_register(lua_state, "sin8", (const lua_CFunction) &lua_sin8);
    lua_register(lua_state, "sin16", (const lua_CFunction) &lua_sin16);
    lua_register(lua_state, "getSeconds", (const lua_CFunction) &lua_getSeconds);
    lua_register(lua_state, "getMinutes", (const lua_CFunction) &lua_getMinutes);
    lua_register(lua_state, "getHours", (const lua_CFunction) &lua_getHours);
    lua_register(lua_state, "delay", (const lua_CFunction) &lua_delay);
    lua_register(lua_state, "random", (const lua_CFunction) &lua_getRandom);
    lua_register(lua_state, "setInterval", (const lua_CFunction) &lua_setInterval);
    lua_register(lua_state, "getAccX", (const lua_CFunction) &lua_getAcceleration_x);
    lua_register(lua_state, "getAccY", (const lua_CFunction) &lua_getAcceleration_y);
    lua_register(lua_state, "getAccZ", (const lua_CFunction) &lua_getAcceleration_z);

    int error = luaL_dofile(lua_state, file.c_str());
    String error_string = lua_tostring(lua_state, -1);

    if (error != 0) {
        printf("LUA ERROR: %s\r\n", error_string);
        scrollString(error_string, 0, 0x0000, 0xffff);
        lua_pop(lua_state, 1);
    }

    vTaskDelete(NULL);
}

bool example_is_our_netif(const char *prefix, esp_netif_t *netif) {
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

static void example_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    s_retry_num = 0;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!example_is_our_netif(EXAMPLE_NETIF_DESC_STA, event->esp_netif)) {
        return;
    }
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    if (s_semph_get_ip_addrs) {
        xSemaphoreGive(s_semph_get_ip_addrs);
    } else {
        ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));
    }
}

static void example_handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    s_retry_num++;
    if (s_retry_num > CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY) {
        ESP_LOGI(TAG, "WiFi Connect failed %d times, stop reconnect.", s_retry_num);
        /* let example_wifi_sta_do_connect() return */
        if (s_semph_get_ip_addrs) {
            xSemaphoreGive(s_semph_get_ip_addrs);
        }
        return;
    }
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return;
    }
    ESP_ERROR_CHECK(err);
}

static void example_handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "Connected!");
}

void example_wifi_start(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    esp_netif_config.if_desc = EXAMPLE_NETIF_DESC_STA;
    esp_netif_config.route_prio = 128;
    s_example_sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t example_wifi_sta_do_disconnect(void) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect));

    if (s_semph_get_ip_addrs) {
        vSemaphoreDelete(s_semph_get_ip_addrs);
    }

    return esp_wifi_disconnect();
}

void example_wifi_stop(void) {
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(s_example_sta_netif));
    esp_netif_destroy(s_example_sta_netif);
    s_example_sta_netif = NULL;
}

void example_wifi_shutdown(void) {
    example_wifi_sta_do_disconnect();
    example_wifi_stop();
}

esp_err_t example_wifi_sta_do_connect(wifi_config_t wifi_config, bool wait) {
    if (wait) {
        s_semph_get_ip_addrs = xSemaphoreCreateBinary();
        if (s_semph_get_ip_addrs == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_retry_num = 0;
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &example_handler_on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect, s_example_sta_netif));

    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
        return ret;
    }
    if (wait) {
        ESP_LOGI(TAG, "Waiting for IP(s)");

        xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);

        if (s_retry_num > CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_err_t example_wifi_connect(void) {
    ESP_LOGI(TAG, "Start example_connect.");
    example_wifi_start();
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);

    return example_wifi_sta_do_connect(wifi_config, true);
}

void example_print_all_netif_ips(const char *prefix) {
    // iterate over active interfaces, and print out IPs of "our" netifs
    esp_netif_t *netif = NULL;
    for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) {
        netif = esp_netif_next(netif);
        if (example_is_our_netif(prefix, netif)) {
            ESP_LOGI(TAG, "Connected to %s", esp_netif_get_desc(netif));
            esp_netif_ip_info_t ip;
            ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip));

            ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&ip.ip));
        }
    }
}

esp_err_t example_connect(void) {
    if (example_wifi_connect() != ESP_OK) {
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&example_wifi_shutdown));

    example_print_all_netif_ips(EXAMPLE_NETIF_DESC_STA);

    return ESP_OK;
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

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(ARTNET_PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(ARTNET_PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));

        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }

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
                //if (source_addr.ss_family == PF_INET) {
                    //inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                    /*for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s\n", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }*/
                //} else if (source_addr.ss_family == PF_INET6) {
                //    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                //}

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
                        printf("poll\r\n");
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
    i2c_master_init();

    acellero_init();

    // Initialize rtc
    if (getSeconds() == 0) {
        uint8_t rtc_config[1] = {0x80};
        ESP_ERROR_CHECK(i2c_master_write_slave_reg(I2C_NUM_0, 0x6F, 0x00, rtc_config, 1));
    }

    init_fat();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    
    setCpuFrequencyMhz(240);

    /*lua_state = luaL_newstate();
    luaL_requiref( lua_state, "math", luaopen_math, 1 );
    luaL_requiref( lua_state, "table", luaopen_table, 1 );
    luaL_requiref( lua_state, "base", luaopen_base, 1 );
    luaL_requiref( lua_state, "string", luaopen_string, 1 );
    luaL_requiref( lua_state, "matrix", lua_matrix_libs.luaopen_matrix_lib, 1 );
    luaL_requiref( lua_state, "physics", lua_box2d_libs.luaopen_box2d_lib, 1 );
    lua_pop( lua_state, 1 );

    // install functions
    //lua_register(lua_state, "printText", (const lua_CFunction) &lua_printText);
    lua_register(lua_state, "sin8", (const lua_CFunction) &lua_sin8);
    lua_register(lua_state, "sin16", (const lua_CFunction) &lua_sin16);
    lua_register(lua_state, "getSeconds", (const lua_CFunction) &lua_getSeconds);
    lua_register(lua_state, "getMinutes", (const lua_CFunction) &lua_getMinutes);
    lua_register(lua_state, "getHours", (const lua_CFunction) &lua_getHours);
    lua_register(lua_state, "delay", (const lua_CFunction) &lua_delay);
    lua_register(lua_state, "random", (const lua_CFunction) &lua_getRandom);
    lua_register(lua_state, "setInterval", (const lua_CFunction) &lua_setInterval);
    lua_register(lua_state, "getAccX", (const lua_CFunction) &lua_getAcceleration_x);
    lua_register(lua_state, "getAccY", (const lua_CFunction) &lua_getAcceleration_y);
    lua_register(lua_state, "getAccZ", (const lua_CFunction) &lua_getAcceleration_z);//*/

    list_files();

    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);

    //current_file_idx = 0;
    //xTaskCreate(&run_lua_task,"lua task", 4096*8, &current_file_idx, 0, &lua_task_handle);
    
    // cycle through all scripts
    /*while (true) {
        for (int i=0; i<file_count; i++) {
            current_file_idx = i;
            printf("found file: %s.lua\r\n", files[current_file_idx]);

            xTaskCreate(&run_lua_task,"lua task", 4096*20, &current_file_idx, 0, &lua_task_handle);
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            matrixPanel.fillScreen(0x0000);
            matrixPanel.drawBuffer();
            esp_restart();
        }
    }//*/

    //matrixPanel.drawPixel(16,16,20,0,255);

    /*String script = files[0];
    String file = (MOUNT_POINT"/" + script + ".lua");
    printf("file: %s", file.c_str());
    fflush(stdout);

    //String script = ""
    //"";

    int error = luaL_dofile(lua_state, file.c_str());

    if (error != 0) {
        printf("LUA ERROR: %s\r\n", lua_tostring(lua_state, -1));
        matrixPanel.fillScreen(0x0000);
        //scrollString(error_string, 0, 0x0000, 0xffff);
        lua_pop(lua_state, 1);
    }//*/

    //buttonCount = 0;
    //gpio_install_isr_service(0);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_A, "ButtonA_task", ButtonA_task);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_B, "ButtonB_task", ButtonB_task);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_C, "ButtonC_task", ButtonC_task);

    // create task where the lua script runs in
    

    //current_file_idx = 1;
    //xTaskCreate(&run_lua_task,"lua task", 4096*8, &current_file_idx, 0, &lua_task_handle);
}

void loop() {
    matrixPanel.drawBuffer();
    delay(100);
}
