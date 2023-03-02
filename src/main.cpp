#include <Arduino.h>
#include <driver/sdmmc_defs.h>
#include <driver/sdmmc_types.h>
#include <driver/sdmmc_host.h>
#include <driver/gpio.h>
#include <sdmmc_cmd.h>
#include <vector>
#include <esp_vfs_fat.h>
#include <nvs_flash.h>
#include <esp_vfs.h>
#include <string>
//#include <WiFi.h>
//#include <time.h>
//#include <BLEDevice.h>
//#include <BLEUtils.h>
//#include <BLEServer.h>

#include "driver/i2c.h"
#include "freertos/semphr.h"

#include "pinmap.h"
#include "MatrixPanel.h"
#include "Graphics3D.h"
#include "lua_libs.h"
#include "lookup.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

static const char *TAG = "example";

lua_State *lua_state;
MatrixPanel matrixPanel(32,32);
Graphics3D graphics3D(&matrixPanel);

Lua_libs lua_libs(lua_state, &matrixPanel, &graphics3D);

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

static int lua_printText(lua_State *lua_state) {
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
}

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

void run_lua_task(void * param) {
    int script_id = *(int *)param;
    String script = files[script_id];
    String file = ("/sdcard/" + script + ".lua");

    printf("file name: %i\r\n", (script_id));
    fflush(stdout);

    int error = luaL_dofile(lua_state, file.c_str());
    String error_string = lua_tostring(lua_state, -1);

    if (error != 0) {
        printf("LUA ERROR: %s\r\n", error_string);
        scrollString(error_string, 0, 0x0000, 0xffff);
        lua_pop(lua_state, 1);
    }

    vTaskDelete(NULL);
}

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

static int lua_sin8(lua_State *lua_state) {
    int theta = luaL_checkinteger(lua_state, 1)%256;

    uint8_t y = 0;
    if (theta < 0) {
        y = -sine_table_8[-theta];
    } else {
        y = sine_table_8[theta];
    }

    lua_pushinteger(lua_state, y);

    return 1;
}

//https://github.com/FastLED/FastLED/blob/master/src/lib8tion/trig8.h
static int lua_sin16(lua_State *lua_state) {
    int theta = luaL_checkinteger(lua_state, 1);

    static const uint16_t base[] = { 0, 6393, 12539, 18204, 23170, 27245, 30273, 32137 };
    static const uint8_t slope[] = { 49, 48, 44, 38, 31, 23, 14, 4 };

    uint16_t offset = (theta & 0x3FFF) >> 3; // 0..2047

    if ( theta & 0x4000 ) {
        offset = 2047 - offset;
    }

    uint8_t section = offset / 256; // 0..7
    uint16_t b   = base[section];
    uint8_t  m   = slope[section];

    uint8_t secoffset8 = (uint8_t)(offset) / 2;

    uint16_t mx = m * secoffset8;
    int16_t  y  = mx + b;

    if ( theta & 0x8000 ) {
        y = -y;
    }

    lua_pushinteger(lua_state, y);

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
}

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
}

//https://github.com/espressif/esp-idf/issues/1807
static void i2c_master_init()
{
    Mutex = xSemaphoreCreateMutex();

    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    printf("scanning the bus...\r\n\r\n");
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
    if(devices_found == 0) printf("\r\n-> no devices found\r\n");
}

// http://ww1.microchip.com/downloads/en/DeviceDoc/MCP7940N-Battery-Backed-I2C-RTCC-with-SRAM-20005010G.pdf
int getSeconds() {
    uint8_t rx_data[2];
    uint8_t reg = 0x00;
    ESP_ERROR_CHECK(i2c_master_read_slave_reg(I2C_NUM_0, 0x6F, reg, rx_data, 1));
    return (rx_data[0] & 0b00001111) + (((rx_data[0] >> 4) & 0b00000111)*10);
}

int getMinutes() {
    uint8_t rx_data[2];
    uint8_t reg = 0x01;
    ESP_ERROR_CHECK(i2c_master_read_slave_reg(I2C_NUM_0, 0x6F, reg, rx_data, 1));
    return (rx_data[0] & 0b00001111) + (((rx_data[0] >> 4) & 0b00000111)*10);
}

int getHours() {
    uint8_t rx_data[2];
    uint8_t reg = 0x02;
    ESP_ERROR_CHECK(i2c_master_read_slave_reg(I2C_NUM_0, 0x6F, reg, rx_data, 1));
    return (rx_data[0] & 0b00001111) + (((rx_data[0] >> 4) & 0b00000001)*10);
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
    return 0;
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

void setup() {
    i2c_master_init();

	uint8_t data = 0x80; //=> 0 seconds bit 7 on for counting
    ESP_ERROR_CHECK(i2c_master_write_slave_reg(I2C_NUM_0, 0x6F, 0x00, &data, 1));

    /*BLEDevice::init("Long name works now");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE
                                        );

    pCharacteristic->setValue("Hello World says Neil");
    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();//*/

    init_fat();
    
    setCpuFrequencyMhz(240);

    // install libraries
    lua_state = luaL_newstate();
    luaL_requiref( lua_state, "math", luaopen_math, 1 );
    luaL_requiref( lua_state, "matrix", lua_libs.luaopen_matrix_lib, 1 );
    lua_pop( lua_state, 1 );

    // install functions
    lua_register(lua_state, "printText", (const lua_CFunction) &lua_printText);
    lua_register(lua_state, "sin8", (const lua_CFunction) &lua_sin8);
    lua_register(lua_state, "sin16", (const lua_CFunction) &lua_sin16);
    lua_register(lua_state, "getSeconds", (const lua_CFunction) &lua_getSeconds);
    lua_register(lua_state, "getMinutes", (const lua_CFunction) &lua_getMinutes);
    lua_register(lua_state, "getHours", (const lua_CFunction) &lua_getHours);
    lua_register(lua_state, "delay", (const lua_CFunction) &lua_delay);
    lua_register(lua_state, "random", (const lua_CFunction) &lua_getRandom);

    list_files();
    /*for (int i=0; i<file_count; i++) {
        printf("found file: %s.lua\r\n", files[i]);
        fflush(stdout);
    }*/

    //buttonCount = 0;
    //gpio_install_isr_service(0);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_A, "ButtonA_task", ButtonA_task);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_B, "ButtonB_task", ButtonB_task);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_C, "ButtonC_task", ButtonC_task);

    // create task where the lua script runs in
    current_file_idx = 0;
    xTaskCreate(&run_lua_task,"lua task", 4096*8, &current_file_idx, 0, &lua_task_handle);

    /*int error = luaL_dofile(lua_state, "/sdcard/clock.lua");
    if (error) {
        printf("LUA ERROR: %s\r\n", lua_tostring(lua_state, -1));
    }//*/
}

void loop() {

}
