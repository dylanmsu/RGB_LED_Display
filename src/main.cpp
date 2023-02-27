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

#include "pinmap.h"
#include "MatrixPanel.h"
#include "Graphics3D.h"
#include "lua_libs.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

static const char *TAG = "example";

lua_State *lua_state;
MatrixPanel matrixPanel;
Graphics3D graphics3D(&matrixPanel);

Lua_libs lua_libs(lua_state, &matrixPanel, &graphics3D);

// Mount path for the partition
static sdmmc_card_t *mount_card = NULL;
#define MOUNT_POINT 	"/sdcard"

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

// Variable to save current epoch time
unsigned long epochTime;

xQueueHandle interputQueue;

TaskHandle_t lua_task_handle;

int buttonCount = 0;

std::vector<String> files;

// Function that gets current epoch time
/*unsigned long getTime() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return(0);
    }
    time(&now);
    return now;
}*/

float clamp_value(float num, float mini, float maxi){
	return min(max(maxi,num),mini);
}

// Initialize WiFi
/*void initWiFi(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}*/

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

/*static int lua_getEpoch(lua_State *lua_state) {
    return getTime();
}*/

/*static int lua_syncTime(lua_State *lua_state) {
    const char* ssid = luaL_checkstring(lua_state, 1);
    const char* password = luaL_checkstring(lua_state, 2);

    initWiFi(ssid, password);
    configTime(0, 0, ntpServer);

    //TODO: implement rtc and set correct time

    return getTime();
}*/

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
    matrixPanel.fillRect(0,height,32,7,0x0000);
    matrixPanel.setTextWrap(false);
    matrixPanel.setTextSize(1);
    int charWidth = 6;
    int pxwidth = (str.length()*charWidth);
    for (int32_t x=charWidth; x>=-pxwidth+32; x--) {
        matrixPanel.fillRect(0,height,32,7,0x0000);
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

/*void run_lua_task(void * param) {
    String script = *(String *)param;
    String file = ("/sdcard/" + script + ".lua");
    int error = luaL_dofile(lua_state, file.c_str());
    if (error) {
        switch (error)
        {
        case LUA_YIELD:
            matrixPanel.print("Yield\nerror");
            break;
        case LUA_ERRRUN:
            matrixPanel.print("errrun");
            break;
        case LUA_ERRSYNTAX:
            matrixPanel.print("errsyntax");
            break;
        case LUA_ERRMEM:
            matrixPanel.print("errmem");
            break;
        case LUA_ERRERR:
            matrixPanel.print("errerr");
            break;
        
        default:
            matrixPanel.print("unknown");
            break;
        }
        printf("LUA ERROR: %s\r\n", String(lua_tostring(lua_state, -1)));
        //scrollString(String(lua_tostring(lua_state, -1)));

        lua_pop(lua_state, 1);
    }
    vTaskDelete(NULL);
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

void setup() {
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
    BLEDevice::startAdvertising();*/

    init_fat();
    
    setCpuFrequencyMhz(240);

    lua_state = luaL_newstate();
    luaL_requiref( lua_state, "math", luaopen_math, 1 );
    luaL_requiref( lua_state, "matrix", lua_libs.luaopen_matrix_lib, 1 );
    lua_pop( lua_state, 1 );

    lua_register(lua_state, "printText", (const lua_CFunction) &lua_printText);

    //lua_register(lua_state, "getEpoch", (const lua_CFunction) &lua_getEpoch);
    //lua_register(lua_state, "syncTime", (const lua_CFunction) &lua_syncTime);
    lua_register(lua_state, "delay", (const lua_CFunction) &lua_delay);

    /*DIR* dir = opendir(MOUNT_POINT"/");
    struct dirent* de;
    while (de = readdir(dir)) {
        if (!de) {
            break;
        }

        String filename = String(de->d_name);
        String extention = filename.substring(1 + filename.lastIndexOf("."));
        if (extention == "lua") {
            String base = filename.substring(0, filename.lastIndexOf("."));
            files.push_back(base);
            //printf("found: %s\r\n", base);
        }
    }*/

    //buttonCount = 0;
    //gpio_install_isr_service(0);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_A, "ButtonA_task", ButtonA_task);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_B, "ButtonB_task", ButtonB_task);
    //setup_interrupt_ISR((gpio_num_t)BUTTON_C, "ButtonC_task", ButtonC_task);

    //xTaskCreate(&run_lua_task,"lua task", 4096*8, &files[0], 0, &lua_task_handle);
    //run_lua_task(&files[0]);

    try
    {
        String file = "/sdcard/cube.lua";
        int error = luaL_dofile(lua_state, file.c_str());
        printf("hello1\r\n");
        fflush(stdout);
    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        printf("error: %s\r\n", exc.what());
        fflush(stdout);
    }

    /*delay(5000);
    vTaskDelete(lua_task_handle);

    xTaskCreate(&run_lua_task,"lua task", 4096*8, &files[1], 0, &lua_task_handle);
    delay(5000);
    vTaskDelete(lua_task_handle);*/
}

void loop() {

}
