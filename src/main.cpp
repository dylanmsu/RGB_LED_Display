#include <Arduino.h>
#include <driver/sdmmc_defs.h>
#include <driver/sdmmc_types.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include <vector>
#include <esp_vfs_fat.h>
#include <nvs_flash.h>
#include <esp_vfs.h>
#include <WiFi.h>
#include <time.h>

#include "pinmap.h"
#include "MatrixPanel.h"
#include "Graphics3D.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#define PIN_NUM_MISO  (gpio_num_t)12
#define PIN_NUM_MOSI  (gpio_num_t)13
#define PIN_NUM_CLK   (gpio_num_t)14
#define PIN_NUM_CS    (gpio_num_t)20

static const char *TAG = "example";

lua_State *lua_state;
MatrixPanel matrixPanel;
Graphics3D graphics3D(&matrixPanel);

// Mount path for the partition
static sdmmc_card_t *mount_card = NULL;
#define MOUNT_POINT 	"/sdcard"

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

// Variable to save current epoch time
unsigned long epochTime;

// Function that gets current epoch time
unsigned long getTime() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return(0);
    }
    time(&now);
    return now;
}

float clamp_value(float num, float mini, float maxi){
	return min(max(maxi,num),mini);
}

// Initialize WiFi
void initWiFi(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
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
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
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
    slot_config.gpio_cs = PIN_NUM_CS;
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

static int lua_drawPixel(lua_State *lua_state) {
    int x = luaL_checkinteger(lua_state, 1);
    int y = luaL_checkinteger(lua_state, 2);
    int red = luaL_checkinteger(lua_state, 3);
    int grn = luaL_checkinteger(lua_state, 4);
    int blu = luaL_checkinteger(lua_state, 5);
    matrixPanel.drawPixel(x, y, red, grn, blu);
    return 0;
}

static int lua_drawPixelHSV(lua_State *lua_state) {
    int x = luaL_checkinteger(lua_state, 1);
    int y = luaL_checkinteger(lua_state, 2);
    int hue = luaL_checkinteger(lua_state, 3);
    int sat = luaL_checkinteger(lua_state, 4);
    int val = luaL_checkinteger(lua_state, 5);
    matrixPanel.drawPixelHSV(x, y, hue, sat, val);
    return 0;
}

static int lua_drawLine(lua_State *lua_state) {
    int x0 = luaL_checkinteger(lua_state, 1);
    int y0 = luaL_checkinteger(lua_state, 2);
    int x1 = luaL_checkinteger(lua_state, 3);
    int y1 = luaL_checkinteger(lua_state, 4);
    int red = luaL_checkinteger(lua_state, 5);
    int grn = luaL_checkinteger(lua_state, 6);
    int blu = luaL_checkinteger(lua_state, 7);
    matrixPanel.drawLine(x0, y0, x1, y1, red, grn, blu);
    return 0;
}

static int lua_fillQuat(lua_State *lua_state) {
    double px[4] = {
        luaL_checknumber(lua_state, 1),
        luaL_checknumber(lua_state, 3),
        luaL_checknumber(lua_state, 5),
        luaL_checknumber(lua_state, 7)
    };
    double py[4] = {
        luaL_checknumber(lua_state, 2),
        luaL_checknumber(lua_state, 4),
        luaL_checknumber(lua_state, 6),
        luaL_checknumber(lua_state, 8)
    };

    int red = luaL_checkinteger(lua_state, 9);
    int grn = luaL_checkinteger(lua_state, 10);
    int blu = luaL_checkinteger(lua_state, 11);
    
    matrixPanel.fillQuat((float *)px, (float *)py, red, grn, blu);
    return 0;
}

static int lua_delay(lua_State *lua_state) {
    int a = luaL_checkinteger(lua_state, 1);
    delay(a);
    return 0;
}

static int lua_clearScreen(lua_State *lua_state) {
    matrixPanel.fillScreen(0x0000);
    return 0;
}

/*static int lua_getEpoch(lua_State *lua_state) {
    return getTime();
}*/

static int lua_syncTime(lua_State *lua_state) {
    const char* ssid = luaL_checkstring(lua_state, 1);
    const char* password = luaL_checkstring(lua_state, 2);

    initWiFi(ssid, password);
    configTime(0, 0, ntpServer);

    //TODO: implement rtc and set correct time

    return getTime();
}

//push3dVertex(x, y, z)
static int lua_push3dVertex(lua_State *lua_state) {
    graphics3D.pushVertex(
        (float)luaL_checknumber(lua_state, 1),
        (float)luaL_checknumber(lua_state, 2),
        (float)luaL_checknumber(lua_state, 3)
    );
    return 0;
}

//push3dQuat(v1, v2, v3, v4, r, g, b)
static int lua_push3dQuat(lua_State *lua_state) {
    int p1 = luaL_checkinteger(lua_state, 1);
    int p2 = luaL_checkinteger(lua_state, 2);
    int p3 = luaL_checkinteger(lua_state, 3);
    int p4 = luaL_checkinteger(lua_state, 4);

    uint8_t r = luaL_checknumber(lua_state, 5);
    uint8_t g = luaL_checknumber(lua_state, 6);
    uint8_t b = luaL_checknumber(lua_state, 7);
    graphics3D.pushQuat(p1, p2, p3, p4, r, g, b);
    return 0;
}

static int lua_set3dRotation(lua_State *lua_state) {
    float x = luaL_checknumber(lua_state, 1);
    float y = luaL_checknumber(lua_state, 2);
    float z = luaL_checknumber(lua_state, 3);
    graphics3D.setRotation(x, y, z);
    return 0;
}

static int lua_draw3dsolid(lua_State *lua_state) {
    graphics3D.drawMesh();
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

void scrollString(String str) {
    int yoff = 1;
    matrixPanel.fillScreen(0x0000);
    matrixPanel.setTextWrap(false);  // we don't wrap text so it scrolls nicely
    matrixPanel.setTextSize(1);
    int charWidth = 9; // textsize 2 @todo auto calculate charwidth from font
    int pxwidth = (str.length()*charWidth); // @todo get actual string pixel length, add support to gfx if needed
    for (int32_t x=charWidth; x>=-pxwidth; x--) {
        matrixPanel.setCursor(x,yoff);
        // Serial.println((String)x);
        // display.print("ABCDEFGHIJKLMNONPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_0123456789");
        matrixPanel.print(str);
        // delay(ANIMSPEED/2);
        delay(60);
        matrixPanel.fillScreen(0x0000);
    }
    Serial.println("done");
}

void setup() {
    init_fat();
    
    setCpuFrequencyMhz(240);

    Serial.begin(115200);

    lua_state = luaL_newstate();

    // include math library in the lua environment
    luaL_requiref( lua_state, "math", luaopen_math, 1 );
    lua_pop( lua_state, 1 );

    // self implemented methods
    lua_register(lua_state, "drawPixel", (const lua_CFunction) &lua_drawPixel);
    lua_register(lua_state, "drawPixelHSV", (const lua_CFunction) &lua_drawPixelHSV);
    lua_register(lua_state, "drawLine", (const lua_CFunction) &lua_drawLine);
    lua_register(lua_state, "fillQuat", (const lua_CFunction) &lua_fillQuat);
    lua_register(lua_state, "clearScreen", (const lua_CFunction) &lua_clearScreen);

    // adafruit gfx
    lua_register(lua_state, "printText", (const lua_CFunction) &lua_printText);

    // miscellanious methods
    //lua_register(lua_state, "getEpoch", (const lua_CFunction) &lua_getEpoch);
    lua_register(lua_state, "syncTime", (const lua_CFunction) &lua_syncTime);
    lua_register(lua_state, "delay", (const lua_CFunction) &lua_delay);

    // 3d functions
    lua_register(lua_state, "push3dVertex", (const lua_CFunction) &lua_push3dVertex);
    lua_register(lua_state, "push3dQuat", (const lua_CFunction) &lua_push3dQuat);
    lua_register(lua_state, "set3dRotation", (const lua_CFunction) &lua_set3dRotation);
    lua_register(lua_state, "draw3dsolid", (const lua_CFunction) &lua_draw3dsolid);

    // run the lua script from the sd card
    bool error = luaL_dofile(lua_state, MOUNT_POINT"/script.lua");
    if (error) {
        Serial.println("LUA ERROR: " + String(lua_tostring(lua_state, -1)));
        scrollString(String(lua_tostring(lua_state, -1)));
        lua_pop(lua_state, 1);
    }//*/
}

/*void wireCube() {
    //you can define your own shape here if you want
    //(x, y, z)
    double verts[8*3] = {
        -1,-1,-1, //0           1---------5         ^ +z
        -1,-1, 1, //1          /|        /|         |
        -1, 1,-1, //2         3-|-------7 |         |
        -1, 1, 1, //3         | |       | |         |
        1,-1,-1, //4          | |       | |         o--------> +x
        1,-1, 1, //5          | 0---------4
        1, 1,-1, //6          |/        |/
        1, 1, 1  //7          2---------6
    };

    uint16_t edges[12*2] = {
        0,1,  1,3,  3,2,  2,0,
        0,4,  1,5,  2,6,  3,7,
        4,5,  5,7,  7,6,  6,4,
    };

    // create smooth color transitions
    int amplitude = 8;
    int red = sin(freq*k + rad/3*1) * amplitude + amplitude-1;
    int grn = sin(freq*k + rad/3*2) * amplitude + amplitude-1;
    int blu = sin(freq*k + rad/3*3) * amplitude + amplitude-1;

    // draw complete mesh with rotated vertices and color
    int edgeSize = sizeof(edges)/sizeof(edges[0]);
    drawMesh(rotate(verts,8,k,k/3,k/2), edges, edgeSize/2, red, grn, blu);

    // sleep and then clear canvas
    delay(10);
    matrix.clearScreen();
    k+=0.05;//
}*/

void loop() {

}
