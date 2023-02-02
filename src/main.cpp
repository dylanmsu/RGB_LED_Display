#include <Arduino.h>
#include <driver/sdmmc_defs.h>
#include <driver/sdmmc_types.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include <esp_vfs_fat.h>
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "pinmap.h"
#include "MatrixPanel.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#define BOARD_IO_SPI3_CS (gpio_num_t)34

#define BOARD_SDCARD_SDIO_CMD (gpio_num_t)35
#define BOARD_SDCARD_SDIO_CLK (gpio_num_t)36
#define BOARD_SDCARD_SDIO_DATA0 (gpio_num_t)37

#define BOARD_SDCARD_SDIO_CLK_PIN BOARD_SDCARD_SDIO_CLK
#define BOARD_SDCARD_SDIO_CMD_PIN BOARD_SDCARD_SDIO_CMD
#define BOARD_SDCARD_SDIO_DO_PIN BOARD_SDCARD_SDIO_DATA0

#define PIN_NUM_MISO  CONFIG_EXAMPLE_PIN_MISO
#define PIN_NUM_MOSI  CONFIG_EXAMPLE_PIN_MOSI
#define PIN_NUM_CLK   CONFIG_EXAMPLE_PIN_CLK
#define PIN_NUM_CS    CONFIG_EXAMPLE_PIN_CS

static const char *TAG = "example";

lua_State *lua_state;
MatrixPanel matrixPanel;

// Mount path for the partition
static sdmmc_card_t *mount_card = NULL;
#define MOUNT_POINT 	"/sdcard"

static int lua_drawPixel(lua_State *lua_state) {
    int x = luaL_checkinteger(lua_state, 1);
    int y = luaL_checkinteger(lua_state, 2);
    int red = luaL_checkinteger(lua_state, 3);
    int grn = luaL_checkinteger(lua_state, 4);
    int blu = luaL_checkinteger(lua_state, 5);
    matrixPanel.drawPixel(x, y, red, grn, blu);
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
    
    matrixPanel.fillPoly((float *)px, (float *)py, 4, red, grn, blu);
    return 0;
}

static int lua_fillTriangle(lua_State *lua_state) {
    double px[4] = {
        luaL_checknumber(lua_state, 1),
        luaL_checknumber(lua_state, 3),
        luaL_checknumber(lua_state, 5)
    };
    double py[4] = {
        luaL_checknumber(lua_state, 2),
        luaL_checknumber(lua_state, 4),
        luaL_checknumber(lua_state, 6)
    };

    int red = luaL_checkinteger(lua_state, 7);
    int grn = luaL_checkinteger(lua_state, 8);
    int blu = luaL_checkinteger(lua_state, 9);
    
    matrixPanel.fillPoly((float *)px, (float *)py, 3, red, grn, blu);
    return 0;
}

static int lua_delay(lua_State *lua_state) {
    int a = luaL_checkinteger(lua_state, 1);
    delay(a);
    return 0;
}

void setup() {
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 13,
        .miso_io_num = 12,
        .sclk_io_num = 14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)20;
    slot_config.host_id = (spi_host_device_t)host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

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

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    
    setCpuFrequencyMhz(240);

    Serial.begin(115200);

    lua_state = luaL_newstate();
    luaopen_base(lua_state);
    luaopen_table(lua_state);
    luaopen_string(lua_state);
    luaopen_math(lua_state);

    lua_register(lua_state, "drawPixel", (const lua_CFunction) &lua_drawPixel);
    lua_register(lua_state, "drawLine", (const lua_CFunction) &lua_drawLine);
    lua_register(lua_state, "fillTriangle", (const lua_CFunction) &lua_fillTriangle);
    lua_register(lua_state, "fillQuat", (const lua_CFunction) &lua_fillQuat);
    lua_register(lua_state, "delay", (const lua_CFunction) &lua_delay);

    matrixPanel.drawChar(10,10,'w',0xffff, 0x0000, 1);

    bool error = luaL_dofile(lua_state, MOUNT_POINT"/script.lua");
    if (error) {
        Serial.println("LUA ERROR: " + String(lua_tostring(lua_state, -1)));
        lua_pop(lua_state, 1);
    }
}

/*void solidCube() {
    float rad = 2*pi;	
	
	float r = .5;
	float g = .8;
	float b = .4;

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

    int faces[6*4] = {
        1,3,7,5,  2,0,4,6,
        1,0,2,3,  7,6,4,5,
        3,2,6,7,  5,4,0,1,
    };
	
	//int num = 5*5;
	//double verts[num*3] = {0};
	//int faces[num*4] = {0};
	//torus(verts,faces);
		
    drawSolid(rotate(verts,8,k,k/3,k/2),faces,6,r,g,b);
    //draw(&Sbuffer);

    delay(1000/30);
    matrix.clearScreen();
    k+=0.05;
}

void wireCube() {
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
