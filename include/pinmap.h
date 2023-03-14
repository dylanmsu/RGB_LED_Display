#define V2

#ifdef V1

    // HUB75 connections
    #define R1_PIN          25
    #define G1_PIN          26
    #define B1_PIN          27
    #define R2_PIN          15
    #define G2_PIN          7
    #define B2_PIN          2

    #define A_PIN           32
    #define B_PIN           33
    #define C_PIN           5
    #define D_PIN           -1
    #define E_PIN           -1

    #define LAT_PIN         4
    #define OE_PIN          19
    #define CLK_PIN         8

    // SD_Card
    #define PIN_NUM_MISO    12
    #define PIN_NUM_MOSI    13
    #define PIN_NUM_CLK     14
    #define PIN_NUM_CS      20

        // I2C
    #define PIN_I2C_SDA     21
    #define PIN_I2C_SCL     22

    // IO
    #define BUTTON_A        39
    #define BUTTON_B        38
    #define BUTTON_C        37

#endif

#ifdef V2

    // HUB75 connections
    #define R1_PIN          40
    #define G1_PIN          39
    #define B1_PIN          4
    #define R2_PIN          5
    #define G2_PIN          1
    #define B2_PIN          2

    #define A_PIN           42
    #define B_PIN           41
    #define C_PIN           47
    #define D_PIN           -1
    #define E_PIN           -1

    #define LAT_PIN         14
    #define OE_PIN          13
    #define CLK_PIN         21

    // SD_Card
    #define PIN_NUM_MISO    37
    #define PIN_NUM_MOSI    35
    #define PIN_NUM_CLK     36
    #define PIN_NUM_CS      48

    // I2C
    #define PIN_I2C_SDA     7
    #define PIN_I2C_SCL     6

    // IO
    //#define BUTTON_A        39
    //#define BUTTON_B        38
    //#define BUTTON_C        37

#endif