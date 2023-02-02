#ifndef MATRIX_PANEL_H
#define MATRIX_PANEL_H

#include <math.h>
#include "Adafruit_GFX.h"

#include "pinmap.h"
#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
#include "ESP32-VirtualMatrixPanel-I2S-DMA.h"

class MatrixPanel : public Adafruit_GFX
{
private:
    MatrixPanel_I2S_DMA *dma_display;
    VirtualMatrixPanel  *OneEightMatrixDisplay;

    int pixel_mapper(int in_x, int in_y, int *out_x, int *out_y);

public:
    MatrixPanel(/* args */);
    ~MatrixPanel();

    void drawPixel(int16_t x, int16_t y, uint8_t red, uint8_t grn, uint8_t blu);
    void drawLine(int x0, int y0, int x1, int y1, uint8_t red, uint8_t grn, uint8_t blu);
    void drawLineWu(int x0, int y0, int x1, int y1, uint8_t red, uint8_t grn, uint8_t blu);
    void fillPoly(float *px, float *py, uint n, uint8_t r, uint8_t g, uint8_t b);
    
    void drawPixel(int16_t x, int16_t y, uint16_t color) override;
    void fillScreen(uint16_t color) override;
};

#endif
