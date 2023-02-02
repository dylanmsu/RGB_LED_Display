#include "MatrixPanel.h"

MatrixPanel::MatrixPanel(/* args */) : Adafruit_GFX(32, 32)
{
    HUB75_I2S_CFG::i2s_pins _pins = {
        R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, 
        A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, 
        LAT_PIN, OE_PIN, CLK_PIN
    };

    HUB75_I2S_CFG mxconfig(64, 16, 1, _pins);

    mxconfig.clkphase = false;
    mxconfig.driver   = HUB75_I2S_CFG::SHIFTREG;

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->begin();
    dma_display->setBrightness8(255);
    fillScreen(0x0000);

    OneEightMatrixDisplay = new VirtualMatrixPanel((*dma_display), 1, 1, 32, 32, true, false);
    OneEightMatrixDisplay->setPhysicalPanelScanRate(ONE_EIGHT_32);
}

MatrixPanel::~MatrixPanel() {
    
}

//the used panel has a weird scanning order
int MatrixPanel::pixel_mapper(int in_x, int in_y, int *out_x, int *out_y) {
    if (in_y < 8) {
        *out_x = in_x+32;
        *out_y = in_y;
        return 1;
    } else if (in_y >= 8 && in_y < 16) {
        *out_x = in_x;
        *out_y = in_y-8;
        return 1;
    } else if (in_y >= 16 && in_y < 24) {
        *out_x = in_x+32;
        *out_y = in_y-8;
        return 1;
    } else if (in_y >= 24 && in_y < 32) {
        *out_x = in_x;
        *out_y = in_y-16;
        return 1;
    } else {
        return 0;
    }
}

void MatrixPanel::drawPixel(int16_t x, int16_t y, uint8_t red, uint8_t grn, uint8_t blu) {
    int i,j = 0;
    pixel_mapper(x, y, &i, &j);
    dma_display->drawPixelRGB888(i, j, red, grn, blu);
}

void MatrixPanel::fillScreen(uint16_t color) {
    if (color != 0x0000) {
        dma_display->fillScreen(color);
    } else {
        dma_display->clearScreen();
    }
}

void MatrixPanel::drawPixel(int16_t x, int16_t y, uint16_t color) {
    uint8_t r = ((((color >> 11) & 0x1F) * 527) + 23) >> 6;
    uint8_t g = ((((color >> 5) & 0x3F) * 259) + 33) >> 6;
    uint8_t b = (((color & 0x1F) * 527) + 23) >> 6;
    drawPixel(x, y, r, g,  b);
};

void MatrixPanel::drawLine(int x0, int y0, int x1, int y1, uint8_t red, uint8_t grn, uint8_t blu) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2;

    while (drawPixel(x0, y0, red, grn, blu), x0 != x1 || y0 != y1) {
        int e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
}

void swapInt(int &a, int &b) {
    int temp = a;
    a = b;
    b = temp;
}

void MatrixPanel::drawLineWu(int x0, int y0, int x1, int y1, uint8_t red, uint8_t grn, uint8_t blu) {
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    
    if (steep) {
        swapInt(x0, y0);
        swapInt(x1, y1);
    }
    if (x0 > x1) {
        swapInt(x0, x1);
        swapInt(y0, y1);
    }
    
    int dx = x1 - x0;
    int dy = y1 - y0;
     
    float gradient = 0;
    if (dx == 0.0) {
        gradient = 1.0;
    } else {
        gradient = dy / dx;
    }

    // handle first endpoint
    float xend1 = floor(x0 + 0.5);
    float yend1 = y0 + gradient * (xend1 - x0);
    float xgap1 = 1 - ((x0 + 0.5) - floor(x0 + 0.5));
    float xpxl1 = xend1; // this will be used in the main loop
    float ypxl1 = floor(yend1);
    if (steep) {
        drawPixel(ypxl1,   xpxl1, (1-(yend1 - floor(yend1))) * xgap1, 0, 0);
        drawPixel(ypxl1+1, xpxl1,  (yend1 - floor(yend1)) * xgap1, 0, 0);
    } else {
        drawPixel(xpxl1, ypxl1  , (1-(yend1 - floor(yend1))) * xgap1, 0, 0);
        drawPixel(xpxl1, ypxl1+1,  (yend1 - floor(yend1)) * xgap1, 0, 0);
    }
    float intery = yend1 + gradient; // first y-intersection for the main loop
    
    // handle second endpoint
    float xend2 = floor(x1 + 0.5);
    float yend2 = y1 + gradient * (xend2 - x1);
    float xgap2 = ((x1 + 0.5) - floor(x1 + 0.5));
    float xpxl2 = xend2; //this will be used in the main loop
    float ypxl2 = floor(yend2);
    if (steep) {
        drawPixel(ypxl2  , xpxl2, (1-(yend2 - floor(yend2))) * xgap2, 0, 0);
        drawPixel(ypxl2+1, xpxl2,  (yend2 - floor(yend2)) * xgap2, 0, 0);
    } else {
        drawPixel(xpxl2, ypxl2,  (1-(yend2 - floor(yend2))) * xgap2, 0, 0);
        drawPixel(xpxl2, ypxl2+1, (yend2 - floor(yend2)) * xgap2, 0, 0);
    }
    
    // main loop
    if (steep) {
        for (int x=(xpxl1 + 1); x<(xpxl2 - 1); x++){
            drawPixel(floor(intery)  , x, (1-(intery - floor(intery))), 0, 0);
            drawPixel(floor(intery)+1, x,  (intery - floor(intery)), 0, 0);
            intery = intery + gradient;
        }
    } else {
        for (int x=(xpxl1 + 1); x<(xpxl2 - 1); x++) {
            drawPixel(x, floor(intery),  (1-(intery - floor(intery))), 0, 0);
            drawPixel(x, floor(intery)+1, (intery - floor(intery)), 0, 0);
            intery = intery + gradient;
        }
    }
}

//creates a filled polygon with n = the amount of corners
void MatrixPanel::fillPoly(float *px, float *py, uint n, uint8_t r, uint8_t g, uint8_t b) 
{
    int IMG_TOP = py[0];
    int IMG_BOT = py[0];
    int IMG_LEF = px[0];
    int IMG_RIG = px[0];
	int MAX_POLY_CORNERS = n;
	int polyCorners = n;
	int nodes;
	int nodeX[MAX_POLY_CORNERS] = {};
	int pixelY;
	int i;
	int j;
	int swap;

    // Find maximum and minimum in all array elements.
    for (i = 1; i < MAX_POLY_CORNERS; i++) {
        if (py[i] > IMG_TOP)
            IMG_TOP = py[i];
        if (py[i] < IMG_BOT)
            IMG_BOT = py[i];
        if (px[i] > IMG_RIG)
            IMG_RIG = px[i];
        if (px[i] < IMG_LEF)
            IMG_LEF = px[i];
    }
	
	for (pixelY=IMG_TOP; pixelY<IMG_BOT; pixelY++) {
		nodes = 0; j = polyCorners-1;
		for (i=0; i<polyCorners; i++){
			if ((py[i]<pixelY && py[j]>=pixelY) || (py[j]<pixelY && py[i]>=pixelY)) {
				nodeX[nodes++] = (px[i]+(pixelY-py[i])/(py[j]-py[i])*(px[j]-px[i]));
			}
			j=i;
		}
		
		i=0;
		while(i<nodes-1){
			if (nodeX[i]>nodeX[i+1]) {
				swap=nodeX[i];
                nodeX[i]=nodeX[i+1]; 
                nodeX[i+1]=swap; 
                if (i) {
                    i--;
                }
            } else {
				i++;
			}
		}
		
		
        for (i=0; i<nodes; i+=2){
            int nodeXi = nodeX[i];
            int nodeXii = nodeX[i+1];
            if (nodeXi>=IMG_RIG) {
                break;
            }
            if (nodeXii>IMG_LEF) {
                if (nodeXi<IMG_LEF) {
                    nodeXi=IMG_LEF;
                }
                if (nodeXii>IMG_RIG) {
                    nodeXii = IMG_RIG;
                }
                for (int pixelX=nodeXi; pixelX<nodeXii; pixelX++) {
                    drawPixel(pixelX, pixelY, r, g, b);
                }
            }
	    }
    }
}