#include "MatrixPanel.h"

MatrixPanel::MatrixPanel(int width, int height) 
    : Adafruit_GFX(width, height)
    , _width(width)
    , _height(height)
{
    HUB75_I2S_CFG::i2s_pins _pins = {
        R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, 
        A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, 
        LAT_PIN, OE_PIN, CLK_PIN
    };

    HUB75_I2S_CFG mxconfig(2*_width, _height/2, 1, _pins);

    mxconfig.clkphase = false;
    mxconfig.driver   = HUB75_I2S_CFG::SHIFTREG;

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->begin();
    dma_display->setBrightness8(4);
    fillScreen(0x0000);

    OneEightMatrixDisplay = new VirtualMatrixPanel((*dma_display), 1, 1, _width, _height, true, false);
    OneEightMatrixDisplay->setPhysicalPanelScanRate(FOUR_SCAN_16PX_HIGH);
    //OneEightMatrixDisplay->setPhysicalPanelScanRate(ONE_EIGHT_16);
}

MatrixPanel::~MatrixPanel() {
    
}

//the used panel has a weird scanning order so we remap the pixels to have a continuous canvas
int MatrixPanel::pixel_mapper(int in_x, int in_y, int *out_x, int *out_y) {
    if (in_x < 0 || in_x > 31) {
        return 0;
    }

    if (in_y < 0 || in_y > 31) {
        return 0;
    }

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

void MatrixPanel::setBrightness(uint8_t brightness) {
    dma_display->setBrightness8(brightness);
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
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2;

    while (drawPixel(x0, y0, red, grn, blu), x0 != x1 || y0 != y1) {
        int e2 = err;
        if (e2 > -dx) { 
            err -= dy; 
            x0 += sx; 
        }
        if (e2 <  dy) {
            err += dx;
            y0 += sy;
        }
    }
}

void MatrixPanel::drawPixelHSV(int16_t x, int16_t y, float H, float S,float V){
    if(H>360 || H<0 || S>100 || S<0 || V>100 || V<0){
        //cout<<"The givem HSV values are not in valid range"<<endl;
        return;
    }
    float s = S/100;
    float v = V/100;
    float C = s*v;
    float X = C*(1-abs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }
    uint8_t R = (r+m)*255;
    uint8_t G = (g+m)*255;
    uint8_t B = (b+m)*255;

    drawPixel(x, y, R, G, B);
}

void MatrixPanel::fillQuat(float px[4], float py[4], uint8_t r, uint8_t g, uint8_t b){
	int IMG_TOP = min(min(min(py[0],py[1]),py[2]),py[3]);
	int IMG_BOT = max(max(max(py[0],py[1]),py[2]),py[3])+1;
	int IMG_LEF = min(min(min(px[0],px[1]),px[2]),px[3]);
	int IMG_RIG = max(max(max(px[0],px[1]),px[2]),px[3])+1;
	int MAX_POLY_CORNERS = 4;
	int polyCorners = 4;
	int nodes;
	int nodeX[MAX_POLY_CORNERS] = {};
	int pixelY;
	int i;
	int j;
	int swap;
	
	for (pixelY=IMG_TOP; pixelY<IMG_BOT; pixelY++){
		nodes = 0; j = polyCorners-1;
		for (i=0; i<polyCorners; i++){
			if ((py[i]<pixelY && py[j]>=pixelY) || (py[j]<pixelY && py[i]>=pixelY)){
				nodeX[nodes++] = (px[i]+(pixelY-py[i])/(py[j]-py[i])*(px[j]-px[i]));
			}
			j=i;
		}
		
		i=0;
		while(i<nodes-1){
			if (nodeX[i]>nodeX[i+1]){
				swap=nodeX[i]; nodeX[i]=nodeX[i+1]; nodeX[i+1]=swap; if (i) i--; }
			else{
				i++;
			}
		}
		
		
		for (i=0; i<nodes; i+=2){
			int nodeXi = nodeX[i];
			int nodeXii = nodeX[i+1];
			if (nodeXi>=IMG_RIG) break;
			if (nodeXii>IMG_LEF) {
				if (nodeXi<IMG_LEF) nodeXi=IMG_LEF;
				if (nodeXii>IMG_RIG) nodeXii = IMG_RIG;
				for (int pixelX=nodeXi;pixelX<nodeXii;pixelX++){
					drawPixel(pixelX, pixelY, r, g, b);
				}
			}
		}
	}
}

int MatrixPanel::getWidth() {
    return _width;
}

int MatrixPanel::getHeight() {
    return _height;
}