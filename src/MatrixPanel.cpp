#include "MatrixPanel.h"

MatrixPanel::MatrixPanel(int width, int height) 
    : Adafruit_GFX(width, height)
    , _width(width)
    , _height(height)
{
    // buffer for the pixels to support opacity
    pixel_buffer = (uint8_t *) calloc(width*height*3, sizeof(uint8_t));

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
    dma_display->setBrightness8(4); //can be changed later by the lua script
    fillScreen(0x0000);

    //OneEightMatrixDisplay = new VirtualMatrixPanel((*dma_display), 1, 1, _width, _height, true, false);
    //OneEightMatrixDisplay->setPhysicalPanelScanRate(FOUR_SCAN_16PX_HIGH);
    //OneEightMatrixDisplay->setPhysicalPanelScanRate(ONE_EIGHT_16);
}

MatrixPanel::~MatrixPanel() {
    free(pixel_buffer);
}

//the used panel has a scanning order not implemented in the hub75 library, 
//so we remap the pixels to have a continuous canvas
int MatrixPanel::pixel_mapper(int in_x, int in_y, int *out_x, int *out_y) {
    if (in_x < 0 || in_x > 31) {
        return 1;
    }

    if (in_y < 0 || in_y > 31) {
        return 1;
    }

    if (in_y < 8) {
        *out_x = in_x+32;
        *out_y = in_y;
        return 0;
    } else if (in_y >= 8 && in_y < 16) {
        *out_x = in_x;
        *out_y = in_y-8;
        return 0;
    } else if (in_y >= 16 && in_y < 24) {
        *out_x = in_x+32;
        *out_y = in_y-8;
        return 0;
    } else if (in_y >= 24 && in_y < 32) {
        *out_x = in_x;
        *out_y = in_y-16;
        return 0;
    } else {
        return 1;
    }
}

void MatrixPanel::drawPixel(int16_t x, int16_t y, uint8_t red, uint8_t grn, uint8_t blu) {
    int i,j = 0;
    int err = pixel_mapper(x, y, &i, &j);
    if (!err) {
        int idx = i + _width*2*j;
        pixel_buffer[idx*3 + 0] = red;
        pixel_buffer[idx*3 + 1] = grn;
        pixel_buffer[idx*3 + 2] = blu;
        //dma_display->drawPixelRGB888(i, j, red, grn, blu);
    } else {
        // pixel is outside of range
    }
}

void MatrixPanel::drawPixelRGBA(int16_t x, int16_t y, uint8_t red, uint8_t grn, uint8_t blu, float alpha) {
    auto min = [](float a, float b) -> int {if (a>b) return b; else return a;};
    auto max = [](float a, float b) -> int {if (a<b) return b; else return a;};

    int i,j = 0;
    int err = pixel_mapper(x, y, &i, &j);
    if (!err) {
        // clamp alpha between 0 and 1
        //alpha = min(max(alpha, 0.0f), 1.0f);

        int idx = i + _width*2*j;
        // [new pixel] = [bottom pixel]*(1-alpha) + [top pixel]*alpha
        pixel_buffer[idx*3 + 0] = floor(pixel_buffer[idx*3 + 0]*(1.0f - alpha) + red*alpha);
        pixel_buffer[idx*3 + 1] = floor(pixel_buffer[idx*3 + 1]*(1.0f - alpha) + grn*alpha);
        pixel_buffer[idx*3 + 2] = floor(pixel_buffer[idx*3 + 2]*(1.0f - alpha) + blu*alpha);
    } else {
        // pixel is outside of range
    }
}

void MatrixPanel::drawBuffer() {
    for (int idx=0; idx<_width*_height; idx++) {
        uint16_t x = idx % (_width*2);
        uint16_t y = idx / (_width*2);
        
        uint8_t r = pixel_buffer[idx*3 + 0];
        uint8_t g = pixel_buffer[idx*3 + 1];
        uint8_t b = pixel_buffer[idx*3 + 2];

        dma_display->drawPixelRGB888(x, y, r, g, b);
    }
}

void MatrixPanel::setBrightness(uint8_t brightness) {
    dma_display->setBrightness8(brightness);
}

void MatrixPanel::fillScreen(uint16_t color) {
    for (int idx=0; idx<_width*_height; idx++) {
        uint16_t x = idx % (_width*2);
        uint16_t y = idx / (_width*2);
        pixel_buffer[idx*3 + 0] = ((((color >> 11) & 0x1F) * 527) + 23) >> 6;
        pixel_buffer[idx*3 + 1] = ((((color >> 5) & 0x3F) * 259) + 33) >> 6;
        pixel_buffer[idx*3 + 2] = (((color & 0x1F) * 527) + 23) >> 6;
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

// https://rosettacode.org/wiki/Xiaolin_Wu's_line_algorithm
void MatrixPanel::drawLineWu(float x0, float y0, float x1, float y1, uint8_t red, uint8_t grn, uint8_t blu) {
    auto ipart = [](float x) -> int {return int(std::floor(x));};
    auto round = [](float x) -> float {return std::round(x);};
    auto fpart = [](float x) -> float {return x - std::floor(x);};
    auto rfpart = [=](float x) -> float {return 1 - fpart(x);};
        
    const bool steep = abs(y1 - y0) > abs(x1 - x0);

    if (steep) {
        std::swap(x0,y0);
        std::swap(x1,y1);
    }
    if (x0 > x1) {
        std::swap(x0,x1);
        std::swap(y0,y1);
    }
        
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float gradient = (dx == 0) ? 1 : dy/dx;
        
    int xpx11;
    float intery;
    {
        const float xend = round(x0);
        const float yend = y0 + gradient * (xend - x0);
        const float xgap = rfpart(x0 + 0.5);
        xpx11 = int(xend);
        const int ypx11 = ipart(yend);
        if (steep) {
            drawPixelRGBA(ypx11,     xpx11, red, grn, blu, rfpart(yend) * xgap);
            drawPixelRGBA(ypx11 + 1, xpx11, red, grn, blu,  fpart(yend) * xgap);
        } else {
            drawPixelRGBA(xpx11, ypx11,     red, grn, blu, rfpart(yend) * xgap);
            drawPixelRGBA(xpx11, ypx11 + 1, red, grn, blu,  fpart(yend) * xgap);
        }
        intery = yend + gradient;
    }
    
    int xpx12;
    {
        const float xend = round(x1);
        const float yend = y1 + gradient * (xend - x1);
        const float xgap = rfpart(x1 + 0.5);
        xpx12 = int(xend);
        const int ypx12 = ipart(yend);
        if (steep) {
            drawPixelRGBA(ypx12,     xpx12, red, grn, blu, rfpart(yend) * xgap);
            drawPixelRGBA(ypx12 + 1, xpx12, red, grn, blu, fpart(yend) * xgap);
        } else {
            drawPixelRGBA(xpx12, ypx12,    red, grn, blu, rfpart(yend) * xgap);
            drawPixelRGBA(xpx12, ypx12 + 1,red, grn, blu, fpart(yend) * xgap);
        }
    }
        
    if (steep) {
        for (int x = xpx11 + 1; x < xpx12; x++) {
            drawPixelRGBA(ipart(intery),     x, red, grn, blu, rfpart(intery));
            drawPixelRGBA(ipart(intery) + 1, x, red, grn, blu, fpart(intery));
            intery += gradient;
        }
    } else {
        for (int x = xpx11 + 1; x < xpx12; x++) {
            drawPixelRGBA(x, ipart(intery),     red, grn, blu, rfpart(intery));
            drawPixelRGBA(x, ipart(intery) + 1, red, grn, blu, fpart(intery));
            intery += gradient;
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

void MatrixPanel::fillQuat(float px[4], float py[4], uint8_t r, uint8_t g, uint8_t b, float alpha){
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
					drawPixelRGBA(pixelX, pixelY, r, g, b, alpha);
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