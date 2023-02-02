#include "Arduino.h"
#include "MatrixPanel.h"

#ifndef GRAPHICS_H
#define GRAPHICS_H

class Graphics
{
private:
    MatrixPanel *matrix;

public:
    Graphics(MatrixPanel *mat);
    ~Graphics();

    void drawMesh(double verts[], uint16_t edges[],int edgeCnt, int r, int g, int b);
    void drawSolid(double *verts, int *faces, int facelen, float r, float g, float b);
};

#endif
