#include "MatrixPanel.h"
#include <vector>

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#ifndef GRAPHICS_3D
#define GRAPHICS_3D

class Graphics3D
{
private:
    MatrixPanel *matrixPanel;

    std::vector<float> vertices;
    std::vector<int> faces;
    std::vector<float> face_colors;

    bool enable_specular = false;
    bool enable_diffuse = true;

    double rotation[3] = {0.0, 0.0, 0.0};


public:
    Graphics3D(MatrixPanel *matrixPanel);
    ~Graphics3D();

    void drawSolid(double *verts, int *faces, float *colors, int facelen, double zoom);
    void pushVertex(float x, float y, float z);
    void pushQuat(int p1, int p2, int p3, int p4, uint8_t r, uint8_t g, uint8_t b);
    void setRotation(float x, float y, float z);
    void drawMesh();
};

#endif