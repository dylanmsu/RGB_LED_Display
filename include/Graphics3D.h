#ifndef GRAPHICS_3D_H
#define GRAPHICS_3D_H

#include "MatrixPanel.h"
#include "FastMath.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#define VERTS_PER_FACE 4

class Graphics3D
{
private:
    MatrixPanel *matrixPanel;

    float *vertices;
    int *faces;
    float *face_colors;
    float *face_normals;

    float *transformed_vertices;
    float *transformed_face_normals;

    int face_count;
    int vert_count;

    bool enable_specular = false;
    bool enable_diffuse = true;

    bool normals_precalculated = false;

    // parapeters that may change later
    float rotation[3] = {0.0, 0.0, 0.0};
    float cameraPos[3] = {0.0, 4.0, 0.0};
    float lightColor[3] = {1.0, 1.0, 1.0};
    float lightPos[3] = {-5.0, 5.0, 5.0};
    float lightPower = 100;

public:
    Graphics3D(MatrixPanel *matrixPanel);
    ~Graphics3D();

    void calculateNormals();

    void drawSolid(float *verts, int *faces, float *colors, int facelen, double zoom);
    void pushVertex(float x, float y, float z);
    void pushQuat(int p1, int p2, int p3, int p4, uint8_t r, uint8_t g, uint8_t b);
    void setRotation(float x, float y, float z);
    void drawMesh();
};

#endif  // GRAPHICS_3D_H