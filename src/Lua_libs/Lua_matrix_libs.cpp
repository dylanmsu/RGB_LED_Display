#include "Lua_libs/Lua_matrix_libs.h"

Lua_matrix_libs::Lua_matrix_libs(lua_State *lua, MatrixPanel *matrix, Graphics3D *graphics3d)
{
    Lua_matrix_libs::_lua = lua;
    Lua_matrix_libs::_matrix = matrix;
    Lua_matrix_libs::_graphics3d = graphics3d;
}

Lua_matrix_libs::~Lua_matrix_libs()
{
}

lua_State *Lua_matrix_libs::_lua = nullptr;
MatrixPanel *Lua_matrix_libs::_matrix = nullptr;
Graphics3D *Lua_matrix_libs::_graphics3d = nullptr;

const luaL_Reg Lua_matrix_libs::matrixfunctions[] = {
    {"drawPixel",  lua_drawPixel},
    {"drawPixelHSV", lua_drawPixelHSV},
    {"drawLine", lua_drawLine},
    {"drawLineWu", lua_drawLineWu},
    {"fillQuat", lua_fillQuat},
    {"drawCircle", lua_drawCircle},
    {"clearScreen", lua_clearScreen},
    {"setBrightness", lua_setBrightness},
    {"push3dVertex", lua_push3dVertex},
    {"push3dQuat", lua_push3dQuat},
    {"set3dRotation", lua_set3dRotation},
    {"draw3dsolid", lua_draw3dsolid},
    {"calculate3dNormals", lua_calculate3dNormals},
    {"draw", lua_updateScreen},
    {NULL, NULL}
};

int Lua_matrix_libs::lua_drawPixel(lua_State *lua_state) {
    int x = luaL_checkinteger(lua_state, 1);
    int y = luaL_checkinteger(lua_state, 2);
    int red = luaL_checkinteger(lua_state, 3);
    int grn = luaL_checkinteger(lua_state, 4);
    int blu = luaL_checkinteger(lua_state, 5);
    _matrix->drawPixel(x, y, red, grn, blu);
    return 1;
}

int Lua_matrix_libs::lua_drawPixelHSV(lua_State *lua_state) {
    int x = luaL_checkinteger(lua_state, 1);
    int y = luaL_checkinteger(lua_state, 2);
    int hue = luaL_checkinteger(lua_state, 3);
    int sat = luaL_checkinteger(lua_state, 4);
    int val = luaL_checkinteger(lua_state, 5);
    _matrix->drawPixelHSV(x, y, hue, sat, val);
    return 1;
}

int Lua_matrix_libs::lua_drawLine(lua_State *lua_state) {
    int x0 = luaL_checkinteger(lua_state, 1);
    int y0 = luaL_checkinteger(lua_state, 2);
    int x1 = luaL_checkinteger(lua_state, 3);
    int y1 = luaL_checkinteger(lua_state, 4);
    int red = luaL_checkinteger(lua_state, 5);
    int grn = luaL_checkinteger(lua_state, 6);
    int blu = luaL_checkinteger(lua_state, 7);
    _matrix->drawLine(x0, y0, x1, y1, red, grn, blu);
    return 1;
}

int Lua_matrix_libs::lua_drawLineWu(lua_State *lua_state) {
    float x0 = luaL_checknumber(lua_state, 1);
    float y0 = luaL_checknumber(lua_state, 2);
    float x1 = luaL_checknumber(lua_state, 3);
    float y1 = luaL_checknumber(lua_state, 4);
    uint8_t red = luaL_checkinteger(lua_state, 5);
    uint8_t grn = luaL_checkinteger(lua_state, 6);
    uint8_t blu = luaL_checkinteger(lua_state, 7);
    _matrix->drawLineWu(x0, y0, x1, y1, red, grn, blu);
    return 1;
}

int Lua_matrix_libs::lua_fillQuat(lua_State *lua_state) {
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
    float alpha = luaL_checknumber(lua_state, 12);
    
    _matrix->fillQuat((float *)px, (float *)py, red, grn, blu, alpha);
    return 1;
}

int Lua_matrix_libs::lua_clearScreen(lua_State *lua_state) {
    _matrix->fillScreen(0x0000);
    return 1;
}

int Lua_matrix_libs::lua_setBrightness(lua_State *lua_state) {
    _matrix->setBrightness(luaL_checkinteger(lua_state, 1));
    return 1;
}

int Lua_matrix_libs::lua_push3dVertex(lua_State *lua_state) {
    _graphics3d->pushVertex(
        (float)luaL_checknumber(lua_state, 1),
        (float)luaL_checknumber(lua_state, 2),
        (float)luaL_checknumber(lua_state, 3)
    );

    return 1;
}

int Lua_matrix_libs::lua_push3dQuat(lua_State *lua_state) {
    fflush(stdout);
    int p1 = luaL_checkinteger(lua_state, 1);
    int p2 = luaL_checkinteger(lua_state, 2);
    int p3 = luaL_checkinteger(lua_state, 3);
    int p4 = luaL_checkinteger(lua_state, 4);

    uint8_t r = luaL_checknumber(lua_state, 5);
    uint8_t g = luaL_checknumber(lua_state, 6);
    uint8_t b = luaL_checknumber(lua_state, 7);
    _graphics3d->pushQuat(p1, p2, p3, p4, r, g, b);
    return 1;
}

int Lua_matrix_libs::lua_set3dRotation(lua_State *lua_state) {
    float x = luaL_checknumber(lua_state, 1);
    float y = luaL_checknumber(lua_state, 2);
    float z = luaL_checknumber(lua_state, 3);
    _graphics3d->setRotation(x, y, z);
    return 1;
}

int Lua_matrix_libs::lua_calculate3dNormals(lua_State *lua_state) {
    _graphics3d->calculateNormals();
    return 1;
}

int Lua_matrix_libs::lua_draw3dsolid(lua_State *lua_state) {
    _graphics3d->drawMesh();
    return 1;
}

int Lua_matrix_libs::lua_drawCircle(lua_State *lua_state) {
    int xc = luaL_checkinteger(lua_state, 1);
    int yc = luaL_checkinteger(lua_state, 2);
    int r = luaL_checkinteger(lua_state, 3);
    uint8_t red = luaL_checkinteger(lua_state, 4);
    uint8_t green = luaL_checkinteger(lua_state, 5);
    uint8_t blue = luaL_checkinteger(lua_state, 6);

    int x = 0, y = r;
    int d = 3 - 2 * r;
    _matrix->drawPixel(xc+x, yc+y, red, green, blue);
    _matrix->drawPixel(xc-x, yc+y, red, green, blue);
    _matrix->drawPixel(xc+x, yc-y, red, green, blue);
    _matrix->drawPixel(xc-x, yc-y, red, green, blue);
    _matrix->drawPixel(xc+y, yc+x, red, green, blue);
    _matrix->drawPixel(xc-y, yc+x, red, green, blue);
    _matrix->drawPixel(xc+y, yc-x, red, green, blue);
    _matrix->drawPixel(xc-y, yc-x, red, green, blue);
    while (y >= x)
    {
        // for each pixel we will
        // draw all eight pixels
         
        x++;
 
        // check for decision parameter
        // and correspondingly
        // update d, x, y
        if (d > 0)
        {
            y--;
            d = d + 4 * (x - y) + 10;
        }
        else
        {
            d = d + 4 * x + 6;
        }
        _matrix->drawPixel(xc+x, yc+y, red, green, blue);
        _matrix->drawPixel(xc-x, yc+y, red, green, blue);
        _matrix->drawPixel(xc+x, yc-y, red, green, blue);
        _matrix->drawPixel(xc-x, yc-y, red, green, blue);
        _matrix->drawPixel(xc+y, yc+x, red, green, blue);
        _matrix->drawPixel(xc-y, yc+x, red, green, blue);
        _matrix->drawPixel(xc+y, yc-x, red, green, blue);
        _matrix->drawPixel(xc-y, yc-x, red, green, blue);
    }
    return 1;
}

int Lua_matrix_libs::lua_updateScreen(lua_State *lua_state) {
    _matrix->drawBuffer();
    return 1;
}

int Lua_matrix_libs::luaopen_matrix_lib(lua_State *L) {
    luaL_newlib(L, matrixfunctions);
    lua_pushnumber(L, _matrix->getWidth());
    lua_setfield(L, -2, "width");
    lua_pushnumber(L, _matrix->getHeight());
    lua_setfield(L, -2, "height");
    return 1;
}
