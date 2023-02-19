#include "lua_libs.h"

Lua_libs::Lua_libs(lua_State *lua, MatrixPanel *matrix, Graphics3D *graphics3d)
{
    lua = lua;
    mtrx = matrix;
    graphics3d = graphics3d;
}

Lua_libs::~Lua_libs()
{
}

lua_State *Lua_libs::lua = nullptr;
MatrixPanel *Lua_libs::mtrx = nullptr;
Graphics3D *Lua_libs::graphics3d = nullptr;

const luaL_Reg Lua_libs::matrixfunctions[] = {
    {"drawPixel",  lua_drawPixel},
    {"drawPixelHSV", lua_drawPixelHSV},
    {"drawLine", lua_drawLine},
    {"fillQuat", lua_fillQuat},
    {"clearScreen", lua_clearScreen},
    {"push3dVertex", lua_push3dVertex},
    {"push3dQuat", lua_push3dQuat},
    {"set3dRotation", lua_set3dRotation},
    {"draw3dsolid", lua_draw3dsolid},
    {NULL, NULL}
};

int Lua_libs::lua_drawPixel(lua_State *lua_state) {
    int x = luaL_checkinteger(lua_state, 1);
    int y = luaL_checkinteger(lua_state, 2);
    int red = luaL_checkinteger(lua_state, 3);
    int grn = luaL_checkinteger(lua_state, 4);
    int blu = luaL_checkinteger(lua_state, 5);
    mtrx->drawPixel(x, y, red, grn, blu);
    return 0;
}

int Lua_libs::lua_drawPixelHSV(lua_State *lua_state) {
    int x = luaL_checkinteger(lua_state, 1);
    int y = luaL_checkinteger(lua_state, 2);
    int hue = luaL_checkinteger(lua_state, 3);
    int sat = luaL_checkinteger(lua_state, 4);
    int val = luaL_checkinteger(lua_state, 5);
    mtrx->drawPixelHSV(x, y, hue, sat, val);
    return 0;
}

int Lua_libs::lua_drawLine(lua_State *lua_state) {
    int x0 = luaL_checkinteger(lua_state, 1);
    int y0 = luaL_checkinteger(lua_state, 2);
    int x1 = luaL_checkinteger(lua_state, 3);
    int y1 = luaL_checkinteger(lua_state, 4);
    int red = luaL_checkinteger(lua_state, 5);
    int grn = luaL_checkinteger(lua_state, 6);
    int blu = luaL_checkinteger(lua_state, 7);
    mtrx->drawLine(x0, y0, x1, y1, red, grn, blu);
    return 0;
}

int Lua_libs::lua_fillQuat(lua_State *lua_state) {
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
    
    mtrx->fillQuat((float *)px, (float *)py, red, grn, blu);
    return 0;
}

int Lua_libs::lua_clearScreen(lua_State *lua_state) {
    mtrx->fillScreen(0x0000);
    return 0;
}

int Lua_libs::lua_push3dVertex(lua_State *lua_state) {
    graphics3d->pushVertex(
        (float)luaL_checknumber(lua_state, 1),
        (float)luaL_checknumber(lua_state, 2),
        (float)luaL_checknumber(lua_state, 3)
    );
    return 0;
}

int Lua_libs::lua_push3dQuat(lua_State *lua_state) {
    int p1 = luaL_checkinteger(lua_state, 1);
    int p2 = luaL_checkinteger(lua_state, 2);
    int p3 = luaL_checkinteger(lua_state, 3);
    int p4 = luaL_checkinteger(lua_state, 4);

    uint8_t r = luaL_checknumber(lua_state, 5);
    uint8_t g = luaL_checknumber(lua_state, 6);
    uint8_t b = luaL_checknumber(lua_state, 7);
    graphics3d->pushQuat(p1, p2, p3, p4, r, g, b);
    return 0;
}

int Lua_libs::lua_set3dRotation(lua_State *lua_state) {
    float x = luaL_checknumber(lua_state, 1);
    float y = luaL_checknumber(lua_state, 2);
    float z = luaL_checknumber(lua_state, 3);
    graphics3d->setRotation(x, y, z);
    return 0;
}

int Lua_libs::lua_draw3dsolid(lua_State *lua_state) {
    graphics3d->drawMesh();
    return 0;
}

int Lua_libs::luaopen_matrix_lib(lua_State *L) {
    luaL_newlib(L, matrixfunctions);
    return 1;
}