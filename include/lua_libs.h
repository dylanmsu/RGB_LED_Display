#include "MatrixPanel.h"
#include "Graphics3D.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

#ifndef MATRIX_LIBS_H
#define MATRIX_LIBS_H

class Lua_libs
{
private:
    static lua_State *lua;
    static MatrixPanel *mtrx;
    static Graphics3D *graphics3d;

    static int lua_drawPixel(lua_State *lua_state);
    static int lua_drawPixelHSV(lua_State *lua_state);
    static int lua_drawLine(lua_State *lua_state);
    static int lua_fillQuat(lua_State *lua_state);
    static int lua_clearScreen(lua_State *lua_state);

    static int lua_push3dVertex(lua_State *lua_state);
    static int lua_push3dQuat(lua_State *lua_state);
    static int lua_set3dRotation(lua_State *lua_state);
    static int lua_draw3dsolid(lua_State *lua_state);

    static const luaL_Reg matrixfunctions[];

public:
    Lua_libs(lua_State *lua, MatrixPanel *matrix, Graphics3D *graphics3d);
    ~Lua_libs();

    static int luaopen_matrix_lib(lua_State *L);
};

#endif