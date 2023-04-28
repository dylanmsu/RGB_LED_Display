#ifndef BOX2D_LIBS_H
#define BOX2D_LIBS_H

#include "MatrixPanel.h"

#include "box2d/b2_math.h"
#include "box2d/b2_world.h"
#include "box2d/b2_body.h"
#include "box2d/b2_circle_shape.h"
#include "box2d/b2_polygon_shape.h"
#include "box2d/b2_fixture.h"

extern "C" {
    #include "lua.h"
    #include "lualib.h"
    #include "lauxlib.h"
}

class Lua_box2d_libs
{
private:
    static lua_State *_lua;
    static MatrixPanel *_matrix;

    static b2World *_world;

    static int num_bodies;
    static b2Body** _bodies;

    static int lua_createWorld(lua_State *lua_state);
    static int lua_setGravity(lua_State *lua_state);

    static int lua_createStaticBody(lua_State *lua_state);
    static int lua_createDynamicBody(lua_State *lua_state);
    static int lua_setAsBox(lua_State *lua_state);
    static int lua_setAsCircle(lua_State *lua_state);

    static int lua_applyForceToCenter(lua_State *lua_state);
    
    static int lua_getPosition(lua_State *lua_state);
    static int lua_getRotation(lua_State *lua_state);

    static int lua_drawCircle(lua_State *lua_state);

    static int lua_step(lua_State *lua_state);

    static const luaL_Reg box2d_functions[];

public:
    Lua_box2d_libs(lua_State *lua, MatrixPanel *matrix);
    ~Lua_box2d_libs();

    static int luaopen_box2d_lib(lua_State *L);
};

#endif