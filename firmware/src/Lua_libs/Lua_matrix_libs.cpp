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
    {"drawCircle", lua_drawAaCircle},
    {"clearScreen", lua_clearScreen},
    {"setBrightness", lua_setBrightness},
    {"set3dVertices", lua_set3dVertices},
    {"set3dFaces", lua_set3dFaces},
    {"set3dFaceColors", lua_set3dFaceColors},
    {"set3dRotation", lua_set3dRotation},
    {"draw3dsolid", lua_draw3dsolid},
    {"draw3dmesh", lua_draw3dmesh},
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

// https://stackoverflow.com/questions/25940366/passing-array-to-c-as-argument-in-the-stack
int Lua_matrix_libs::lua_set3dVertices(lua_State *lua_state) {
    luaL_checktype(lua_state, 1, LUA_TTABLE);
    // let alone excessive arguments (idiomatic), or do:
    lua_settop(lua_state, 1);

    int a_size = lua_rawlen(lua_state, 1); // absolute indexing for arguments
    float buf[a_size] = { 0 };

    for (int i = 1; i <= a_size; i++) {
        lua_pushinteger(lua_state, i);
        lua_gettable(lua_state, 1); // always give a chance to metamethods
        // OTOH, metamethods are already broken here with lua_rawlen()
        // if you are on 5.2, use lua_len()

        if (lua_isnil(lua_state, -1)) { // relative indexing for "locals"
            a_size = i-1; // fix actual size (e.g. 4th nil means a_size==3)
            break;
        }

        if (!lua_isnumber(lua_state, -1)) // optional check
            return luaL_error(lua_state, "item %d invalid (number required, got %s)",
                              i, luaL_typename(lua_state, -1));

        lua_Number b = lua_tonumber(lua_state, -1);

        buf[i-1] = b; // Lua is 1-based, C is 0-based
        lua_pop(lua_state, 1);
    }

    _graphics3d->setVertices(buf, a_size);

    return 0;
}

// https://stackoverflow.com/questions/25940366/passing-array-to-c-as-argument-in-the-stack
int Lua_matrix_libs::lua_set3dFaces(lua_State *lua_state) {
    luaL_checktype(lua_state, 1, LUA_TTABLE);
    // let alone excessive arguments (idiomatic), or do:
    lua_settop(lua_state, 1);

    int a_size = lua_rawlen(lua_state, 1); // absolute indexing for arguments
    int buf[a_size] = { 0 };

    for (int i = 1; i <= a_size; i++) {
        lua_pushinteger(lua_state, i);
        lua_gettable(lua_state, 1); // always give a chance to metamethods
        // OTOH, metamethods are already broken here with lua_rawlen()
        // if you are on 5.2, use lua_len()

        if (lua_isnil(lua_state, -1)) { // relative indexing for "locals"
            a_size = i-1; // fix actual size (e.g. 4th nil means a_size==3)
            break;
        }

        if (!lua_isnumber(lua_state, -1)) // optional check
            return luaL_error(lua_state, "item %d invalid (number required, got %s)",
                              i, luaL_typename(lua_state, -1));

        lua_Integer b = lua_tointeger(lua_state, -1);

        buf[i-1] = b; // Lua is 1-based, C is 0-based
        lua_pop(lua_state, 1);
    }

    _graphics3d->setFaces(buf, a_size);

    return 0;
}

// https://stackoverflow.com/questions/25940366/passing-array-to-c-as-argument-in-the-stack
int Lua_matrix_libs::lua_set3dFaceColors(lua_State *lua_state) {
    luaL_checktype(lua_state, 1, LUA_TTABLE);
    // let alone excessive arguments (idiomatic), or do:
    lua_settop(lua_state, 1);

    int a_size = lua_rawlen(lua_state, 1); // absolute indexing for arguments
    uint8_t buf[a_size] = { 0 };

    for (int i = 1; i <= a_size; i++) {
        lua_pushinteger(lua_state, i);
        lua_gettable(lua_state, 1); // always give a chance to metamethods
        // OTOH, metamethods are already broken here with lua_rawlen()
        // if you are on 5.2, use lua_len()

        if (lua_isnil(lua_state, -1)) { // relative indexing for "locals"
            a_size = i-1; // fix actual size (e.g. 4th nil means a_size==3)
            break;
        }

        if (!lua_isnumber(lua_state, -1)) // optional check
            return luaL_error(lua_state, "item %d invalid (number required, got %s)",
                              i, luaL_typename(lua_state, -1));

        lua_Integer b = lua_tointeger(lua_state, -1);

        buf[i-1] = b; // Lua is 1-based, C is 0-based
        lua_pop(lua_state, 1);
    }

    _graphics3d->setFaceColors(buf, a_size);

    return 0;
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
    _graphics3d->drawSolid();
    return 1;
}

int Lua_matrix_libs::lua_draw3dmesh(lua_State *lua_state) {
    uint8_t r = luaL_checkinteger(lua_state, 1);
    uint8_t g = luaL_checkinteger(lua_state, 2);
    uint8_t b = luaL_checkinteger(lua_state, 3);
    _graphics3d->drawMesh(r, g, b);
    return 0;
}

int Lua_matrix_libs::lua_drawAaCircle(lua_State *lua_state) {
    float x = luaL_checknumber(lua_state, 1);
    float y = luaL_checknumber(lua_state, 2);
    float rad = luaL_checknumber(lua_state, 3);
    uint8_t red = luaL_checkinteger(lua_state, 5);
    uint8_t green = luaL_checkinteger(lua_state, 6);
    uint8_t blue = luaL_checkinteger(lua_state, 7);
    int circle_points = 10;

    float prevX = rad;
    float prevY = 0;
    for (int i=0; i<circle_points; i++) {
        float xr = rad*cos(((2*3.141592)/circle_points)*i);
        float yr = rad*sin(((2*3.141592)/circle_points)*i);
        _matrix->drawLineWu(x + prevX, y + prevY, x + xr, y + yr, red, green, blue);
        prevX = xr;
        prevY = yr;
    }

    for (float yi=-rad; yi<rad; yi+=1) {
        for (float xi=-rad; xi<rad; xi+=1) {
            if (xi*xi+yi*yi <= rad*rad) {
                _matrix->drawPixel(floor(x+xi+0.5), floor(y+yi+0.5), red, green, blue);
            }
        }
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
