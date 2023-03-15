#include "Lua_libs/Lua_box2d_libs.h"

Lua_box2d_libs::Lua_box2d_libs(lua_State *lua, MatrixPanel *MatrixPanel)
{
    Lua_box2d_libs::_lua = lua;
    Lua_box2d_libs::_matrix = MatrixPanel;

    num_bodies = 0;

    //b2Vec2 gravity(0.0f, -10.0f);
    //b2World world(gravity);

    //b2BodyDef groundBodyDef;
    //groundBodyDef.position.Set(0.0f, -10.0f);
    //b2Body* groundBody = world.CreateBody(&groundBodyDef);
    //b2PolygonShape groundBox;
    //groundBox.SetAsBox(50.0f, 10.0f);
    //groundBody->CreateFixture(&groundBox, 0.0f);

    //b2BodyDef bodyDef;
    //bodyDef.type = b2_dynamicBody;
    //bodyDef.position.Set(0.0f, 4.0f);
    //b2Body* body = world.CreateBody(&bodyDef);
    //b2PolygonShape dynamicBox;
    //dynamicBox.SetAsBox(1.0f, 1.0f);
    //b2FixtureDef fixtureDef;
    //fixtureDef.shape = &dynamicBox;
    //fixtureDef.density = 1.0f;
    //fixtureDef.friction = 0.3f;
    //body->CreateFixture(&fixtureDef);

    //float timeStep = 1.0f / 60.0f;
    //int32 velocityIterations = 6;
    //int32 positionIterations = 2;
    //for (int32 i=0; i<60; ++i)
    //{
    //    world.Step(timeStep, velocityIterations, positionIterations);
    //    b2Vec2 position = body->GetPosition();
    //    float angle = body->GetAngle();
    //    printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
    //}
}

Lua_box2d_libs::~Lua_box2d_libs()
{
}

lua_State *Lua_box2d_libs::_lua = nullptr;
MatrixPanel *Lua_box2d_libs::_matrix = nullptr;

b2World *Lua_box2d_libs::_world = nullptr;
b2Body **Lua_box2d_libs::_bodies = nullptr;
int Lua_box2d_libs::num_bodies = 0;

int Lua_box2d_libs::lua_createWorld(lua_State *lua_state) {
    float gravity_x = luaL_checknumber(lua_state, 1);
    float gravity_y = luaL_checknumber(lua_state, 2);

    b2Vec2 gravity(gravity_x, gravity_y);
    _world = new b2World(gravity);

    return 1;
}

int Lua_box2d_libs::lua_createDynamicBody(lua_State *lua_state) {
    float position_x = luaL_checknumber(lua_state, 1);
    float position_y = luaL_checknumber(lua_state, 2);

    _bodies = (b2Body **) realloc(_bodies, sizeof(b2Body **)*(num_bodies + 1));

    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(position_x, position_y);
    _bodies[num_bodies] = _world->CreateBody(&bodyDef);

    lua_pushinteger(lua_state, num_bodies);

    num_bodies += 1;
    return 1;
}

int Lua_box2d_libs::lua_createStaticBody(lua_State *lua_state) {
    float position_x = luaL_checknumber(lua_state, 1);
    float position_y = luaL_checknumber(lua_state, 2);

    _bodies = (b2Body **) realloc(_bodies, sizeof(b2Body **)*(num_bodies + 1));

    b2BodyDef bodyDef;
    bodyDef.type = b2_staticBody;
    bodyDef.position.Set(position_x, position_y);
    _bodies[num_bodies] = _world->CreateBody(&bodyDef);

    lua_pushinteger(lua_state, num_bodies);

    num_bodies += 1;
    return 1;
}

int Lua_box2d_libs::lua_setAsBox(lua_State *lua_state) {
    int body_idx = luaL_checkinteger(lua_state, 1);
    float width = luaL_checknumber(lua_state, 2);
    float height = luaL_checknumber(lua_state, 3);
    float angle = luaL_checknumber(lua_state, 4);
    float density = luaL_checknumber(lua_state, 5);
    float friction = luaL_checknumber(lua_state, 6);
    float restitution = luaL_checknumber(lua_state, 7);

    b2PolygonShape box;
    box.SetAsBox(width, height);
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &box;
    fixtureDef.density = density;
    fixtureDef.friction = friction;
    fixtureDef.restitution = restitution;
    _bodies[body_idx]->CreateFixture(&fixtureDef);

    //lua_pushnumber(lua_state, body_idx);
    return 1;
}

int Lua_box2d_libs::lua_setAsCircle(lua_State *lua_state) {
    int body_idx = luaL_checkinteger(lua_state, 1);
    float radius = luaL_checknumber(lua_state, 2);
    float density = luaL_checknumber(lua_state, 3);
    float friction = luaL_checknumber(lua_state, 4);
    float restitution = luaL_checknumber(lua_state, 5);

    b2CircleShape circle;
    circle.m_radius = radius;
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circle;
    fixtureDef.density = density;
    fixtureDef.friction = friction;
    fixtureDef.restitution = restitution;
    _bodies[body_idx]->CreateFixture(&fixtureDef);

    //lua_pushnumber(lua_state, body_idx);
    return 1;
}

int Lua_box2d_libs::lua_applyForceToCenter(lua_State *lua_state) {
    int body_idx = luaL_checkinteger(lua_state, 1);
    float force_x = luaL_checknumber(lua_state, 2);
    float force_y = luaL_checknumber(lua_state, 3);

    b2Vec2 force(force_x, force_y);
    _bodies[body_idx]->ApplyForceToCenter(force, true);

    //lua_pushnumber(lua_state, body_idx);
    return 1;
}

int Lua_box2d_libs::lua_getRotation(lua_State *lua_state) {
    int body_idx = luaL_checkinteger(lua_state, 1);

    float angle = _bodies[body_idx]->GetAngle();

    lua_pushnumber(lua_state, angle);
    return 1;
}

int Lua_box2d_libs::lua_getPosition(lua_State *lua_state) {
    int body_idx = luaL_checkinteger(lua_state, 1);

    b2Vec2 position = _bodies[body_idx]->GetPosition();

    // return 2d vector to lua
    lua_createtable(lua_state, 2, 0);
    lua_pushnumber(lua_state, position.x);
    lua_rawseti (lua_state, -2, 1);
    lua_pushnumber(lua_state, position.y);
    lua_rawseti (lua_state, -2, 2);
    return 1;
}

int Lua_box2d_libs::lua_step(lua_State *lua_state) {
    float timeStep = luaL_checknumber(lua_state, 1);
    int velocityIterations = luaL_checkinteger(lua_state, 2);
    int positionIterations = luaL_checkinteger(lua_state, 3);

    _world->Step(timeStep, velocityIterations, positionIterations);
    return 1;
}

const luaL_Reg Lua_box2d_libs::box2d_functions[] = {
    {"createWorld", lua_createWorld},
    {"createDynamicBody", lua_createDynamicBody},
    {"createStaticBody", lua_createStaticBody},
    {"setAsBox", lua_setAsBox},
    {"setAsCircle", lua_setAsCircle},
    {"applyForceToCenter", lua_applyForceToCenter},
    {"getPosition", lua_getPosition},
    {"getRotation", lua_getRotation},
    {"step", lua_step},
    {NULL, NULL}
};

int Lua_box2d_libs::luaopen_box2d_lib(lua_State *L) {
    luaL_newlib(L, box2d_functions);
    return 1;
}
