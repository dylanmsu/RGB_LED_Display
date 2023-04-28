# lua api

## general
- draw()
- setBrightness(brightness)
- sin8(theta)
- sin16(theta)
- getSeconds()
- getMinutes()
- getHours()
- random(min, max)
- setInterval(delta_t)
- getAccX()
- getAccY()
- getAccZ()

## pixel manipulation
- drawPixel(x, y, r, g, b)
- drawPixelHSV(x, y, h, s, v)
- drawLine(x0, y0, x1, y1, r, g, b)
- drawLineWu(x0, y0, x1, y1, r, g, b)
- fillQuat(x0, y0, x1, y1, x2, y2, x3, y3, r, g, b)
- drawCircle(x, y, radius, r, g, b)
- clearScreen()



## 3d graphics
- push3dVertex(x, y, z)
- push3dQuat(p1, p2, p3, p4, r, g, b)
- set3dRotation(theta_x, theta_y, theta_z)
- draw3dsolid()

## physics
- createWorld(gravity_x, gravity_y)
- createDynamicBody(position_x, position_y)
- createStaticBody(position_x, position_y)
- setAsBox(body_idx, width, height, density, friciton, restitution)
- setAsCircle(body_idx, radius, density, friciton, restitution)
- applyForceToCenter(body_idx, force_x, force_y)
- getPosition(body_idx)
- getRotation(body_idx)
- step(timeStep, velocityIterations, positionIterations)
