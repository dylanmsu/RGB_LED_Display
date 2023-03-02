matrix.setBrightness(4)

--vertices (x, y, z)
matrix.push3dVertex(-1,-1,-1) --0           1---------5         ^ +z
matrix.push3dVertex(-1,-1, 1) --1          /|        /|         |
matrix.push3dVertex(-1, 1,-1) --2         3-|-------7 |         |
matrix.push3dVertex(-1, 1, 1) --3         | |       | |         |
matrix.push3dVertex( 1,-1,-1) --4         | |       | |         o--------> +x
matrix.push3dVertex( 1,-1, 1) --5         | 0---------4
matrix.push3dVertex( 1, 1,-1) --6         |/        |/
matrix.push3dVertex( 1, 1, 1) --7         2---------6

--faces (v1, v2, v3, v4, r, g, b)
matrix.push3dQuat(1,3,7,5, 255, 128, 128)
matrix.push3dQuat(2,0,4,6, 128, 255, 128)
matrix.push3dQuat(1,0,2,3, 128, 128, 255)
matrix.push3dQuat(7,6,4,5, 255, 255, 128)
matrix.push3dQuat(3,2,6,7, 128, 255, 255)
matrix.push3dQuat(5,4,0,1, 255, 128, 255)

i = 0.0
while (true) do
    matrix.set3dRotation(i/2, i/3, i/5)
    matrix.draw3dsolid()
    updateDisplay()
    delay(10)
    i = i + 0.1
    matrix.clearScreen()
end