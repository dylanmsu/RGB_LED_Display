--vertices
push3dVertex(-1,-1,-1) --0           1---------5         ^ +z
push3dVertex(-1,-1, 1) --1          /|        /|         |
push3dVertex(-1, 1,-1) --2         3-|-------7 |         |
push3dVertex(-1, 1, 1) --3         | |       | |         |
push3dVertex( 1,-1,-1) --4         | |       | |         o--------> +x
push3dVertex( 1,-1, 1) --5         | 0---------4
push3dVertex( 1, 1,-1) --6         |/        |/
push3dVertex( 1, 1, 1) --7         2---------6

--faces
push3dQuat(1,3,7,5, 0.8, 0.5, 0.5)
push3dQuat(2,0,4,6, 0.5, 0.8, 0.5)
push3dQuat(1,0,2,3, 0.5, 0.5, 0.8)
push3dQuat(7,6,4,5, 0.8, 0.8, 0.5)
push3dQuat(3,2,6,7, 0.5, 0.8, 0.8)
push3dQuat(5,4,0,1, 0.8, 0.5, 0.8)

i = 0.0
while (true) do
    set3dRotation(i/2, i/3, i/5)    --set mesh rotation in radians
    draw3dsolid()                   --draw 3d mesh
    delay(10)
    clearScreen()
    i = i + 0.1
end
