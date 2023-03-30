matrix.setBrightness(255)

verts = {
    -1,-1,-1,  --0           1---------5         ^ +z
    -1,-1, 1,  --1          /|        /|         |
    -1, 1,-1,  --2         3-|-------7 |         |
    -1, 1, 1,  --3         | |       | |         |
     1,-1,-1,  --4         | |       | |         o--------> +x
     1,-1, 1,  --5         | 0---------4
     1, 1,-1,  --6         |/        |/
     1, 1, 1   --7         2---------6
}

faces = {
    1,3,7,5,
    2,0,4,6,
    1,0,2,3,
    7,6,4,5,
    3,2,6,7,
    5,4,0,1,
}

face_colors = {
    255, 128, 128,
    128, 255, 128,
    128, 128, 255,
    255, 255, 128,
    128, 255, 255,
    255, 128, 255
}

matrix.set3dVertices(verts)
matrix.set3dFaces(faces)
matrix.set3dFaceColors(face_colors)

matrix.calculate3dNormals()

i = 0.0
function run()
    matrix.clearScreen()
    matrix.set3dRotation(i/2, i/3, i/5)
    matrix.draw3dsolid(255, 255, 255)
    i = i + 0.1
    return 1
end

setInterval(10)