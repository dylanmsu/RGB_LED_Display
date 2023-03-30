ppm = 1/0.006

physics.createWorld(0.1, 10)

ground = physics.createStaticBody(16/ppm, 34/ppm)
physics.setAsBox(ground, 32/ppm, 1/ppm, 0, 1, 0.5, 0.8)

ceiling = physics.createStaticBody(16/ppm, -2/ppm)
physics.setAsBox(ceiling, 32/ppm, 1/ppm, 0, 1, 0.5, 0.8)

left_wall = physics.createStaticBody(-2/ppm, 16/ppm)
physics.setAsBox(left_wall, 1/ppm, 32/ppm, 0, 1, 0.5, 0.8)

right_wall = physics.createStaticBody(34/ppm, 16/ppm)
physics.setAsBox(right_wall, 1/ppm, 32/ppm, 0, 1, 0.5, 0.8)

boxes = {}
radius = 8
num = 9
for i=1, num do
    boxes[i] = physics.createDynamicBody((radius/2+(radius + 1)*((i-1)%3))/ppm, (radius/2+(radius + 1)*((i-1)//3))/ppm)
    physics.setAsCircle(boxes[i], (radius/ppm)/2, 0.1, 0.5, 0.8)
end

function run() 
    physics.step(0.01, 2, 2)
    physics.setGravity(getAccX()*9.81, getAccY()*9.81)
    matrix.clearScreen()
    for i=1,num do
        position = physics.getPosition(boxes[i])
        rotation = physics.getRotation(boxes[i])

        local red = math.sin((((2*math.pi)/8)*i + (2*math.pi)/3*1)) * 127 + 128
        local grn = math.sin((((2*math.pi)/8)*i + (2*math.pi)/3*2)) * 127 + 128
        local blu = math.sin((((2*math.pi)/8)*i + (2*math.pi)/3*3)) * 127 + 128
        matrix.drawCircle(position[1]*ppm, position[2]*ppm, (radius/2), rotation, math.floor(red), math.floor(grn), math.floor(blu))
    end
    return 0
end

matrix.setBrightness(255)
setInterval(10)