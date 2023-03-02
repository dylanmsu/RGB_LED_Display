matrix.setBrightness(255)
prev_seconds = 0

cx = 15.5
cy = 15.5

function draw()
    matrix.clearScreen()

    for p=0,11 do
        x_p = math.cos(p*(math.pi/6.0))*15.5 + cx
        y_p = math.sin(p*(math.pi/6.0))*15.5 + cy
        matrix.drawPixel(math.floor(x_p), math.floor(y_p), 64, 64, 64)
        matrix.drawPixel(math.floor(x_p), math.ceil(y_p), 64, 64, 64)
        matrix.drawPixel(math.ceil(x_p), math.floor(y_p), 64, 64, 64)
        matrix.drawPixel(math.ceil(x_p), math.ceil(y_p), 64, 64, 64)

    end

    x_s = math.cos((prev_seconds - 15)*(math.pi/30.0))*16 + cx
    y_s = math.sin((prev_seconds - 15)*(math.pi/30.0))*16 + cy
    matrix.drawLineWu(cx, cy, x_s, y_s, 255, 0, 0)

    minutes = getMinutes()
    x_m = math.cos((minutes - 15)*(math.pi/30.0))*13 + cx
    y_m = math.sin((minutes - 15)*(math.pi/30.0))*13 + cy
    matrix.drawLineWu(cx, cy, x_m, y_m, 0, 255, 0)

    hours = getHours()
    x_h = math.cos((hours - 3)*(math.pi/6.0))*10 + cx
    y_h = math.sin((hours - 3)*(math.pi/6.0))*10 + cy
    matrix.drawLineWu(cx, cy, x_h, y_h, 0, 0, 255)

    matrix.drawPixel(math.floor(cx), math.floor(cy), 64, 64, 64)
    matrix.drawPixel(math.floor(cx), math.ceil(cy), 64, 64, 64)
    matrix.drawPixel(math.ceil(cx), math.floor(cy), 64, 64, 64)
    matrix.drawPixel(math.ceil(cx), math.ceil(cy), 64, 64, 64)
end

while (true) do
    seconds = getSeconds()
    if (prev_seconds != seconds) do
        prev_seconds = seconds
        draw()
    end
    delay(50)
end