--based on the plasma example of adafruit
--https://github.com/adafruit/RGB-matrix-Panel/blob/master/examples/plasma_32x32/plasma_32x32.ino
matrix.setBrightness(255)

radius1 = 16.3
radius2 = 23.0
radius3 = 40.8
radius4 = 44.2

centerx1 = 16.1
centerx2 = 11.6
centerx3 = 23.4
centerx4 = 4.1
            
centery1 = 8.7
centery2 = 6.5
centery3 = 14.0
centery4 = -2.9

angle1 = 0.0
angle2 = 0.0
angle3 = 0.0
angle4 = 0.0

hueShift = 0

function run()
    sx1 = math.cos(angle1) * radius1 + centerx1;
    sx2 = math.cos(angle2) * radius2 + centerx2;
    sx3 = math.cos(angle3) * radius3 + centerx3;
    sx4 = math.cos(angle4) * radius4 + centerx4;
    y1  = math.sin(angle1) * radius1 + centery1;
    y2  = math.sin(angle2) * radius2 + centery2;
    y3  = math.sin(angle3) * radius3 + centery3;
    y4  = math.sin(angle4) * radius4 + centery4;

    for y=0,31 do
        x1 = sx1
        x2 = sx2
        x3 = sx3
        x4 = sx4
        for x=0,31 do
            value = hueShift
                + (sin8(math.floor((x1 * x1 + y1 * y1) / 4))-128)
                + (sin8(math.floor((x2 * x2 + y2 * y2) / 4))-128)
                + (sin8(math.floor((x3 * x3 + y3 * y3) / 8))-128)
                + (sin8(math.floor((x4 * x4 + y4 * y4) / 8))-128)
            matrix.drawPixelHSV(x, y, math.floor(value/2)%360, 100, 100);
            x1 = x1 - 1
            x2 = x2 - 1
            x3 = x3 - 1
            x4 = x4 - 1
        end
        y1 = y1 - 1
        y2 = y2 - 1
        y3 = y3 - 1
        y4 = y4 - 1
    end

    matrix.draw()

    mult = 0.001
    angle1 = angle1 + 3*mult;
    angle2 = angle2 - 7*mult;
    angle3 = angle3 + 13*mult;
    angle4 = angle4 + 15*mult;
    hueShift = hueShift + 200*mult;
end

setInterval(10)