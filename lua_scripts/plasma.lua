time_counter = 0
cycles = 0
fps = 0

function run()
    for x=0,31 do
        for y=0,31 do
            v = 0;
            wibble = sin8(time_counter)
            v = v + sin16(x * wibble * 3 + time_counter);
            v = v + sin16(y * (128 - wibble)  + time_counter + 16384);
            v = v + sin16(y * x * sin8(-time_counter + 64) / 8);
            matrix.drawPixelHSV(x, y, math.floor((v/258.0) + 127)%360, 100, 100);
        end
    end
    matrix.draw()
    time_counter = time_counter + 1;
    cycles = cycles + 1;
    fps = fps + 1;
end

setInterval(10)