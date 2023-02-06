time_counter = 0
cycles = 0
fps = 0
while (true) do
    for x=0,31 do
        for y=0,31 do
            v = 0;
            wibble = ((math.sin(time_counter*((2*math.pi)/256.0)) * 128.0) + 128);
            v = v + math.sin((x * wibble * 3 + time_counter)*((2*math.pi)/32768.0)) * 32767.0;
            v = v + math.cos((y * (128 - wibble)  + time_counter)*((2*math.pi)/32768.0)) * 32767.0;
            v = v + math.sin((y * x * ((math.cos(-time_counter*((2*math.pi)/256.0)) * 128.0) + 128) / 8)*((2*math.pi)/32768.0)) * 32767.0;

            drawPixelHSV(x, y, math.floor((v/258.0) + 127)%360, 100, 100);
        end
    end
    time_counter = time_counter + 1;
    cycles = cycles + 1;
    fps = fps + 1;
end