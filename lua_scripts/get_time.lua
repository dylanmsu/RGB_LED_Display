while (true) do
    seconds = getSeconds()
    minutes = getMinutes()
    hours = getHours()
    matrix.drawPixel(seconds, 0, 255, 0, 0)
    matrix.drawPixel(minutes, 1, 0, 255, 0)
    matrix.drawPixel(hours, 2, 0, 0, 255)
end