from PIL import Image
from numpy import asarray
from pathlib import Path

image = Image.open('./python/img/minecraft/textures/block/nether_portal.png')
data = asarray(image)
name = Path(image.filename).stem

row = str(name) + " = {\n"
for idx, r in enumerate(data):
    row += "\t{"
    first_pixel = True
    for index, pixel in enumerate(r):
        if index != len(r) - 1:
            row += str(pixel[0]) + ","
            row += str(pixel[1]) + ","
            row += str(pixel[2]) + ","
        else:
            row += str(pixel[0]) + ","
            row += str(pixel[1]) + ","
            row += str(pixel[2])

    if idx != len(data) - 1:
        row += "},\n"
    else:
        row += "}\n"
        row += "}\n\n"
        row += "for j=0,15 do\n"
        row += "\tfor i=0,15 do\n"
        row += "\t\tred = " + str(name) + "[j+1][3*i+1]\n"
        row += "\t\tgrn = " + str(name) + "[j+1][3*i+2]\n"
        row += "\t\tblu = " + str(name) + "[j+1][3*i+3]\n"
        row += "\t\tdrawPixel(i*2,   j*2,   red,grn,blu)\n"
        row += "\t\tdrawPixel(i*2+1, j*2,   red,grn,blu)\n"
        row += "\t\tdrawPixel(i*2,   j*2+1, red,grn,blu)\n"
        row += "\t\tdrawPixel(i*2+1, j*2+1, red,grn,blu)\n"
        row += "\tend\n"
        row += "end\n"
        
with open('./python/generated_scripts/' + str(name) + '.lua', 'w') as f:
    f.write(row)