furnace_front_on = {
	{80,78,78,80,78,78,80,78,78,80,78,78,60,59,59,80,78,78,80,78,78,80,78,78,60,59,59,60,59,59,60,59,59,80,78,78,60,59,59,60,59,59,80,78,78,80,78,78},
	{80,78,78,104,104,104,104,104,104,119,119,119,119,119,119,104,104,104,119,119,119,104,104,104,119,119,119,119,119,119,104,104,104,119,119,119,104,104,104,119,119,119,93,91,91,80,78,78},
	{60,59,59,93,91,91,119,119,119,145,145,145,119,119,119,133,133,133,104,104,104,119,119,119,133,133,133,145,145,145,133,133,133,145,145,145,119,119,119,104,104,104,119,119,119,80,78,78},
	{80,78,78,104,104,104,133,133,133,133,133,133,60,59,59,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,60,59,59,133,133,133,119,119,119,104,104,104,80,78,78},
	{80,78,78,119,119,119,133,133,133,60,59,59,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,60,59,59,119,119,119,93,91,91,60,59,59},
	{60,59,59,119,119,119,104,104,104,17,17,17,17,17,17,17,17,17,33,33,33,33,33,33,33,33,33,33,33,33,17,17,17,17,17,17,17,17,17,145,145,145,93,91,91,60,59,59},
	{80,78,78,119,119,119,104,104,104,17,17,17,17,17,17,60,59,59,60,59,59,60,59,59,60,59,59,60,59,59,60,59,59,17,17,17,17,17,17,145,145,145,93,91,91,80,78,78},
	{60,59,59,119,119,119,145,145,145,168,168,168,168,168,168,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,168,168,168,168,168,168,145,145,145,119,119,119,60,59,59},
	{60,59,59,119,119,119,133,133,133,119,119,119,104,104,104,93,91,91,119,119,119,104,104,104,93,91,91,104,104,104,119,119,119,104,104,104,119,119,119,104,104,104,104,104,104,80,78,78},
	{80,78,78,197,197,197,176,176,176,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,168,168,168,80,78,78},
	{80,78,78,168,168,168,168,168,168,168,168,168,176,176,176,168,168,168,133,133,133,133,133,133,133,133,133,133,133,133,168,168,168,176,176,176,168,168,168,176,176,176,157,157,157,60,59,59},
	{80,78,78,157,157,157,168,168,168,145,145,145,93,91,91,33,33,33,17,17,17,17,17,17,17,17,17,17,17,17,33,33,33,255,143,0,145,145,145,168,168,168,157,157,157,80,78,78},
	{80,78,78,145,145,145,145,145,145,80,78,78,255,143,0,17,17,17,195,93,27,255,143,0,255,143,0,195,93,27,255,216,0,255,216,0,80,78,78,119,119,119,157,157,157,80,78,78},
	{80,78,78,119,119,119,145,145,145,17,17,17,255,216,0,255,216,0,255,143,0,255,255,151,255,216,0,255,216,0,255,255,151,255,143,0,17,17,17,157,157,157,145,145,145,80,78,78},
	{60,59,59,145,145,145,119,119,119,195,93,27,255,216,0,255,255,151,255,216,0,255,255,255,255,216,0,255,255,255,255,255,151,255,216,0,195,93,27,119,119,119,157,157,157,60,59,59},
	{60,59,59,104,104,104,60,59,59,255,143,0,255,143,0,255,255,255,255,255,151,255,255,255,255,143,0,255,255,151,255,216,0,255,143,0,255,216,0,60,59,59,104,104,104,60,59,59}
}

for j=0,15 do
	for i=0,15 do
		red = furnace_front_on[j+1][3*i+1]
		grn = furnace_front_on[j+1][3*i+2]
		blu = furnace_front_on[j+1][3*i+3]
		matrix.drawPixel(i*2,   j*2,   red,grn,blu)
		matrix.drawPixel(i*2+1, j*2,   red,grn,blu)
		matrix.drawPixel(i*2,   j*2+1, red,grn,blu)
		matrix.drawPixel(i*2+1, j*2+1, red,grn,blu)
	end
end
