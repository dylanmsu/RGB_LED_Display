function setInterval(interval)
    while true do
        run()
        matrix.draw()
        delay(interval)
    end
end

w = 32
h = 32

pos = {}; --position of the particles 
velocity = {}; --defines the direction in witch the particle moves and its speed
destroyed = {}; 
color = {};
mass = {};

numBodies = 0; --the number of particles
blhole = {w/2,h/2}; --position of the black hole

numdestroyed = 0;

for i = 1, numBodies do
    pos[i] = {0, 0};
    velocity[i] = {0, 0};
    destroyed[i] = false; 
    color[i] = {0, 0, 0};
    mass[i] = 1;
end

function makeColGrad(freq, i)
    local center = 128;
    local width = 127;
                    
    local red = math.sin((freq*i + (2*math.pi)/3*1)) * width + center;
    local grn = math.sin((freq*i + (2*math.pi)/3*2)) * width + center;
    local blu = math.sin((freq*i + (2*math.pi)/3*3)) * width + center;
    return {math.floor(red),math.floor(grn),math.floor(blu)};
end

function normalize(vec)
    local x = vec[1];
    local y = vec[2];
    local len = math.sqrt(x*x+y*y);
    return {x/len,y/len};
end

function initialize()
    grid_width = 10
    numBodies = grid_width*grid_width
    numdestroyed = 0;
    startpos = {
        math.random(0,32 - grid_width),
        math.random(0,32 - grid_width)
    }
    for i=1,numBodies do
        x = ((i - 1)%grid_width) + startpos[1]
        y = ((i - 1)//grid_width) + startpos[2]

        if (x == w/2 and y == h/2) then
            velocity[i] = {0, 0}
        else
            --vector tangent to the position of the black hole: (x, y) = (y, -x)
            velocity[i] = normalize({(y - w/2), -(x - w/2)})
            velocity[i][1] = velocity[i][1]/3
            velocity[i][2] = velocity[i][2]/3
        end

        pos[i] = {x,y}
        color[i] = makeColGrad((2*math.pi)/numBodies, i);
        destroyed[i] = false;
        mass[i] = 1;
    end
end

function physics()
    -- iterate over every particle 
    for i=1, numBodies do
        if (destroyed[i] == false) then

            local p = {pos[i][1]-blhole[1],pos[i][2]-blhole[2]};
            local distance_squared = p[1]*p[1]+p[2]*p[2]

            -- find the normal vector pointing from the black hole to the particle
            local np = normalize(p)
            
            -- apply gravity from the black hole
            local acelleration = {}
            if (p[1] == 0 and p[2] == 0) then
                acelleration[1] = 0
                acelleration[2] = 0
            else
                acelleration[1] = -np[1]/(distance_squared*mass[i])
                acelleration[2] = -np[2]/(distance_squared*mass[i])
            end
            
            -- integrate acceleration to get velocity
            velocity[i][1] = velocity[i][1] + acelleration[1]
            velocity[i][2] = velocity[i][2] + acelleration[2]
            
            -- apply air resistance
            velocity[i][1] = velocity[i][1]*0.999
            velocity[i][2] = velocity[i][2]*0.999
 
            -- integrate velocity to get the position
            pos[i][1] = pos[i][1] + velocity[i][1]
            pos[i][2] = pos[i][2] + velocity[i][2]

            -- draw particles
            matrix.drawPixel(math.floor(pos[i][1]), math.floor(pos[i][2]), color[i][1], color[i][2], color[i][3])
            
            -- delete particle if particle is inside black hole or is 100 pixels away from black hole
            if (distance_squared<1 or distance_squared>10000) then
                destroyed[i] = true
                numdestroyed = numdestroyed + 1;
            end
            
            -- if all bodies are deleted, reinitialize particles
            if (numdestroyed >= numBodies) then
                initialize()
            end
        end
    end
end

matrix.setBrightness(255)

initialize()

function run()
    matrix.clearScreen()

    physics()

    matrix.drawPixel(math.floor(blhole[1]-1), math.floor(blhole[2]-1), 64, 64, 64)
    matrix.drawPixel(math.floor(blhole[1]-1), math.floor(blhole[2]),   64, 64, 64)
    matrix.drawPixel(math.floor(blhole[1]),   math.floor(blhole[2]-1), 64, 64, 64)
    matrix.drawPixel(math.floor(blhole[1]),   math.floor(blhole[2]),   64, 64, 64)

    --matrix.draw()
    --delay(100)
    return 0
end

setInterval(10)