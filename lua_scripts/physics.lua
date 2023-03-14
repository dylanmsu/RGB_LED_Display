function setInterval(interval)
    while true do
        run()
        matrix.draw()
        delay(interval)
    end
end

w = 32
h = 32

--variables of the individual bodies
pos = {};             --position                   (vector)
velocity = {};        --velocity                   (vector)
color = {};           --color                      (color)
mass = {};            --mass                       (float)
radius = {};          --radius                     (float)

--variables you can play with:
gravity = {0, 1};     --direction of gravity
numBodies = 9;       --the number of bodies

for i = 1, numBodies do
    pos[i] = {0, 0};
    velocity[i] = {0, 0};
    color[i] = {0, 0, 0};
    mass[i] = 1;
    radius[i] = 1;
end

function makeColGrad(freq, i)
    local center = 128;
    local width = 127;
                    
    local red = math.sin((freq*i + (2*math.pi)/3*1)) * width + center;
    local grn = math.sin((freq*i + (2*math.pi)/3*2)) * width + center;
    local blu = math.sin((freq*i + (2*math.pi)/3*3)) * width + center;
    return {math.floor(red),math.floor(grn),math.floor(blu)};
end

function initialize()
    for i=1,numBodies do
        radius[i] = 3;
        mass[i] = 1;
        velocity[i] = {0.01,-1}; --set start velocity
        
        pos[i] = {((i-1)%3)*10,((i-1)//3)*10}; --set start position
        color[i] = makeColGrad((2*math.pi)/numBodies, i);
    end
end

function collidedwall(x,y,radius)
    
    local normal = {}
    
    local up = -(x-radius);
    local dow = -((y+radius)-h);
    local lef = -(y-radius);
    local rig = -((x+radius)-w);
    
    if(x-radius<0) then normal = {up, 0} end
    if(y-radius<0) then normal = {0,lef} end
    if(x+radius>w) then normal = {rig,0} end
    if(y+radius>h) then normal = {0,dow} end
    
    if(x-radius<0 and y-radius<0) then normal = {up, lef} end
    if(x+radius>w and y-radius<0) then normal = {rig,lef} end
    if(x+radius>w and y+radius>h) then normal = {rig,dow} end
    if(x-radius<0 and y+radius>h) then normal = {up, dow} end
    
    local col = x<radius or y<radius or x+radius>w or y+radius>h;
    
    return {col,normal};
end

function normalize(vec) 
    local x = vec[1];
    local y = vec[2];
    local len = math.sqrt(x*x+y*y);
    return {x/len,y/len}
end


--multiplies a vector by a value
function multiply(vec,value)
    return {vec[1]*value,vec[2]*value}
end

--multiplies a vector by a vector
function mult(vec1,vec2)
    return {vec1[1]*vec2[1],vec1[2]*vec2[2]}
end

--substracts two vectors
function sub(a,b)
    return {a[1]-b[1],a[2]-b[2]}
end

--returns the dot product of itself
function pyth(a)
    return a[1]*a[1]+a[2]*a[2]
end

--gets the reflection vector
function reflect(d,n)
    return {2*dot(d,n)*n[1]-d[1], 2*dot(d,n)*n[2]-d[2]}
end

--returns the negative vector
function neg(vec)
    return {-vec[1],-vec[2]}
end

--returns the dot product of the two vectors
function dot(a,b)
    return a[1]*b[1]+a[2]*b[2]
end

function filledCircle(x, y, radius, r, g, b)
    for yi=-radius, radius do
        for xi=-radius, radius do
            --if ((x + xi)*(x + xi)+(y + yi)*(y + yi) <= radius*radius) then
            if (xi*xi+yi*yi <= radius*radius) then
                matrix.drawPixel(math.floor(x+xi+0.5), math.floor(y+yi+0.5), r, g, b)
            end
        end
    end
end

function physics()
    for i=1,numBodies do
        local x = pos[i][1];
        local y = pos[i][2];
        local colliding = collidedwall(x,y,radius[i]);
        local valu = 0.90;--restitution
        
        if (colliding[1] and dot(normalize(colliding[2]),normalize(velocity[i]))<0) then
            
            --reflect off the wall if it collides with it
            velocity[i] = multiply(reflect(neg(velocity[i]),normalize(colliding[2])),valu);
            velocity[i][1] = velocity[i][1] + (colliding[2][1]/4);
            velocity[i][2] = velocity[i][2] + (colliding[2][2]/4);
        end
        
        for k=1, numBodies do
            
            local pp = {x-pos[k][1],y-pos[k][2]}; 
            local np = neg(pp);
            local r = radius[i]+radius[k];
            local m = mass[i]+mass[k];
            local dist2 = pyth(pp);
            local collided = dist2 < r*r;  
            
            if (collided and pos[k] ~= pos[i]) then
                
                --computes velocities after collison
                --https://en.wikipedia.org/wiki/Elastic_collision
                local v1 = sub(velocity[i],multiply(pp,((2*mass[k])/m)*(dot(sub(velocity[i],velocity[k]),pp)/pyth(pp))));
                local v2 = sub(velocity[k],multiply(np,((2*mass[i])/m)*(dot(sub(velocity[k],velocity[i]),np)/pyth(np))));
                velocity[i] = multiply(v1,valu);
                velocity[k] = multiply(v2,valu);
                
                local dist = math.sqrt(dist2);--computes the distance between the two bodies
                local n = normalize(pp);--gets the normal of the collision point
                velocity[i][1] = velocity[i][1] - n[1]*(dist-r)/4;--push them away if they are inside each other
                velocity[i][2] = velocity[i][2] - n[2]*(dist-r)/4;
            end
        end
        
        local grav = multiply(gravity,1);
        
        velocity[i][1] = velocity[i][1] + grav[1]/15;--adds gravity
        velocity[i][2] = velocity[i][2] + grav[2]/15;
         
        pos[i][1] = pos[i][1] + velocity[i][1];--adds the velocity to the position of the body
        pos[i][2] = pos[i][2] + velocity[i][2];

        filledCircle(pos[i][1], pos[i][2], radius[i], color[i][1], color[i][2], color[i][3])
        
        --drawPixel(math.floor(pos[i][1]), math.floor(pos[i][2]), color[i][1], color[i][2], color[i][3])
    end
end

matrix.setBrightness(4)

initialize()

function run()
    matrix.clearScreen()

    physics()

    --matrix.draw()
    --delay(100)
    return 0
end

setInterval(10)