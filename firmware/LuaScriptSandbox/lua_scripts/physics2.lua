function setInterval(interval)
    while true do
        run()
        matrix.draw()
        delay(interval)
    end
end

w = 32
h = 32
circle_points = 8
pixels_per_meter = 1/0.006
EPSILON = 0.0001;

--variables of the individual bodies
pos = {}
velocity = {}

angularVelocity = {}
torque = {}
orient = {}

force = {}

inertia = {}
inverseInertia = {}
mass = {}
inverseMass = {}

staticFriction = {}
dynamicFriction = {}
restitution = {}

radius = {}
color = {}

--
gravity = {0, 0}
numBodies = 0

--t = 0

function initialize()
    gravity = {0, 9.81}
    numBodies = 7
    density = 1
    for i=1,numBodies do
        velocity[i] = {1, 1}

        angularVelocity[i] = 0
        torque[i] = 0
        orient[i] = 0

        force[i] = {0, 0}

        radius[i] = 4

        mass[i] = math.pi * radius[i] * radius[i] * density;
        if (mass[i] ~= 0) then
            inverseMass[i] = 1.0 / mass[i]
        else
            inverseMass[i] = 0.0
        end

        inertia[i] = mass[i] * radius[i] * radius[i];
        if (inertia[i] ~= 0) then
            inverseInertia[i] = 1.0 / inertia[i]
        else
            inverseInertia[i] = 0.0
        end

        staticFriction[i] = 0.2;
        dynamicFriction[i] = 0.2;
        restitution[i] = 0.5;

        pos[i] = {
            radius[i] + ((i-1)%4)*radius[i]*2,
            radius[i] + ((i-1)//4)*radius[i]*2
        }
        color[i] = makeColGrad((2*math.pi)/numBodies, i)
    end
end


function Circle(x, y, radius, rotation, r, g, b)
    local prevX = 0
    local prevY = 0
    for i=0,circle_points do
        local xr = radius*math.cos(((2*math.pi)/circle_points)*i + rotation)
        local yr = radius*math.sin(((2*math.pi)/circle_points)*i + rotation)
        matrix.drawLineWu(x + prevX, y + prevY, x + xr, y + yr, r, g, b)
        prevX = xr
        prevY = yr
    end
end

function makeColGrad(freq, i)
    local center = 128
    local width = 127
                    
    local red = math.sin((freq*i + (2*math.pi)/3*1)) * width + center
    local grn = math.sin((freq*i + (2*math.pi)/3*2)) * width + center
    local blu = math.sin((freq*i + (2*math.pi)/3*3)) * width + center
    return {math.floor(red),math.floor(grn),math.floor(blu)}
end

function cross_vs( vec_a, a )
    return {a * vec_a[2], -a * vec_a[1]};
end

function cross_sv( a, vec_a )
    return {-a * vec_a[2], a * vec_a[1]};
end

function cross_vv( vec_a, vec_b )
    return vec_a[1] * vec_b[2] - vec_a[2] * vec_b[1];
end

function dot( vec_a, vec_b)
    return vec_a[1]*vec_b[1]+vec_a[2]*vec_b[2]
end

function equal( a, b )
  -- <= instead of < for NaN comparison safety
  return math.abs( a - b ) <= EPSILON;
end

function integrateForces( body_index, dt )
    if (inverseMass[body_index] == 0.0) then
        return;
    end

    velocity[body_index][1] = velocity[body_index][1] + (force[body_index][1] * inverseMass[body_index] + gravity[1]) * (dt / 2.0);
    velocity[body_index][2] = velocity[body_index][2] + (force[body_index][2] * inverseMass[body_index] + gravity[2]) * (dt / 2.0);

    angularVelocity[body_index] = angularVelocity[body_index] + torque[body_index] * inverseInertia[body_index] * (dt / 2.0);
end

function integrateVelocity( body_index, dt )
    if (inverseMass[body_index] == 0.0) then
        return;
    end

    pos[body_index][1] = pos[body_index][1] + (velocity[body_index][1] * dt)*(1/0.006);
    pos[body_index][2] = pos[body_index][2] + (velocity[body_index][2] * dt)*(1/0.006);

    orient[body_index] = orient[body_index] + (angularVelocity[body_index] * dt);

    --b->SetOrient( b->orient );
    integrateForces( body_index, dt );
end

function applyImpulse( body_index, impulse, contactVector )
    velocity[body_index][1] = velocity[body_index][1] + (inverseMass[body_index] * impulse[1]);
    velocity[body_index][2] = velocity[body_index][2] + (inverseMass[body_index] * impulse[2]);
    angularVelocity[body_index] = angularVelocity[body_index] + inverseInertia[body_index] * cross_vv( contactVector, impulse );
end

function get_collision_vector(x,y,radius)
    local normal = {0, 0}
    local is_colliding = false
    
    local lef = x-radius
    local dow = h - (y+radius)
    local up = y-radius
    local rig = w - (x+radius)

    if(lef < 0) then normal = {lef, 0} end
    if(up < 0) then normal = {0,up} end
    if(rig < 0) then normal = {-rig,0} end
    if(dow < 0) then normal = {0,-dow} end

    if(lef < 0 and up < 0) then normal = {lef, up} end
    if(up < 0 and rig < 0) then normal = {-rig,up} end
    if(rig < 0 and dow < 0) then normal = {-rig,-dow} end
    if(dow < 0 and lef < 0) then normal = {lef, -dow} end
    
    return normal
end

function normalize(vec) 
    local x = vec[1]
    local y = vec[2]
    local len = math.sqrt(x*x+y*y)
    return {x/len,y/len}
end

function applyimpulses(index_a, index_b, contactVectors, contact_count, normal, penetration)
    -- Early out and positional correct if both objects have infinite mass
    if(equal( inverseMass[index_a] + inverseMass[index_b], 0 )) then
      velocity[index_a] = {0, 0}
      velocity[index_b] = {0, 0}
      return;
    end

    --print("hello")

    local e = math.min( restitution[index_a], restitution[index_b] );
    local df = math.sqrt( dynamicFriction[index_a] * dynamicFriction[index_a] );
    local sf = math.sqrt( staticFriction[index_a] * staticFriction[index_a] );

    for i=1,contact_count do
        -- Calculate radii from COM to contact
        local ra = {}
        ra[1] = contactVectors[i][1] - pos[index_a][1]
        ra[2] = contactVectors[i][2] - pos[index_a][2]

        local rb = {}
        rb[1] = contactVectors[i][1] - pos[index_b][1]
        rb[2] = contactVectors[i][2] - pos[index_b][2]

        -- Relative velocity
        local rv = {}
        rv[1] = velocity[index_b][1] + cross_sv( angularVelocity[index_b], rb )[1] - velocity[index_a][1] - cross_sv( angularVelocity[index_a], ra )[1];
        rv[2] = velocity[index_b][2] + cross_sv( angularVelocity[index_b], rb )[2] - velocity[index_a][2] - cross_sv( angularVelocity[index_a], ra )[2];

        --print("rv[" .. velocity[index_b][1] .. ", " .. velocity[index_b][2] .. "]")

        -- Relative velocity along the normal
        local contactVel = dot( rv, normal );

        -- Do not resolve if velocities are separating
        if (contactVel > 0) then
            return
        end

        local raCrossN = cross_vv( ra, normal );
        local rbCrossN = cross_vv( rb, normal );
        local invMassSum = inverseMass[index_a] + inverseMass[index_b] + (raCrossN*raCrossN) * inverseInertia[index_a] + ( rbCrossN*rbCrossN ) * inverseInertia[index_b];

        -- Calculate impulse scalar
        local j = ((-(1.0 + e) * contactVel)/invMassSum)/contact_count;

        -- Apply impulse
        local impulse_p = {}
        local impulse_n = {}
        impulse_p[1] = normal[1] * j
        impulse_p[2] = normal[2] * j
        impulse_n[1] = - normal[1] * j
        impulse_n[2] = - normal[2] * j
        applyImpulse( index_a, impulse_n, ra );
        applyImpulse( index_b, impulse_p, rb );

        -- Friction impulse
        --local rv = {}
        --rv[1] = velocity[index_b][1] + cross_sv( angularVelocity[index_b], rb )[1] - velocity[index_a][1] - cross_sv( angularVelocity[index_a], ra )[1];
        --rv[2] = velocity[index_b][2] + cross_sv( angularVelocity[index_b], rb )[2] - velocity[index_a][2] - cross_sv( angularVelocity[index_a], ra )[2];
--
        --local t = {}
        --t[1] = rv[1] - (normal[1] * dot( rv, normal ));
        --t[2] = rv[2] - (normal[2] * dot( rv, normal ));
        --t = normalize(t)
--
        ---- j tangent magnitude
        --local jt = ((-dot( rv, t ))/invMassSum)/contact_count;
--
        ---- Don't apply tiny friction impulses
        --if(equal( jt, 0.0 )) then
        --    return;
        --end
--
        ---- Coulumb's law
        --local tangentImpulse_p = {};
        --local tangentImpulse_n = {};
        --if (math.abs( jt ) < j * sf) then
        --    tangentImpulse_p[1] = t[1] * jt;
        --    tangentImpulse_p[2] = t[2] * jt;
        --    tangentImpulse_n[1] = - t[1] * jt;
        --    tangentImpulse_n[2] = - t[2] * jt;
        --else
        --    tangentImpulse_p[1] = t[1] * -j * df;
        --    tangentImpulse_p[2] = t[2] * -j * df;
        --    tangentImpulse_n[1] = - (t[1] * -j * df);
        --    tangentImpulse_n[2] = - (t[2] * -j * df);
        --end

        -- Apply friction impulse
        --applyImpulse( index_a, tangentImpulse_n, ra );
        --applyImpulse( index_b, tangentImpulse_p, rb );
    end
end

function positionalCorrection( index_a, index_b, normal, penetration )
    local k_slop = 0.05; -- Penetration allowance
    local percent = 0.4; -- Penetration percentage to correct
    local correction = {}
    correction[1] = (math.max( penetration - k_slop, 0.0 ) / (inverseMass[index_a] + inverseMass[index_b])) * normal[1] * percent;
    correction[2] = (math.max( penetration - k_slop, 0.0 ) / (inverseMass[index_a] + inverseMass[index_b])) * normal[2] * percent;
    pos[index_a][1] = pos[index_a][1] - correction[1] * inverseMass[index_a];
    pos[index_a][2] = pos[index_a][2] - correction[2] * inverseMass[index_a];

    pos[index_a][1] = pos[index_a][1] + correction[1] * inverseMass[index_b];
    pos[index_a][2] = pos[index_a][2] + correction[2] * inverseMass[index_b];
end

function physics(dt)

    for i=1,numBodies do
        for k=1, numBodies do
            local pp = {pos[k][1]-pos[i][1],pos[k][2]-pos[i][2]};
            local r = radius[i]+radius[k];
            local dist2 = pp[1]*pp[1] + pp[2]*pp[2];
            local collided = dist2 < r*r;  
            
            if (collided and pos[k] ~= pos[i]) then

                local dist = math.sqrt(dist2);

                local contacts = {}
                local normal = {}
                local penetration = 0

                contact_count = 1;

                contacts[1] = {}

                if (dist == 0.0) then
                    penetration = radius[i];
                    normal = { 1, 0 };
                    contacts[1] = pos[i];
                else
                    penetration = r - dist;
                    normal[1] = (pp[1] / dist);
                    normal[2] = (pp[2] / dist);
                    contacts[1][1] = normal[1] * radius[i] + pos[i][1];
                    contacts[1][2] = normal[2] * radius[i] + pos[i][2];
                end

                --applyImpulse( i, {-normal[1], -normal[2]}, {normal[1], normal[2]} )
                --applyImpulse( k, {normal[1], normal[2]}, {-normal[1], -normal[2]} )
                applyimpulses(i, k, contacts, contact_count, normal, penetration)

                --pos[i][1] = pos[i][1] - normal[1]*penetration*0.5
                --pos[i][2] = pos[i][2] - normal[2]*penetration*0.5
                --pos[k][1] = pos[k][1] + normal[1]*penetration*0.5
                --pos[k][2] = pos[k][2] + normal[2]*penetration*0.5
            end
        end
    end
    for i=1,numBodies do
        integrateVelocity(i, dt)
    end

    for i=1,numBodies do
        integrateForces(i, dt)
    end

    

    for i=1,numBodies do
        if (pos[i][1]<radius[i] or pos[i][2]<radius[i] or (pos[i][1]+radius[i])>=w or (pos[i][2]+radius[i])>=h) then
            
            local collision_vector = get_collision_vector(pos[i][1],pos[i][2],radius[i])
            local normal_col_v = normalize(collision_vector)

            local contacts = {}
            contacts[1] = normal_col_v[1] * radius[i] + pos[i][1];
            contacts[2] = normal_col_v[2] * radius[i] + pos[i][2];

            local ra = {}
            ra[1] = (contacts[1] - pos[i][1])
            ra[2] = (contacts[2] - pos[i][2])

            -- Calculate impulse scalar
            local raCrossN = cross_vv( ra, normal_col_v );
            local invMassSum = inverseMass[i] + 0 + (raCrossN*raCrossN) * inverseInertia[i] + 0;
    
            local velocitymag = math.sqrt(velocity[i][1]*velocity[i][1]+velocity[i][2]*velocity[i][2])
            local j = (-(1.0 + restitution[i]) * velocitymag)/invMassSum;
    
            -- Apply impulse
            local impulse_p = {}
            local impulse_n = {}
            impulse_p[1] = normal_col_v[1] * j
            impulse_p[2] = normal_col_v[2] * j
            applyImpulse( i, impulse_p, ra );
            
            --pos[i][1] = pos[i][1] - collision_vector[1]
            --pos[i][2] = pos[i][2] - collision_vector[2]
        end
    end
end

initialize()
delta = 10 --ms
substeps = 4
elapsed = 0
matrix.setBrightness(255)

function run()
    matrix.clearScreen()
    --gravity = {math.cos(elapsed/1000)*9.81,math.sin(elapsed/1000)*9.81}

    for i=1,substeps do
        physics((delta/1000)/substeps)
    end

    for i=1,numBodies do
        pos[i][1] = math.min(math.max(pos[i][1], -radius[i]), w + radius[i])
        pos[i][2] = math.min(math.max(pos[i][2], -radius[i]), h + radius[i])
        Circle(pos[i][1], pos[i][2], radius[i], orient[i], color[i][1], color[i][2], color[i][3])
    end

    elapsed = elapsed + delta

    --matrix.draw()
    --delay(100)
    return 0
end

setInterval(delta)