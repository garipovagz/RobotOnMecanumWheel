function sysCall_init()
    base = sim.getObject('.')
    wheelJoints={-1,-1,-1,-1} 
    wheelJoints[1]=sim.getObject('./rollingJoint_fl')
    wheelJoints[2]=sim.getObject('./rollingJoint_rl')
    wheelJoints[3]=sim.getObject('./rollingJoint_fr')
    wheelJoints[4]=sim.getObject('./rollingJoint_rr')
    robotTrace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,500000,{1,1,0})
    robot=sim.getObject('./Reference')
    init=sim.getObject('./StartPosition')
    vehicleTarget=sim.getObject('./TargetPosition') 
    sim.setObjectParent(vehicleTarget,-1,true)
    sim.setObjectParent(init,-1,true)
    sim.setObjectOrientation(vehicleTarget,sim.handle_parent,{0,0,0})
    sensor = sim.getObject('./Proximity_sensor')
    fr = sim.getObject('./fr')
    fl = sim.getObject('./fl')
    br = sim.getObject('./br')
    bl = sim.getObject('./bl')
    
    m = sim.getShapeMass(base)
    wheel_radius = 0.0475
    L = 0.2335
    l = 0.15
    D = 0.25
    Jw = 6*10^-3
    
    initPos=sim.getObjectPosition(init,robot)
    goalPos=sim.getObjectPosition(vehicleTarget,robot)
    max_dist = 0.5
    max_dist_2 = 0.5
    min_dist = 0.3
    dObject = 0.5 
    direction = -1
    state = 0
    pose = {0,0,0}
    turnTime = 3
    t = 2
    
    A = {0,0,0}   
    dA = 0
    

    kp = 0.6*0.01 --0.006
    ki = 2*kp / 0.1 --T = 0.1 => 0.12
    kd = kp*0.1/8 -- 0.75*10^-4

    sumErrors = {0, 0, 0, 0}
    
    prevErrors = {0, 0, 0, 0}
    
    pid = {0,0,0,0}
    w_ex = {}
    
    vel = 0
    vel_r = 3
    dt = 0
    
    gyro = {0,0,0}
    error = 0
   
     rotVel = 0
     forwBackVel=0
     leftRightVel=0
     
     pointing = false

    f = 0
   
    first = true
    print ("Enter x: ")
    local x = tonumber(io.read())
    print ("Enter y: ")
    local y = tonumber(io.read())
    print ("Enter z: ")
    local z = tonumber(io.read())

    if  x and  y and  z then
        print("You entered the coordinates:", x, y, z)
    else
        print("Incorrect input.")
    end
        
    sim.setObjectPosition(vehicleTarget,sim.handle_parent,{x,y,z}) 
   
   
    graph=sim.getObject('/Graph')
    ustavka=sim.addGraphStream(graph,'u','t', 0,{0,1,1})
    res=sim.addGraphStream(graph,'res', 't', 0,{1,0,0})
    start = sim.getSimulationTime()
end

function sysCall_sensing()
     local p=sim.getObjectPosition(base,-1)
    sim.addDrawingObjectItem(robotTrace,p)

    data = sim.readCustomDataBlock(base, 'g_data')
    if (data ~= nil) then
        gyro = sim.unpackTable(data)
     
    end
     data=sim.readCustomDataBlock(base,'dt')
        if (data ~= nil) then
            dt=sim.unpackTable(data)[1]
            
        end 
    
    table.insert(w_ex, gyro[3])
    flag,dFront = getDistance(sensor, max_dist)
    flag1, dFR = getDistance(fr, max_dist_2)
    flag2, dFL = getDistance(fl, max_dist_2)
    flag3,dBR = getDistance(br, max_dist_2)
    flag4,dBL = getDistance(bl, max_dist_2)

    
    pose = getRobotPose()
     if first then 
        lastTime=sim.getSimulationTime()
        
    end 
    first = false 
    if (state == 0) then     
        local turnTime = 5
        local timeElapsed=((sim.getSimulationTime()-lastTime)>turnTime)
        if timeElapsed then
           state = 1
        end
    

    elseif (state == 1) then 
        pointing = PointingToGoal(pose, initPos, goalPos)
        if pointing then
           state = 2
        end
       
    elseif (state == 2) then
        pointing = PointingToGoal(pose, initPos, goalPos)
        goalReached = GoalReached(pose,goalPos)
        if (goalReached) then
            state = 5
        elseif not pointing then
            state = 1
           
        elseif dFront < min_dist then
            A =  getRobotPose()
            dA = math.sqrt((goalPos[2]-A[2])^2+(goalPos[1]-A[1])^2)
            lastTime=sim.getSimulationTime()
            direction = -1 
            state = 3
        end
     elseif (state == 3) then
        
        if (math.abs(math.deg(pose[3])) % 90 < 0.1) then
            state = 4
        end
    

     elseif (state == 4) then
          
            B = getRobotPose()
            local dB = math.sqrt((goalPos[2]-B[2])^2+(goalPos[1]-B[1])^2)
             
            if ( dA - dB > 1 or (flag1 == 0 and  flag3 == 0) or (flag2 == 0 and  flag4 == 0)) then
                state = 1
                
            end
            if dFront < min_dist then
                state = 3
            end
        
       end
       
       pid = PIDcontrol( )
       print(state)
end

function sysCall_actuation()

    if (state == 0) then     
         forwBackVel = 0
         leftRightVel =0 
         rotVel = -vel_r
         J = ident_J (w_ex)
         vel = 15.67 - 0.14*m - 1.83*J
    elseif (state == 1) then 
        forwBackVel,leftRightVel,rotVel = turnToGoal()
        
    elseif (state == 2) then
        forwBackVel = vel
        rotVel = 0
        leftRightVel = 0
     
    elseif (state == 3) then

       forwBackVel,leftRightVel,rotVel = turn()
        
       
    elseif (state == 4) then 
        
   forwBackVel,leftRightVel,rotVel = followWall(direction,dFL,dBL,dFR,dBR,dWallSide)
   
    elseif (state == 5) then
        print('Time' .. sim.getSimulationTime() - start)
        
    rotVel = 0
    forwBackVel = 0
    leftRightVel = 0 
    
     end 
     setMovement(forwBackVel,leftRightVel,rotVel)
     
end

 
function getRobotPose()
    local pose
    position=sim.getObjectPosition(robot,vehicleTarget)
    orientation=sim.getObjectOrientation(robot,vehicleTarget)
    pose={position[1],position[2],orientation[3]}
    return pose
end

function GoalReached(pose,goalPos)
    
    return math.abs(pose[1]) < 0.1 and math.abs(pose[2]) <0.1
end



function getDistance(sensor,max_dist)
    local detected, distance, flag
    detected,distance=sim.readProximitySensor(sensor)
    if (detected<1) then
        distance=max_dist
        flag = false
    else flag = true
    end
    return detected, distance
end



function followWall(direction,dFL,dRL,dFR,dRR,dWallSide)
    
    if (math.abs(math.deg(pose[3])) % 90 < 0.001 ) then 
        rotVel = 0
        forwBackVel = vel
        leftRightVel = 0
        
    else
       
        if ((flag1 == 0 and  flag3 == 1) or (flag2 == 0 and  flag4 == 1) ) then
                rotVel = - direction * vel_r
                forwBackVel =  vel
                leftRightVel = 0
          else 
            rotVel = direction * vel_r
            forwBackVel = 0
            leftRightVel = 0
        end
    end
    
    return forwBackVel,leftRightVel,rotVel
end


function turn() 
    forwBackVel = 0
    leftRightVel = 0
    rotVel = direction *  vel_r
    
    return forwBackVel,leftRightVel,rotVel
end


function PointingToGoal(pose,initPos,goalPos)
    
    local angle =  math.pi/2 + math.atan2(pose[2],pose[1])
    if angle > math.pi then
        angle = angle - 2*math.pi
    elseif angle < -math.pi then
        angle = angle + 2*math.pi
    end
    if (math.abs(angle - pose[3]) > 0.1) then
        
        pointing = false
    else
        
        pointing = true
    end
    return pointing
end


function turnToGoal()
    local angle =  math.pi/2 + math.atan2(pose[2],pose[1])
    if angle > math.pi then
        angle = angle - 2*math.pi
    elseif angle < -math.pi then
        angle = angle + 2*math.pi
    end
    if (math.abs(angle - pose[3]) > 0.01) then
        rotVel = -  vel_r
        
    else
        rotVel = 0
       
    end
    forwBackVel = 0
    leftRightVel =0 
    return  forwBackVel,leftRightVel,rotVel
end



function selectRandom()
    local n,direction
    n=math.random(0,1)
    if (n<0.5) then
        direction=1
    else
        direction=-1
    end
    return direction
end


function PIDcontrol( )
    targetVelocities = {
        -forwBackVel-leftRightVel-rotVel,
        -forwBackVel+leftRightVel-rotVel,
        -forwBackVel-leftRightVel+rotVel,
        -forwBackVel+leftRightVel+rotVel
    }
    wheelVelocities = {
        sim.getJointVelocity(wheelJoints[1]),
        sim.getJointVelocity(wheelJoints[2]),
        sim.getJointVelocity(wheelJoints[3]),
        sim.getJointVelocity(wheelJoints[4])
    
    } 
    
    sim.setGraphStreamValue(graph, ustavka , targetVelocities[1])
    sim.setGraphStreamValue(graph, res, wheelVelocities[1])

    local p, i, d
    
    local errors = {0,0,0,0}
    
    for j = 1,4 do
        errors[j] = targetVelocities[j] - wheelVelocities[j] 
    end
    
    local pid = {}
    
    for j = 1,4 do 
        p = kp * errors[j]
        
        sumErrors[j] = sumErrors[j] + errors[j]*dt
       
        i = ki * sumErrors[j]
        
        d =  kd * (errors[j] - prevErrors[j]) / dt
        
        prevErrors[j] = errors[j]
        
        pid[j] = p + i 
    end
     
    return pid
   
    
end 

function FunMin(T)

    local r = 0
    local z0 = 1.077
    local z = 1
    local t = 0.000000001
    local h = 0.1
     for j = 1, #w_ex do 
     
      z = z0 * math.exp(- (t / T))
      
      r = r + (w_ex[j] - z)^2
    
        t = t + h;
        
    end
    
    return r

end

function minimize( a, b, epsilon)
    local goldenRatio = (math.sqrt(5) - 1) / 2  
    local c = b - goldenRatio * (b - a)
    local d = a + goldenRatio * (b - a)
    
    while math.abs(c - d) > epsilon do
        if FunMin(c) < FunMin(d) then
            b = d
        else
            a = c
        end
        
        c = b - goldenRatio * (b - a)
        d = a + goldenRatio * (b - a)
    end

    
    return (b + a) / 2
end

function ident_J ()

    A = 0.1
    B = 0.9
    eps = 1e-6
    T = minimize(A,B,eps)    
    J = 4*(T*D - Jw)*(L+l)^2 / R^2
    return J
end


function setMovement(forwBackVel,leftRightVel,rotVel)
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel + pid[1])
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel + pid[2])
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel+ pid[3])
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel+ pid[4])
end
