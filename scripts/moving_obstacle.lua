function sysCall_init()
    objectHandle = sim.getObject("/Cylinder[0]") 
    math.randomseed(sim.getSystemTimeInMs(-1)) 
    moveInterval = 2 
    nextMoveTime = sim.getSimulationTime() + moveInterval
    velocity = {0, 0}
    speed = 1 
end

function getRandomDirection()
    local angle = math.random() * 2 * math.pi
    return {math.cos(angle) * speed, math.sin(angle) * speed}
end

function sysCall_actuation()
    local timeNow = sim.getSimulationTime()
    if timeNow >= nextMoveTime then
        velocity = getRandomDirection()
        nextMoveTime = timeNow + moveInterval
    end

    local pos = sim.getObjectPosition(objectHandle, -1)
    pos[1] = pos[1] + velocity[1] * sim.getSimulationTimeStep()
    pos[2] = pos[2] + velocity[2] * sim.getSimulationTimeStep()
    sim.setObjectPosition(objectHandle, -1, pos)
end
