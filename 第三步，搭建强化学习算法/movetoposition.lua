-- Require the necessary CoppeliaSim simulation library
sim = require 'sim'

-- Constants for motion parameters
local MOVE_STEP = 0.005
local MAX_BOUNDARY = 1
local MIN_BOUNDARY = -1
local movementCounter = 0
-- Variables related to the suction pad mechanism
local maxPullForce, maxShearForce, maxPeelTorque
local INITIAL_JOINT_POSITIONS = {}
local INITIAL_WORLD_POSITIONS = {}
-- Function to initialize parameters and handles
function initializeParameters()

    mainIkTask = sim.getIkGroupHandle('irb360_mainTask')
    ikModeTipDummy = sim.getObjectHandle('irb360_ikTip')
    suctionDummy = sim.getObjectHandle('irb360_axisTip')
    INITIAL_WORLD_POSITIONS = sim.getObjectPosition(suctionDummy, -1)
    -- Handles for FK and IK joint modes
    fkDrivingJoints = {
        sim.getObjectHandle('irb360_drivingJoint1'),
        sim.getObjectHandle('irb360_drivingJoint2'),
        sim.getObjectHandle('irb360_drivingJoint3'),
        sim.getObjectHandle('irb360_motor')
    }
    ikDrivingJoints = {
        sim.getObjectHandle('irb360_cartesianX'),
        sim.getObjectHandle('irb360_cartesianY'),
        sim.getObjectHandle('irb360_cartesianZ'),
        sim.getObjectHandle('irb360_motor')
    }

    -- Base object handle
    irb360Base = sim.getObjectAssociatedWithScript(sim.handle_self)

    -- Initialize dynamics parameters for joints
    local angVel = math.pi  -- 180 degrees per second
    local angAccel = 2 * math.pi  -- 360 degrees per second^2
    local angJerk = 10 * math.pi  -- 1800 degrees per second^3
    currentAngVel = {0, 0, 0, 0}
    currentAngAccel = {0, 0, 0, 0}
    maxAngVel = {angVel, angVel, angVel, angVel}
    maxAngAccel = {angAccel, angAccel, angAccel, angAccel}
    maxAngJerk = {angJerk, angJerk, angJerk, angJerk}
    targetAngVel = {0, 0, 0, 0}

    -- Linear dynamics parameters
    local linVel = 1  -- 2 meters per second
    local linAccel = 1.5  -- 3 meters per second^2
    local linJerk = 15  -- 30 meters per second^3
    currentLinVel = {0, 0, 0, 0}
    currentLinAccel = {0, 0, 0, 0}
    maxLinVel = {linVel, linVel, linVel, linVel}
    maxLinAccel = {linAccel, linAccel, linAccel, linAccel}
    maxLinJerk = {linJerk, linJerk, linJerk, linJerk}
    targetLinVel = {0, 0, 0, 0}

    -- Initial position of the tip dummy

    local INITIAL_POSITION = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    initialPosition =INITIAL_POSITION
    -- Initialization of suction pad components and parameters
    suctionPadSensor = sim.getObjectHandle('suctionPadSensor')
    suctionPadLoopClosureDummy1 = sim.getObjectHandle('suctionPadLoopClosureDummy1')
    suctionPadLoopClosureDummy2 = sim.getObjectHandle('suctionPadLoopClosureDummy2')
    suctionPad = sim.getObjectHandle('suctionPad')
    suctionPadLink = sim.getObjectHandle('suctionPadLink')

    local scriptHandle = sim.getScriptHandle('suctionPad')
    
    if scriptHandle ~= -1 then
        infiniteStrength = sim.getScriptSimulationParameter(scriptHandle, 'infiniteStrength')
        maxPullForce = sim.getScriptSimulationParameter(scriptHandle, 'maxPullForce')
        maxShearForce = sim.getScriptSimulationParameter(scriptHandle, 'maxShearForce')
        maxPeelTorque = sim.getScriptSimulationParameter(scriptHandle, 'maxPeelTorque')
        sim.setLinkDummy(suctionPadLoopClosureDummy1, -1)
        sim.setObjectParent(suctionPadLoopClosureDummy1, suctionPad, true)
        local m = sim.getObjectMatrix(suctionPadLoopClosureDummy2, -1)
        sim.setObjectMatrix(suctionPadLoopClosureDummy1, -1, m)
    else
        print("Error: suctionPad script handle is invalid.")
    end
    
    
    for i = 1, #fkDrivingJoints do
        table.insert(INITIAL_JOINT_POSITIONS, sim.getJointPosition(fkDrivingJoints[i]))
    end
    initialJointPosition = INITIAL_JOINT_POSITIONS
end

-- Set Forward Kinematics mode
setFkMode = function()
    sim.setIkElementProperties(mainIkTask, ikModeTipDummy, 0)
    for i = 1, 3 do
        sim.setJointMode(fkDrivingJoints[i], sim.jointmode_passive, 0)
    end
end

-- Set Inverse Kinematics mode
setIkMode = function()
    
    print(initialPosition)
    local p = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    for i = 1, 3 do
        sim.setJointPosition(ikDrivingJoints[i], p[i] - initialPosition[i])
    end
    sim.setIkElementProperties(mainIkTask, ikModeTipDummy, sim.ik_x_constraint + sim.ik_y_constraint + sim.ik_z_constraint)
    for i = 1, 3 do
        sim.setJointMode(fkDrivingJoints[i], sim.jointmode_ik, 0)
    end
end

-- Reset to initial position function
resetToInitialPosition = function()
    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints, -1, currentAngVel, currentAngAccel, maxAngVel, maxAngAccel, maxAngJerk, {0, 0, 0, 0}, targetAngVel)
    setIkMode()
end

-- Callback function after movement attempt
function moveCallback(result, currentPos)
    if result == -1 then
        print("IK solver failed. Movement cancelled.")
        sim.setObjectPosition(ikModeTipDummy, irb360Base, currentPos)
    end
end

-- Movement function

function move(direction, callback,steps)
    movementCounter  = movementCounter + 1
    if movementCounter % 100 == 0 then
        print(string.format("Movement counter reached 100 times. Current step count: %d", movementCounter))
    end
    local currentPos = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    local targetPos = {
        math.max(math.min(currentPos[1] + direction[1], MAX_BOUNDARY), MIN_BOUNDARY),
        math.max(math.min(currentPos[2] + direction[2], MAX_BOUNDARY), MIN_BOUNDARY),
        math.max(math.min(currentPos[3] + direction[3], MAX_BOUNDARY), MIN_BOUNDARY)
    }

    if targetPos[1] == currentPos[1] and targetPos[2] == currentPos[2] and targetPos[3] == currentPos[3] then
        print("Target position is out of bounds. Movement cancelled.")
        return
    end
    local distance = math.sqrt((targetPos[1] - currentPos[1])^2 + 
                               (targetPos[2] - currentPos[2])^2 + 
                               (targetPos[3] - currentPos[3])^2)
    --moveLinearInterpolation(currentPos, targetPos, 10)
    steps = steps or (distance > 0.1 and 50 or 20)
    cubicInterpolation(currentPos, targetPos, steps)
    --sim.setObjectPosition(ikModeTipDummy, irb360Base, targetPos)
    local result = sim.handleIkGroup(mainIkTask)
    callback(result, currentPos)
end

-- Control suction pad function
-- Movement function with target position
-- Movement function with target position
function moveToPosition(targetPos, steps)
    -- Debugging information
    print("Initial Target Position:", targetPos)
    print("Initial Steps:", steps)

    -- Check if parameters are swapped
    if type(targetPos) == "table" and #targetPos == 1 and type(targetPos[1]) == "number" and
       type(steps) == "table" and #steps == 3 and type(steps[1]) == "number" and type(steps[2]) == "number" and type(steps[3]) == "number" then
        -- Swap parameters
        local temp = targetPos
        targetPos = steps
        steps = temp[1]
        print("Swapped Target Position:", targetPos)
        print("Swapped Steps:", steps)
    end

    -- Validate parameters
    if type(targetPos) ~= "table" or #targetPos ~= 3 or type(steps) ~= "number" or steps <= 0 then
        print("Invalid parameters in moveToPosition")
        return false
    end

    -- Get current position
    local currentPos = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    if type(currentPos) ~= "table" or #currentPos ~= 3 then
        print("Failed to get current position")
        return false
    end

    cubicInterpolation(currentPos, targetPos, steps)
    return true
end
function controlSuctionPad(active)

    local scriptHandle = sim.getScriptHandle('suctionPad')
    if scriptHandle ~= -1 then
        sim.setScriptSimulationParameter(scriptHandle, 'active', tostring(active))
    else
        print("Error: suctionPad script handle is invalid.")
    end
end

-- Main thread function
function sysCall_threadmain()

    -- Initialize settings
    print("hello")
    initializeParameters()
    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints, -1, currentAngVel, currentAngAccel, maxAngVel, maxAngAccel, maxAngJerk, {0, 0, 0, 0}, targetAngVel)
    setIkMode()
    -- Main simulation loop
    while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
        --local message, data, data2 = sim.getSimulatorMessage()
        sim.switchThread()  -- Allow other scripts to run
    end
end
-- Cleanup function to be called on script termination
function sysCall_cleanup()
    -- Cleanup code can be added here if needed
end

-- Public API functions for external control
function checkIKStatus()

    local result = sim.handleIkGroup(mainIkTask)
    if result == 2 then
        print(result)
    end
    return 
        {result},           -- ints (IK??)
        {},                 -- floats (?)
        {},  -- strings (????)
        ''                  -- bytes
end
function moveBack()
    movementCounter = 0
    local zeroPos=INITIAL_WORLD_POSITIONS
    print("zeropos",zeroPos)
    local currentWorldPos = sim.getObjectPosition(suctionDummy, -1)
    print("currentWorldPos",currentWorldPos)
    local direction = {
        currentWorldPos[1] - zeroPos[1],
        currentWorldPos[2] - zeroPos[2],
        currentWorldPos[3] - zeroPos[3]
    }
    -- Normalize the direction vector if necessary
    --local magnitude = math.sqrt(direction[1]^2 + direction[2]^2 + direction[3]^2)    
    --if magnitude > 0 then
    --    direction = {direction[1] / magnitude, direction[2] / magnitude, direction[3] / magnitude}
    --end
    --print("direction",direction)
    -- Scale the direction by the move step
    --direction = {direction[1] * MOVE_STEP, direction[2] * MOVE_STEP, direction[3] * MOVE_STEP}
    --print("direction",direction)
    --move({0,0,direction[3]}, moveCallback)
    --resetToInitialPosition()
    move(direction, moveCallback,500)
    
    print("backward")
    return {},{},{},''
end
local lastSafePosition = nil


function moveBackToSafePosition()
    local ZeroPos=INITIAL_WORLD_POSITIONS
    print("Zeropos",ZeroPos)
    local CurrentWorldPos = sim.getObjectPosition(suctionDummy, -1)
    print("CurrentWorldPos",CurrentWorldPos)
    local direction = {
        CurrentWorldPos[1] - ZeroPos[1],
        CurrentWorldPos[2] - ZeroPos[2],
        CurrentWorldPos[3] - ZeroPos[3]
    }
    -- Normalize the direction vector if necessary
    local magnitude = math.sqrt(direction[1]^2 + direction[2]^2 + direction[3]^2)    
    if magnitude > 0 then
        direction = {direction[1] / magnitude, direction[2] / magnitude, direction[3] / magnitude}
    end
    --print("direction",direction)
    -- Scale the direction by the move step
    direction = {direction[1] * 0.1, direction[2] * 0.1, direction[3] * 0.1}
    --direction = {direction[1] * 0.01, direction[2] * MOVE_STEP, direction[3] * MOVE_STEP}
    --print("direction",direction)
    --move({0,0,direction[3]}, moveCallback)
    --resetToInitialPosition()
    move(direction, moveCallback)
    
    print("backward")
    return {},{},{},''
   
end
function moveUp()

    print("moveUp called")
    move({0, 0, MOVE_STEP}, moveCallback)
    print("up")
    return {},{},{},''
end

function moveDown()

    move({0, 0, -MOVE_STEP}, moveCallback)
    print("down")
    return {},{},{},''
end

function moveLeft()

    move({-MOVE_STEP, 0, 0}, moveCallback)
    print("left")
    return {},{},{},''
end

function moveRight()

    move({MOVE_STEP, 0, 0}, moveCallback)
    print("right")
    return {},{},{},''
end

function moveForward()

    move({0, -MOVE_STEP, 0}, moveCallback)
    print("forward")
    return {},{},{},''
end

function moveBackward()


    move({0, MOVE_STEP, 0}, moveCallback)
    print("backward")
    return {},{},{},''
end
function activateSuctionPad()

    controlSuctionPad(true)
    print("activate suction pad")
    return {},{},{},''
end

function deactivateSuctionPad()

    controlSuctionPad(false)
    print("deactivate suction pad")
    return {},{},{},''
end

function enableIkMode()

    setIkMode()
    return {0}
end

function disableIkMode()
    setFkMode()
    return {0}
end



function cubicInterpolation(currentPos, targetPos, steps)
    local newPos = {currentPos[1], currentPos[2], currentPos[3]}
    local delta = {
        targetPos[1] - currentPos[1],
        targetPos[2] - currentPos[2],
        targetPos[3] - currentPos[3]
    }

    for i = 1, steps do
        local t = i / steps
        local t3 = t * t * t
        newPos[1] = currentPos[1] + delta[1] * (3 * t3 - 2 * t * t)
        newPos[2] = currentPos[2] + delta[2] * (3 * t3 - 2 * t * t)
        newPos[3] = currentPos[3] + delta[3] * (3 * t3 - 2 * t * t)
        sim.setObjectPosition(ikModeTipDummy, irb360Base, newPos)
        --print(string.format("Step %d: Position = [%.3f, %.3f, %.3f]", i, newPos[1], newPos[2], newPos[3]))
        sim.switchThread()
    end
end