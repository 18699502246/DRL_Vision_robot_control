-- Require the necessary CoppeliaSim simulation library
sim = require 'sim'

-- Constants for motion parameters
local MOVE_STEP = 0.01
local MAX_BOUNDARY = 1
local MIN_BOUNDARY = -1
local ANG_VEL = math.pi  -- Angular velocity
local ANG_ACCEL = 2 * math.pi  -- Angular acceleration
local ANG_JERK = 10 * math.pi  -- Angular jerk
local LIN_VEL = 2  -- Linear velocity
local LIN_ACCEL = 3  -- Linear acceleration
local LIN_JERK = 30  -- Linear jerk

-- Variables related to the suction pad mechanism
local suctionPadSensor
local suctionPadLoopClosureDummy1
local suctionPadLoopClosureDummy2
local suctionPad
local suctionPadLink
local infiniteStrength
local maxPullForce
local maxShearForce
local maxPeelTorque

local INITIAL_JOINT_POSITIONS = {}
fkDrivingJoints = {
        sim.getObjectHandle('irb360_drivingJoint1'),
        sim.getObjectHandle('irb360_drivingJoint2'),
        sim.getObjectHandle('irb360_drivingJoint3'),
        sim.getObjectHandle('irb360_motor')
    }
for i = 1, #fkDrivingJoints do
        table.insert(INITIAL_JOINT_POSITIONS, sim.getJointPosition(fkDrivingJoints[i]))
    end
local initialJointPosition = INITIAL_JOINT_POSITIONS
-- Function to initialize parameters and handles
function initializeParameters()
    mainIkTask = sim.getIkGroupHandle('irb360_mainTask')
    ikModeTipDummy = sim.getObjectHandle('irb360_ikTip')
    
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
    local linVel = 2  -- 2 meters per second
    local linAccel = 3  -- 3 meters per second^2
    local linJerk = 30  -- 30 meters per second^3
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
    
    
    --for i = 1, #fkDrivingJoints do
    --    table.insert(INITIAL_JOINT_POSITIONS, sim.getJointPosition(fkDrivingJoints[i]))
    --end
    --initialJointPosition = INITIAL_JOINT_POSITIONS
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
function move(direction, callback)
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

    sim.setObjectPosition(ikModeTipDummy, irb360Base, targetPos)
    local result = sim.handleIkGroup(mainIkTask)
    callback(result, currentPos)
end

function moveToPosition(x, y, z)
    local currentPos = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    local targetPos = {
        math.max(math.min(x, MAX_BOUNDARY), MIN_BOUNDARY),
        math.max(math.min(y, MAX_BOUNDARY), MIN_BOUNDARY),
        math.max(math.min(z, MAX_BOUNDARY), MIN_BOUNDARY)
    }

    -- Check if target position is within boundaries
    if targetPos[1] == currentPos[1] and targetPos[2] == currentPos[2] and targetPos[3] == currentPos[3] then
        print("Target position is out of bounds. Movement cancelled.")
        return "Movement cancelled: Target out of bounds"
    end

    -- Set target position
    sim.setObjectPosition(ikModeTipDummy, irb360Base, targetPos)

    -- Handle IK solving
    local result = sim.handleIkGroup(mainIkTask)
    if result == sim.ikresult_success then
        print(string.format("Moved to position: (%.2f, %.2f, %.2f)", targetPos[1], targetPos[2], targetPos[3]))
        return string.format("Moved to (%.2f, %.2f, %.2f)", targetPos[1], targetPos[2], targetPos[3])
    else
        print("IK solver failed. Movement cancelled.")
        -- If IK solving fails, revert to original position
        sim.setObjectPosition(ikModeTipDummy, irb360Base, currentPos)
        return "Movement failed: IK solver error"
    end
end

-- Control suction pad function
function controlSuctionPad(active)
    local scriptHandle = sim.getScriptHandle('suctionPad')
    if scriptHandle ~= -1 then
        sim.setScriptSimulationParameter(scriptHandle, 'active', tostring(active))
    else
        print("Error: suctionPad script handle is invalid.")
    end
end

-- System initialization call
function sysCall_init()
    
    initializeParameters()
    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints, -1, currentAngVel, currentAngAccel, maxAngVel, maxAngAccel, maxAngJerk, {0, 0, 0, 0}, targetAngVel)
    setIkMode()

        -- ? irb360 ???????
    sim.registerScriptFunction('irb360@moveUp', 'number moveUp()')
    sim.registerScriptFunction('irb360@moveDown', 'number moveDown()')
    sim.registerScriptFunction('irb360@moveLeft', 'number moveLeft()')
    sim.registerScriptFunction('irb360@moveRight', 'number moveRight()')
    sim.registerScriptFunction('irb360@moveForward', 'number moveForward()')
    sim.registerScriptFunction('irb360@moveBackward', 'number moveBackward()')
    sim.registerScriptFunction('irb360@resetPosition', 'number resetPosition()')
    sim.registerScriptFunction('irb360@activateSuctionPad', 'number activateSuctionPad()')
    sim.registerScriptFunction('irb360@deactivateSuctionPad', 'number deactivateSuctionPad()')
    sim.registerScriptFunction("enableIkMode@irb360", "string enableIkMode()")
    sim.registerScriptFunction("disableIkMode@irb360", "string disableIkMode()")
end


-- Main thread function
function sysCall_threadmain()
    -- Initialize settings
    print("Script initialized")
    initializeParameters()
    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints, -1, currentAngVel, currentAngAccel, maxAngVel, maxAngAccel, maxAngJerk, {0, 0, 0, 0}, targetAngVel)
    setIkMode()
    -- Main simulation loop
    while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
        local message, data, data2 = sim.getSimulatorMessage()
        if message == sim.message_keypress then
            if data[1] == 2007 then -- up key
                print("up")
                move({0, 0, MOVE_STEP}, moveCallback)
            elseif data[1] == 2008 then -- down key
                print("down")
                move({0, 0, -MOVE_STEP}, moveCallback)
            elseif data[1] == 2009 then -- left key
                print("left")
                move({-MOVE_STEP, 0, 0}, moveCallback)
            elseif data[1] == 2010 then -- right key
                print("right")
                move({MOVE_STEP, 0, 0}, moveCallback)
            elseif data[1] == string.byte('q') then -- q key
                print("forward")
                move({0, -MOVE_STEP, 0}, moveCallback)
            elseif data[1] == string.byte('w') then -- w key
                print("backward")
                move({0, MOVE_STEP, 0}, moveCallback)
            elseif data[1] == string.byte('r') then -- r key
                print("reset")
                resetToInitialPosition()
            elseif data[1] == string.byte('e') then -- r key
                print("test")
                resetPosition()
            elseif data[1] == string.byte('a') then -- a key
                print("activate suction pad")
                controlSuctionPad(true)
            elseif data[1] == string.byte('d') then -- d key
                print("deactivate suction pad")
                controlSuctionPad(false)

            end
        end
        sim.switchThread()  -- Allow other scripts to run
    end
end

-- Cleanup function to be called on script termination
function sysCall_cleanup()
    -- Cleanup code can be added here if needed
end

-- Public API functions for external control
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

function resetPosition()
    if initialJointPosition == nil then
        print("Error: Initial joint positions not set")
        return {},{},{},''
    end

    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints, -1, currentAngVel, currentAngAccel, maxAngVel, maxAngAccel, maxAngJerk, initialJointPosition, targetAngVel)
    sim.wait(2) -- ??2?,???????????

    setIkMode()
    local currentJointPositions = {}
    for i = 1, #fkDrivingJoints do
        table.insert(currentJointPositions, sim.getJointPosition(fkDrivingJoints[i]))
    end
    print("initialJointPosition",initialJointPosition)
    print("currentJointPositions",currentJointPositions)
    
    local isReset = arePositionsClose(currentJointPositions, initialJointPosition)
    if isReset then
        print("Successfully reset to initial joint positions")
    else
        print("Failed to reset to initial joint positions")
    end

    return {},{},{},''
end


function arePositionsClose(pos1, pos2, tolerance)
    tolerance = tolerance or 0.001
    for i = 1, #pos1 do
        if math.abs(pos1[i] - pos2[i]) > tolerance then
            return false
        end
    end
    return true
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
