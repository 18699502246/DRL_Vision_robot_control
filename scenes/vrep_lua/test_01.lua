-- Require the necessary CoppeliaSim simulation library
sim = require 'sim'

-- Constants for motion parameters
local MOVE_STEP = 0.01
local BOUNDARY = {MAX = 1, MIN = -1}  -- Boundary table
local ANG_VEL = math.pi
local ANG_ACCEL = 2 * math.pi
local ANG_JERK = 10 * math.pi
local LIN_VEL = 2
local LIN_ACCEL = 3
local LIN_JERK = 30

-- Variables related to the suction pad mechanism
local suctionPadComponents = {}
local INITIAL_JOINT_POSITIONS = {}

-- Function to initialize parameters and handles
function initializeParameters()
    local handles = {
        mainIkTask = sim.getIkGroupHandle('irb360_mainTask'),
        ikModeTipDummy = sim.getObjectHandle('irb360_ikTip'),
        irb360Base = sim.getObjectAssociatedWithScript(sim.handle_self),
        suctionPadSensor = sim.getObjectHandle('suctionPadSensor'),
        suctionPadLoopClosureDummy1 = sim.getObjectHandle('suctionPadLoopClosureDummy1'),
        suctionPadLoopClosureDummy2 = sim.getObjectHandle('suctionPadLoopClosureDummy2'),
        suctionPad = sim.getObjectHandle('suctionPad'),
        suctionPadLink = sim.getObjectHandle('suctionPadLink'),
    }

    -- Handles for FK and IK joint modes
    handles.fkDrivingJoints = {
        sim.getObjectHandle('irb360_drivingJoint1'),
        sim.getObjectHandle('irb360_drivingJoint2'),
        sim.getObjectHandle('irb360_drivingJoint3'),
        sim.getObjectHandle('irb360_motor')
    }
    handles.ikDrivingJoints = {
        sim.getObjectHandle('irb360_cartesianX'),
        sim.getObjectHandle('irb360_cartesianY'),
        sim.getObjectHandle('irb360_cartesianZ'),
        sim.getObjectHandle('irb360_motor')
    }

    -- Initialize dynamics parameters for joints
    local function initializeDynamics()
        return {0, 0, 0, 0}, {ANG_VEL, ANG_VEL, ANG_VEL, ANG_VEL}, {ANG_ACCEL, ANG_ACCEL, ANG_ACCEL, ANG_ACCEL}, {ANG_JERK, ANG_JERK, ANG_JERK, ANG_JERK}
    end
    local currentAngVel, maxAngVel, maxAngAccel, maxAngJerk = initializeDynamics()
    local currentLinVel, maxLinVel, maxLinAccel, maxLinJerk = initializeDynamics()

    -- Set initial position of the tip dummy
    local initialPosition = sim.getObjectPosition(handles.ikModeTipDummy, handles.irb360Base)
    
    -- Initialization of suction pad components and parameters
    local scriptHandle = sim.getScriptHandle('suctionPad')
    if scriptHandle ~= -1 then
        for key, param in pairs({
            'infiniteStrength', 
            'maxPullForce', 
            'maxShearForce', 
            'maxPeelTorque'
        }) do
            suctionPadComponents[key] = sim.getScriptSimulationParameter(scriptHandle, param)
        end
        sim.setLinkDummy(handles.suctionPadLoopClosureDummy1, -1)
        sim.setObjectParent(handles.suctionPadLoopClosureDummy1, handles.suctionPad, true)
        local m = sim.getObjectMatrix(handles.suctionPadLoopClosureDummy2, -1)
        sim.setObjectMatrix(handles.suctionPadLoopClosureDummy1, -1, m)
    else
        print("Error: suctionPad script handle is invalid.")
        return
    end

    -- Store initial joint positions
    for i = 1, #handles.fkDrivingJoints do
        table.insert(INITIAL_JOINT_POSITIONS, sim.getJointPosition(handles.fkDrivingJoints[i]))
    end
    initialJointPosition = INITIAL_JOINT_POSITIONS
end

-- Set joint mode function
local function setJointMode(jointHandles, mode)
    for i = 1, #jointHandles do
        sim.setJointMode(jointHandles[i], mode, 0)
    end
end

-- Set Forward Kinematics mode
setFkMode = function()
    sim.setIkElementProperties(mainIkTask, ikModeTipDummy, 0)
    setJointMode(fkDrivingJoints, sim.jointmode_passive)
end

-- Set Inverse Kinematics mode
setIkMode = function()
    local p = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    for i = 1, 3 do
        sim.setJointPosition(ikDrivingJoints[i], p[i] - initialPosition[i])
    end
    sim.setIkElementProperties(mainIkTask, ikModeTipDummy, sim.ik_x_constraint + sim.ik_y_constraint + sim.ik_z_constraint)
    setJointMode(fkDrivingJoints, sim.jointmode_ik)
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

-- Movement utility function
local function clamp(value, min, max)
    return math.max(math.min(value, max), min)
end

-- Movement function
function move(direction, callback)
    local currentPos = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    local targetPos = {
        clamp(currentPos[1] + direction[1], BOUNDARY.MIN, BOUNDARY.MAX),
        clamp(currentPos[2] + direction[2], BOUNDARY.MIN, BOUNDARY.MAX),
        clamp(currentPos[3] + direction[3], BOUNDARY.MIN, BOUNDARY.MAX)
    }

    if currentPos[1] == targetPos[1] and currentPos[2] == targetPos[2] and currentPos[3] == targetPos[3] then
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
        clamp(x, BOUNDARY.MIN, BOUNDARY.MAX),
        clamp(y, BOUNDARY.MIN, BOUNDARY.MAX),
        clamp(z, BOUNDARY.MIN, BOUNDARY.MAX)
    }

    if currentPos[1] == targetPos[1] and currentPos[2] == targetPos[2] and currentPos[3] == targetPos[3] then
        print("Target position is out of bounds. Movement cancelled.")
        return "Movement cancelled: Target out of bounds"
    end

    sim.setObjectPosition(ikModeTipDummy, irb360Base, targetPos)
    local result = sim.handleIkGroup(mainIkTask)
    
    if result == sim.ikresult_success then
        print(string.format("Moved to position: (%.2f, %.2f, %.2f)", targetPos[1], targetPos[2], targetPos[3]))
        return string.format("Moved to (%.2f, %.2f, %.2f)", targetPos[1], targetPos[2], targetPos[3])
    else
        print("IK solver failed. Movement cancelled.")
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

-- Main thread function
function sysCall_threadmain()
    initializeParameters()
    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints, -1, currentAngVel, currentAngAccel, maxAngVel, maxAngAccel, maxAngJerk, {0, 0, 0, 0}, targetAngVel)
    setIkMode()

    while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
        local message, data = sim.getSimulatorMessage()
        if message == sim.message_keypress then
            local movementMap = {
                [2007] = {0, 0, MOVE_STEP},    -- up key
                [2008] = {0, 0, -MOVE_STEP},   -- down key
                [2009] = {-MOVE_STEP, 0, 0},   -- left key
                [2010] = {MOVE_STEP, 0, 0},    -- right key
                [string.byte('q')] = {0, -MOVE_STEP, 0}, -- forward
                [string.byte('w')] = {0, MOVE_STEP, 0},  -- backward
                [string.byte('r')] = {},  -- reset
                [string.byte('a')] = true,  -- activate suction pad
                [string.byte('d')] = false, -- deactivate suction pad
            }
            local moveDirection = movementMap[data[1]]
            if moveDirection then
                if type(moveDirection) == 'table' then
                    if #moveDirection == 0 then
                        resetToInitialPosition()
                    else
                        move(moveDirection, moveCallback)
                    end
                else
                    controlSuctionPad(moveDirection)
                end
            end
        end
        sim.switchThread()
    end
end

-- Cleanup function to be called on script termination
function sysCall_cleanup() 
    -- Cleanup code can be added here if needed
end

-- Public API functions for external control
function moveUp() return move({0, 0, MOVE_STEP}, moveCallback), {}, {}, {}, '' end
function moveDown() return move({0, 0, -MOVE_STEP}, moveCallback), {}, {}, {}, '' end
function moveLeft() return move({-MOVE_STEP, 0, 0}, moveCallback), {}, {}, {}, '' end
function moveRight() return move({MOVE_STEP, 0, 0}, moveCallback), {}, {}, {}, '' end
function moveForward() return move({0, -MOVE_STEP, 0}, moveCallback), {}, {}, {}, '' end
function moveBackward() return move({0, MOVE_STEP, 0}, moveCallback), {}, {}, {}, '' end

function resetPosition()
    if not initialJointPosition then
        print("Error: Initial joint positions not set")
        return {}, {}, {}, ''
    end
    
    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints, -1, currentAngVel, currentAngAccel, maxAngVel, maxAngAccel, maxAngJerk, initialJointPosition, targetAngVel)
    sim.wait(2)

    setIkMode()
    local currentJointPositions = {}
    for i = 1, #fkDrivingJoints do
        table.insert(currentJointPositions, sim.getJointPosition(fkDrivingJoints[i]))
    end

    local isReset = arePositionsClose(currentJointPositions, initialJointPosition)
    print(isReset and "Successfully reset to initial joint positions" or "Failed to reset to initial joint positions")

    return {}, {}, {}, ''
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

function activateSuctionPad() controlSuctionPad(true) end
function deactivateSuctionPad() controlSuctionPad(false) end
function enableIkMode() setIkMode() return {0} end
function disableIkMode() setFkMode() return {0} end
