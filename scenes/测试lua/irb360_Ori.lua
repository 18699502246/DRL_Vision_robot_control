-- This child script runs in a thread. It is meant as an example code to drive the IRB360 robot.
-- There is another child script in this model. The other one is attached to object 'irb360_loopTarget'
-- and doesn't run in a thread and is in charge of explicitely executing the inverse kinematics calculation

setFkMode=function()
    -- disable the platform positional constraints:
    sim.setIkElementProperties(mainIkTask,ikModeTipDummy,0)
    -- Set the driving joints into passive mode (not taken into account during IK resolution):
    sim.setJointMode(fkDrivingJoints[1],sim.jointmode_passive,0)
    sim.setJointMode(fkDrivingJoints[2],sim.jointmode_passive,0)
    sim.setJointMode(fkDrivingJoints[3],sim.jointmode_passive,0)
end

setIkMode=function()
    -- Make sure the target position is at the same position at the tip position:
    local p=sim.getObjectPosition(ikModeTipDummy,irb360Base)
    sim.setJointPosition(ikDrivingJoints[1],p[1]-initialPosition[1])
    sim.setJointPosition(ikDrivingJoints[2],p[2]-initialPosition[2])
    sim.setJointPosition(ikDrivingJoints[3],p[3]-initialPosition[3])
    -- enable the platform positional constraints:
    sim.setIkElementProperties(mainIkTask,ikModeTipDummy,sim.ik_x_constraint+sim.ik_y_constraint+sim.ik_z_constraint)
    -- Set the base joints into ik mode (taken into account during IK resolution):
    sim.setJointMode(fkDrivingJoints[1],sim.jointmode_ik,0)
    sim.setJointMode(fkDrivingJoints[2],sim.jointmode_ik,0)
    sim.setJointMode(fkDrivingJoints[3],sim.jointmode_ik,0)
end

function sysCall_threadmain()
    -- Put some initialization code here:
    -- Retrieve some values:
    mainIkTask=sim.getIkGroupHandle('irb360_mainTask')
    ikModeTipDummy=sim.getObjectHandle('irb360_ikTip')
    -- Following are the joints that we control when in FK mode:
    fkDrivingJoints={-1,-1,-1,-1}
    fkDrivingJoints[1]=sim.getObjectHandle('irb360_drivingJoint1')
    fkDrivingJoints[2]=sim.getObjectHandle('irb360_drivingJoint2')
    fkDrivingJoints[3]=sim.getObjectHandle('irb360_drivingJoint3')
    fkDrivingJoints[4]=sim.getObjectHandle('irb360_motor')
    -- Following are the joints that we control when in IK mode (we use joints in order to be able to use the sim.moveToJointPositions command here too):
    ikDrivingJoints={-1,-1,-1,-1}
    ikDrivingJoints[1]=sim.getObjectHandle('irb360_cartesianX')
    ikDrivingJoints[2]=sim.getObjectHandle('irb360_cartesianY')
    ikDrivingJoints[3]=sim.getObjectHandle('irb360_cartesianZ')
    ikDrivingJoints[4]=sim.getObjectHandle('irb360_motor')

    irb360Base=sim.getObjectAssociatedWithScript(sim.handle_self)

    angVel=180*math.pi/180
    angAccel=360*math.pi/180
    angJerk=3600*math.pi/180
    currentAngVel={0,0,0,0}
    currentAngAccel={0,0,0,0}
    maxAngVel={angVel,angVel,angVel,angVel}
    maxAngAccel={angAccel,angAccel,angAccel,angAccel}
    maxAngJerk={angJerk,angJerk,angJerk,angJerk}
    targetAngVel={0,0,0,0}

    linVel=2
    linAccel=3
    linJerk=30
    currentLinVel={0,0,0,0}
    currentLinAccel={0,0,0,0}
    maxLinVel={linVel,linVel,linVel,angVel}
    maxLinAccel={linAccel,linAccel,linAccel,angAccel}
    maxLinJerk={linJerk,linJerk,linJerk,angJerk}
    targetAngVel={0,0,0,0}

    -- First, make sure we are in initial position:
    setFkMode()
    sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0,0,0,0},targetAngVel)
    initialPosition=sim.getObjectPosition(ikModeTipDummy,irb360Base)

    -- The main routine here:
    -- In following loop, we move alternatively in forward, then inverse kinematic mode.
    -- We drive the main joints (the motors) in FK mode with the 'sim.moveToJointPositions'
    -- In IK mode, we could directly set the desired tip position for the 'irb360_ikTarget' object
    -- with the 'simMoveToPositon' command. However, in order to be able to drive simultaneously the
    -- rotational joint, we simply attached the 'irb360_ikTarget' to 3 cartesian joints, that we drive
    -- in a similar way as in FK mode. We could also imagine writing additional threads where each could
    -- take car.e of one motor for instance.

    -- The 'sim.moveToJointPositions' command, as used below, will drive all joints simultaneously (they start
    -- and stop at the same time). In order to be able to drive the central axis much faster than the other
    -- joints, we applied a trick: the rotational motor is built on top of another motor ('irb360_motorAux')
    -- that is linearly dependent of the first one (motorAux=3*motor). So if we drive the axis motor to x degrees,
    -- the total rotation will be 4x degrees. So we have to remember to feed always 1/4 of the desired angular
    -- value for the central axis
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        -- First in forward kinematics mode:
        setFkMode()
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{-80*math.pi/180,0*math.pi/180,0*math.pi/180,45*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{45*math.pi/180,0*math.pi/180,0*math.pi/180,45*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,0*math.pi/180,0*math.pi/180,45*math.pi/180},targetAngVel)

        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,-80*math.pi/180,0*math.pi/180,0*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,45*math.pi/180,0*math.pi/180,0*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180},targetAngVel)

        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,0*math.pi/180,-80*math.pi/180,45*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,0*math.pi/180,45*math.pi/180,45*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,0*math.pi/180,0*math.pi/180,45*math.pi/180},targetAngVel)

        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{-80*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{-80*math.pi/180,-80*math.pi/180,0*math.pi/180,0*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,-80*math.pi/180,-80*math.pi/180,0*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,0*math.pi/180,-80*math.pi/180,0*math.pi/180},targetAngVel)
        sim.rmlMoveToJointPositions(fkDrivingJoints,-1,currentAngVel,currentAngAccel,maxAngVel,maxAngAccel,maxAngJerk,{0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180},targetAngVel)

        -- Now in inverse kinematics mode:
        setIkMode()
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{0.4,0.4,0,45*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{-0.4,0.4,0,45*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{-0.4,-0.4,0,45*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{0.4,-0.4,0,45*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{0.4,0.4,0,45*math.pi/180},targetLinVel)

        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{0.3,0.3,-0.1,0*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{-0.3,0.3,-0.1,0*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{-0.3,-0.3,-0.1,0*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{0.3,-0.3,-0.1,0*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{0.3,0.3,-0.1,0*math.pi/180},targetLinVel)
        sim.rmlMoveToJointPositions(ikDrivingJoints,-1,currentLinVel,currentLinAccel,maxLinVel,maxLinAccel,maxLinJerk,{0,0,0,0*math.pi/180},targetLinVel)
    end
end
