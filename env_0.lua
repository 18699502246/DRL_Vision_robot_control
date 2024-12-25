sim=require'sim'
simIK=require'simIK'

setFkMode=function(info)
    info.ikMode=false
    -- disable the platform positional constraints:
    simIK.setElementFlags(info.ikEnv,info.mainIkGroup,info.platformIkElement,0)
    -- Set the driving joints into passive mode (not taken into account during IK resolution):
    simIK.setJointMode(info.ikEnv,info.fkDrivingJoints_inIkEnv[1],simIK.jointmode_passive)
    simIK.setJointMode(info.ikEnv,info.fkDrivingJoints_inIkEnv[2],simIK.jointmode_passive)
    simIK.setJointMode(info.ikEnv,info.fkDrivingJoints_inIkEnv[3],simIK.jointmode_passive)
end

setIkMode=function(info)
    info.ikMode=true
    -- enable the platform positional constraints:
    simIK.setElementFlags(info.ikEnv,info.mainIkGroup,info.platformIkElement,1)
    -- Set the base joints into ik mode (taken into account during IK resolution):
    simIK.setJointMode(info.ikEnv,info.fkDrivingJoints_inIkEnv[1],simIK.jointmode_ik)
    simIK.setJointMode(info.ikEnv,info.fkDrivingJoints_inIkEnv[2],simIK.jointmode_ik)
    simIK.setJointMode(info.ikEnv,info.fkDrivingJoints_inIkEnv[3],simIK.jointmode_ik)
end

function moveToConfigCallback(data)
    local joints=data.auxData.fkDrivingJoints
    if data.auxData.ikMode then
        joints=data.auxData.ikDrivingJoints
    end
    for i=1,#joints,1 do
        sim.setJointPosition(joints[i], data.pos[i])
    end
    local error=false
    error=error or simIK.handleGroup(data.auxData.ikEnv,data.auxData.mainIkGroup,{syncWorlds=true})~=simIK.result_success
    error=error or simIK.handleGroup(data.auxData.ikEnv,data.auxData.axisIkGroup,{syncWorlds=true})~=simIK.result_success
    if error then
        sim.addLog(sim.verbosity_scripterrors,sim.getObjectAlias(data.auxData.robotHandle,1),"IK solver failed.")
    else
        for i=1,#data.auxData.bridgeIkGroups,1 do
            simIK.handleGroup(data.auxData.ikEnv,data.auxData.bridgeIkGroups[i],{syncWorlds=true,allowError=true})
        end
    end
end

function moveToConfig(goalConfig,info,maxVelocity,maxAcceleration,maxJerk)
    maxVelocity = maxVelocity or info.maxAngVel
    maxAcceleration = maxAcceleration or info.maxAngAccel
    maxJerk = maxJerk or info.maxAngJerk


    local joints=info.fkDrivingJoints
    if info.ikMode then
        joints=info.ikDrivingJoints
    end
    local startConfig={}
    for i=1,#joints,1 do
        startConfig[i]=sim.getJointPosition(joints[i])
    end
    local params = {
        pos = startConfig,
        targetPos = goalConfig,
        maxVel = maxVelocity,
        maxAccel = maxAcceleration,
        maxJerk = maxJerk,
        callback = moveToConfigCallback,
        auxData = info
    }
    sim.moveToConfig(params)
end

function sysCall_thread()

    -- Following are the joints that we control when in FK mode:
    local fkDrivingJoints={-1,-1,-1,-1}
    fkDrivingJoints[1]=sim.getObject('../drivingJoint1')
    fkDrivingJoints[2]=sim.getObject('../drivingJoint2')
    fkDrivingJoints[3]=sim.getObject('../drivingJoint3')
    fkDrivingJoints[4]=sim.getObject('../motor')

    -- Following are the joints that we control when in IK mode:
    local ikDrivingJoints={-1,-1,-1,-1}
    ikDrivingJoints[1]=sim.getObject('../cartesianX')
    ikDrivingJoints[2]=sim.getObject('../cartesianY')
    ikDrivingJoints[3]=sim.getObject('../cartesianZ')
    ikDrivingJoints[4]=sim.getObject('../motor')
    

    ikEnv=simIK.createEnvironment()

    -- The main IK group (main task, the 3 arms and platform):
    local base=sim.getObject('..')
    local ikTip=sim.getObject('../ikTip')
    local ikTarget=sim.getObject('../ikTarget')
    local loop1Tip=sim.getObject('../loopTip')
    local loop1Target=sim.getObject('../loopTarget')
    local loop2Tip=sim.getObject('../loopTip0')
    local loop2Target=sim.getObject('../loopTarget0')
    local loop3Tip=sim.getObject('../loopTip1')
    local loop3Target=sim.getObject('../loopTarget1')
    local loop4Tip=sim.getObject('../loopTip2')
    local loop4Target=sim.getObject('../loopTarget2')
    local loop5Tip=sim.getObject('../loopTip3')
    local loop5Target=sim.getObject('../loopTarget3')
    
    local ikGroup_main=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_main,simIK.method_pseudo_inverse,0,6)
    local ikElementTip=simIK.addElementFromScene(ikEnv,ikGroup_main,base,ikTip,ikTarget,simIK.constraint_position)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_main,base,loop1Tip,loop1Target,simIK.constraint_pose)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_main,base,loop2Tip,loop2Target,simIK.constraint_pose)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_main,base,loop3Tip,loop3Target,simIK.constraint_pose)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_main,base,loop4Tip,loop4Target,simIK.constraint_pose)
    local ikElement,mapping=simIK.addElementFromScene(ikEnv,ikGroup_main,base,loop5Tip,loop5Target,simIK.constraint_pose)
    local fkDrivingJoints_inIkEnv={mapping[fkDrivingJoints[1]],mapping[fkDrivingJoints[2]],mapping[fkDrivingJoints[3]]}
    

    -- The secondary IK group (handling the center axis):
    local axisBase=sim.getObject('../axisL')
    local axisTip=sim.getObject('../axisTip')
    local axisTarget=sim.getObject('../axisTarget')
    
    local ikGroup_axis=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_axis,simIK.method_pseudo_inverse,0,6)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_axis,axisBase,axisTip,axisTarget,simIK.constraint_pose)
    simIK.setElementPrecision(ikEnv,ikGroup_axis,ikElement,{0.0001,0.1})
    
    
    -- The IK groups handling the secondary arm bridges:
    local bridge1Base=sim.getObject('../j2')
    local bridge1Tip=sim.getObject('../bridgeLTip')
    local bridge1Target=sim.getObject('../bridgeLTarget')

    local ikGroup_bridge1=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_bridge1,simIK.method_damped_least_squares,0.01,3)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_bridge1,bridge1Base,bridge1Tip,bridge1Target,simIK.constraint_position)
    
    local bridge2Base=sim.getObject('../j26')
    local bridge2Tip=sim.getObject('../bridgeRTip')
    local bridge2Target=sim.getObject('../bridgeRTarget')
    
    local ikGroup_bridge2=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_bridge2,simIK.method_damped_least_squares,0.01,3)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_bridge2,bridge2Base,bridge2Tip,bridge2Target,simIK.constraint_position)
    
    local bridge3Base=sim.getObject('../j28')
    local bridge3Tip=sim.getObject('../bridgeLTip0')
    local bridge3Target=sim.getObject('../bridgeLTarget0')
    
    local ikGroup_bridge3=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_bridge3,simIK.method_damped_least_squares,0.01,3)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_bridge3,bridge3Base,bridge3Tip,bridge3Target,simIK.constraint_position)
    
    local bridge4Base=sim.getObject('../j29')
    local bridge4Tip=sim.getObject('../bridgeRTip0')
    local bridge4Target=sim.getObject('../bridgeRTarget0')

    local ikGroup_bridge4=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_bridge4,simIK.method_damped_least_squares,0.01,3)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_bridge4,bridge4Base,bridge4Tip,bridge4Target,simIK.constraint_position)

    local bridge5Base=sim.getObject('../j31')
    local bridge5Tip=sim.getObject('../bridgeLTip1')
    local bridge5Target=sim.getObject('../bridgeLTarget1')
    
    local ikGroup_bridge5=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_bridge5,simIK.method_damped_least_squares,0.01,3)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_bridge5,bridge5Base,bridge5Tip,bridge5Target,simIK.constraint_position)

    local bridge6Base=sim.getObject('../j34')
    local bridge6Tip=sim.getObject('../bridgeRTip1')
    local bridge6Target=sim.getObject('../bridgeRTarget1')
    
    local ikGroup_bridge6=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup_bridge6,simIK.method_damped_least_squares,0.01,3)
    local ikElement=simIK.addElementFromScene(ikEnv,ikGroup_bridge6,bridge6Base,bridge6Tip,bridge6Target,simIK.constraint_position)
    

    -- Movement profiles when in FK mode:
    local angVel=180*math.pi/180
    local angAccel=360*math.pi/180
    local angJerk=3600*math.pi/180
    local maxAngVel={angVel,angVel,angVel,angVel}
    local maxAngAccel={angAccel,angAccel,angAccel,angAccel}
    local maxAngJerk={angJerk,angJerk,angJerk,angJerk}

    -- Movement profiles when in IK mode:
    local linVel=2
    local linAccel=3
    local linJerk=30
    local maxLinVel={linVel,linVel,linVel,angVel}
    local maxLinAccel={linAccel,linAccel,linAccel,angAccel}
    local maxLinJerk={linJerk,linJerk,linJerk,angJerk}
    
    local info={}
    info.robotHandle=base
    info.ikMode=false
    info.ikEnv=ikEnv
    info.mainIkGroup=ikGroup_main
    info.platformIkElement=ikElementTip
    info.axisIkGroup=ikGroup_axis
    info.bridgeIkGroups={ikGroup_bridge1,ikGroup_bridge2,ikGroup_bridge3,ikGroup_bridge4,ikGroup_bridge5,ikGroup_bridge6}
    info.fkDrivingJoints=fkDrivingJoints
    info.ikDrivingJoints=ikDrivingJoints
    info.fkDrivingJoints_inIkEnv=fkDrivingJoints_inIkEnv
    

    -- First, make sure we are in initial position:
    setFkMode(info)
    moveToConfig({-80*math.pi/180,0*math.pi/180,0*math.pi/180,45*math.pi/180},info,maxAngVel,maxAngAccel,maxAngJerk)    

    -- The main routine here:
    -- In following loop, we move alternatively in forward, then inverse kinematic mode.
    
end

function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv)
end
