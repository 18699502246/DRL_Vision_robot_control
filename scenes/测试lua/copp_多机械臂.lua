-- This threaded script handles the motion of the IRB360 robot. It can be controlled
-- in forward or inverse kinematics, but in this case it is only controlled in inverse kinematics

------------------------------------------------------------------------------
-- The functions sim.moveToPosition and sim.moveToJointPositions are DEPRECATED.
-- Try using following much more powerful functions instead:
-- simRMLPosition or simRMLVelocity
------------------------------------------------------------------------------

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
    -- Initialization:
    sim.setThreadSwitchTiming(2) -- Default timing for automatic thread switching

    sim.wait(2)

    -- Retrieve some values:
    mainIkTask=sim.getIkGroupHandle('irb360_mainTask')
    ikModeTipDummy=sim.getObjectHandle('irb360_ikTip')
    ikModeTargetDummy=sim.getObjectHandle('irb360_ikTarget')
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


    conveyor=sim.getObjectHandle('whiteConveyor#')
    suctionPad=sim.getObjectHandle('suctionPad')

    irb360Base=sim.getObjectAssociatedWithScript(sim.handle_self)
    angularVelocity=math.pi
    angularAccel=4*math.pi
    linearVelocity=2
    linearAccel=10
    angleToLinearCoeff=0.0001 -- anything small
    pickupHeight=0.7085 

    -- The 'sim.moveToJointPositions' command, as used below, will drive all joints simultaneously (they start
    -- and stop at the same time). In order to be able to drive the central axis much faster than the other
    -- joints, we applied a trick: the rotational motor is built on top of another motor ('irb360_motorAux')
    -- that is linearly dependent of the first one (motorAux=3*motor). So if we drive the axis motor to x degrees,
    -- the total rotation will be 4x degrees. So we have to remember to feed always 1/4 of the desired angular
    -- value for the central axis.

    -- First, make sure we are in initial position:
    setFkMode()
    --sim.moveToJointPositions(fkDrivingJoints,{12*math.pi/180,12*math.pi/180,12*math.pi/180,0},angularVelocity,angularAccel)
    --sim.wait(99999)
    sim.moveToJointPositions(fkDrivingJoints,{0,0,0,0},angularVelocity,angularAccel)
    initialPosition=sim.getObjectPosition(ikModeTipDummy,irb360Base)
    zeroPos=sim.getObjectPosition(ikModeTipDummy,-1)
    setIkMode()

    stackOccupancy={{nil,nil,nil},{nil,nil,nil}} -- for I and T stacks (each one of them has red, green and blue components)
    dropPositions={{{nil,nil,nil,nil},{nil,nil,nil,nil},{nil,nil,nil,nil}},{{nil,nil,nil,nil},{nil,nil,nil,nil},{nil,nil,nil,nil}}} -- for I and T stacks (each one of them has red, green and blue components)

    stackOccupancy={{{},{}},{{},{}}}
    dropPositions={{{{},{},{}},{{},{},{}}},{{{},{},{}},{{},{},{}}}}
    for i=1,2,1 do
        for j=1,2,1 do
            for k=1,3,1 do
                stackOccupancy[i][j][k]=10
                for l=1,4,1 do
                    dropPositions[i][j][k][l]=nil
                end
            end
        end
    end


    local offCorrX=-0.01
    local offCorrY=-0.02
    local dropH=0.017
    local maxDist=0.64

    for i=-1,10,1 do
        local suffix='#'
        if i>=0 then
            suffix=i..'#'
        end
        container=sim.getObjectHandle('TcontainerRed'..suffix..'@silentError')
        for j=1,2,1 do
            if container>=0 and not dropPositions[j][2][1][1] then
                p=sim.getObjectPosition(container,irb360Base)
                p[1]=p[1]+offCorrX
                p[2]=p[2]+offCorrY
                o=sim.getObjectOrientation(container,irb360Base)
                d=math.sqrt(p[1]*p[1]+p[2]*p[2])
                if d<maxDist then
                    local r,grouping=sim.getObjectInt32Parameter(container,sim.shapeintparam_compound)
                    if grouping then
                        stackOccupancy[j][2][1]=0
                    else
                        stackOccupancy[j][2][1]=-9999 -- this contains in not a container, just a drop location
                    end
                    dropPositions[j][2][1][1]=p[1]
                    dropPositions[j][2][1][2]=p[2]
                    dropPositions[j][2][1][3]=dropH
                    dropPositions[j][2][1][4]=o[3]
                    break
                end
            end
        end

        container=sim.getObjectHandle('TcontainerGreen'..suffix..'@silentError')
        for j=1,2,1 do
            if container>=0 and not dropPositions[j][2][2][1] then
                p=sim.getObjectPosition(container,irb360Base)
                p[1]=p[1]+offCorrX
                p[2]=p[2]+offCorrY
                o=sim.getObjectOrientation(container,irb360Base)
                d=math.sqrt(p[1]*p[1]+p[2]*p[2])
                if d<maxDist then
                    local r,grouping=sim.getObjectInt32Parameter(container,sim.shapeintparam_compound)
                    if grouping~=0 then
                        stackOccupancy[j][2][2]=0
                    else
                        stackOccupancy[j][2][2]=-9999 -- this container in not a container, just a drop location
                    end
                    dropPositions[j][2][2][1]=p[1]
                    dropPositions[j][2][2][2]=p[2]
                    dropPositions[j][2][2][3]=dropH
                    dropPositions[j][2][2][4]=o[3]
                    break
                end
            end
        end

        container=sim.getObjectHandle('TcontainerBlue'..suffix..'@silentError')
        for j=1,2,1 do
            if container>=0 and not dropPositions[j][2][3][1] then
                p=sim.getObjectPosition(container,irb360Base)
                p[1]=p[1]+offCorrX
                p[2]=p[2]+offCorrY
                o=sim.getObjectOrientation(container,irb360Base)
                d=math.sqrt(p[1]*p[1]+p[2]*p[2])
                if d<maxDist then
                    local r,grouping=sim.getObjectInt32Parameter(container,sim.shapeintparam_compound)
                    if grouping~=0 then
                        stackOccupancy[j][2][3]=0
                    else
                        stackOccupancy[j][2][3]=-9999 -- this container in not a container, just a drop location
                    end
                    dropPositions[j][2][3][1]=p[1]
                    dropPositions[j][2][3][2]=p[2]
                    dropPositions[j][2][3][3]=dropH
                    dropPositions[j][2][3][4]=o[3]
                    break
                end
            end
        end

        container=sim.getObjectHandle('IcontainerRed'..suffix..'@silentError')
        for j=1,2,1 do
            if container>=0 and not dropPositions[j][1][1][1] then
                p=sim.getObjectPosition(container,irb360Base)
                p[1]=p[1]+offCorrX
                p[2]=p[2]+offCorrY
                o=sim.getObjectOrientation(container,irb360Base)
                d=math.sqrt(p[1]*p[1]+p[2]*p[2])
                if d<maxDist then
                    local r,grouping=sim.getObjectInt32Parameter(container,sim.shapeintparam_compound)
                    if grouping~=0 then
                        stackOccupancy[j][1][1]=0
                    else
                        stackOccupancy[j][1][1]=-9999 -- this container in not a container, just a drop location
                    end
                    dropPositions[j][1][1][1]=p[1]
                    dropPositions[j][1][1][2]=p[2]
                    dropPositions[j][1][1][3]=dropH
                    dropPositions[j][1][1][4]=o[3]
                    break
                end
            end
        end

        container=sim.getObjectHandle('IcontainerGreen'..suffix..'@silentError')
        for j=1,2,1 do
            if container>=0 and not dropPositions[j][1][2][1] then
                p=sim.getObjectPosition(container,irb360Base)
                p[1]=p[1]+offCorrX
                p[2]=p[2]+offCorrY
                o=sim.getObjectOrientation(container,irb360Base)
                d=math.sqrt(p[1]*p[1]+p[2]*p[2])
                if d<maxDist then
                    local r,grouping=sim.getObjectInt32Parameter(container,sim.shapeintparam_compound)
                    if grouping~=0 then
                        stackOccupancy[j][1][2]=0
                    else
                        stackOccupancy[j][1][2]=-9999 -- this container in not a container, just a drop location
                    end
                    dropPositions[j][1][2][1]=p[1]
                    dropPositions[j][1][2][2]=p[2]
                    dropPositions[j][1][2][3]=dropH
                    dropPositions[j][1][2][4]=o[3]
                    break
                end
            end
        end

        container=sim.getObjectHandle('IcontainerBlue'..suffix..'@silentError')
        for j=1,2,1 do
            if container>=0 and not dropPositions[j][1][3][1] then
                p=sim.getObjectPosition(container,irb360Base)
                p[1]=p[1]+offCorrX
                p[2]=p[2]+offCorrY
                o=sim.getObjectOrientation(container,irb360Base)
                d=math.sqrt(p[1]*p[1]+p[2]*p[2])
                if d<maxDist then
                    local r,grouping=sim.getObjectInt32Parameter(container,sim.shapeintparam_compound)
                    if grouping~=0 then
                        stackOccupancy[j][1][3]=0
                    else
                        stackOccupancy[j][1][3]=-9999 -- this container in not a container, just a drop location
                    end
                    dropPositions[j][1][3][1]=p[1]
                    dropPositions[j][1][3][2]=p[2]
                    dropPositions[j][1][3][3]=dropH
                    dropPositions[j][1][3][4]=o[3]
                    break
                end
            end
        end
    end

    local j=0
    local irbMatrixInv=sim.getObjectMatrix(irb360Base,-1)
    irbMatrixInv=simGetInvertedMatrix(irbMatrixInv)
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        t=sim.getSimulationTime()
        dt=sim.getSimulationTimeStep()
        cv=sim.getScriptSimulationParameter(sim.getScriptAssociatedWithObject(conveyor),'conveyorBeltVelocity')
        -- Get the information about the shape positions etc:
        sim.setThreadAutomaticSwitch(false) -- lock thread switching for a short moment, so that we don't corrupt the 'shapeInfos' signal (another thread could be modifying that signal too)
        info=sim.getStringSignal('shapeInfos')
        if (info) then
            -- Each shape info has 24 bytes (6*4), so loop through all info:
            local fullBreak=false
            for i=string.len(info)/24,1,-1 do
                data=string.sub(info,(i-1)*24+1,(i-1)*24+24)
                newInfo=""
                if (i~=1) then
                    newInfo=newInfo..string.sub(info,1,(i-1)*24+0)
                end
                if (i~=string.len(info)/24) then
                    newInfo=newInfo..string.sub(info,(i-1)*24+24+1)
                end
                intData=sim.unpackInt32Table(data,0,2)
                floatData=sim.unpackFloatTable(data,2,4)
                shapeCurrentPos={floatData[1],floatData[2]+(t-floatData[3])*cv,0}

                local relPos=sim.multiplyVector(irbMatrixInv,shapeCurrentPos)
                local sideDistPickOk=math.abs(relPos[2])<0.4
                pt=sim.getObjectPosition(ikModeTargetDummy,-1)
                for kk=1,2,1 do
                    j=j+1
                    if j>2 then
                        j=1
                    end
                    if (shapeCurrentPos[2]>zeroPos[2])and(shapeCurrentPos[2]<zeroPos[2]+0.1)and(stackOccupancy[j][intData[1]][intData[2]]<5)and sideDistPickOk then
                        -- Ok, we can pickup this shape if the drop position is valid!
    
                        rotAngle=-floatData[4]
                        if (intData[1]==1) then
                            rotAngle=rotAngle+math.pi/2 -- I shape
                        end
                        if dropPositions[j][intData[1]][intData[2]][1] then
                            -- Ok, the drop position is valid
                            dropPos={dropPositions[j][intData[1]][intData[2]][1],dropPositions[j][intData[1]][intData[2]][2],dropPositions[j][intData[1]][intData[2]][3],dropPositions[j][intData[1]][intData[2]][4]/4+rotAngle/4}

                            -- First store an updated shape info (where we removed the info of the shape we are going to pick up):
                            sim.setStringSignal('shapeInfos',newInfo)
                            sim.setThreadAutomaticSwitch(true) -- Now we can allow for thread switches too since we updated the 'shapeInfos' signal

                            -- The pickup movement will be with infinite acceleration to avoid to heavy calculations:
                            sim.setJointPosition(ikDrivingJoints[1],0)
                            sim.setJointPosition(ikDrivingJoints[2],0)
                            sim.setJointPosition(ikDrivingJoints[3],0)
                            p={shapeCurrentPos[1],shapeCurrentPos[2]+(0.5+dt)*cv,pickupHeight+0.01}
                            dp={p[1]-pt[1],p[2]-pt[2],p[3]-pt[3]}
                            dist=math.sqrt(dp[1]*dp[1]+dp[2]*dp[2]+dp[3]*dp[3])
                            sim.moveToPosition(ikModeTargetDummy,-1,p,nil,dist/0.5)
                            p[2]=p[2]+0.25*cv
                            p[3]=pickupHeight
                            pt=sim.getObjectPosition(ikModeTargetDummy,-1)
                            dp={p[1]-pt[1],p[2]-pt[2],p[3]-pt[3]}
                            dist=math.sqrt(dp[1]*dp[1]+dp[2]*dp[2]+dp[3]*dp[3])
                            sim.moveToPosition(ikModeTargetDummy,-1,p,nil,dist/0.25)
                            -- We are just above the shape. Activate the suction pad:
                            sim.setScriptSimulationParameter(sim.getScriptAssociatedWithObject(suctionPad),'active','true')
                            -- Now follow the movement of the conveyor for 1/4 of a second:
                            p[2]=p[2]+0.25*cv
                            p[3]=pickupHeight
                            pt=sim.getObjectPosition(ikModeTargetDummy,-1)
                            dp={p[1]-pt[1],p[2]-pt[2],p[3]-pt[3]}
                            dist=math.sqrt(dp[1]*dp[1]+dp[2]*dp[2]+dp[3]*dp[3])
                            sim.moveToPosition(ikModeTargetDummy,-1,p,nil,dist/0.25) -- not exactly cv, but ok
                            -- Now lift up (while still moving a bit forward:
                            p[2]=p[2]+0.25*cv
                            p[3]=pickupHeight+0.1
                            pt=sim.getObjectPosition(ikModeTargetDummy,-1)
                            dp={p[1]-pt[1],p[2]-pt[2],p[3]-pt[3]}
                            dist=math.sqrt(dp[1]*dp[1]+dp[2]*dp[2]+dp[3]*dp[3])
                            sim.moveToPosition(ikModeTargetDummy,-1,p,nil,dist/0.25) -- not exactly cv, but ok
                            sim.wait(0.25)

                            -- Now we move the tip with the auxiliary joints.
                            -- This part doesn't use infinite acceleration:
                            sim.setThreadAutomaticSwitch(false) -- we don't wanna be interrupted in next section:
                            sim.setObjectPosition(ikModeTargetDummy,sim.handle_parent,{0,0,0})
                            setIkMode()
                            sim.setThreadAutomaticSwitch(true)

                            sim.moveToJointPositions(ikDrivingJoints,dropPos,linearVelocity,linearAccel,angleToLinearCoeff)
                            sim.wait(0.25)
                            -- Deactivate the suction pad:
                            sim.setScriptSimulationParameter(sim.getScriptAssociatedWithObject(suctionPad),'active','false')
                            stackOccupancy[j][intData[1]][intData[2]]=stackOccupancy[j][intData[1]][intData[2]]+1
                            sim.wait(0.25)
                            -- Move back to the zero position:
                            sim.moveToJointPositions(ikDrivingJoints,{0,0,0,0},linearVelocity,linearAccel,angleToLinearCoeff)
                            fullBreak=true
                            break
                        end
                    end
                end
                if fullBreak then
                    break
                end
            end
            sim.setThreadAutomaticSwitch(true) -- Important to allow for thread switches again
        else
            sim.setThreadAutomaticSwitch(true) -- Important to allow for thread switches again
            sim.switchThread() -- don't waste time waiting
        end
    end
end

