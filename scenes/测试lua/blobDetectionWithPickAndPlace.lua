-- 这个线程脚本处理IRB360机器人运动。可以控制前向或逆向运动学，但在这种情况下仅控制逆向运动学

------------------------------------------------------------------------------
-- 函数sim.moveToPosition和sim.moveToJointPositions已弃用。
-- 请尝试使用以下更强大的函数：
-- simRMLPosition或simRMLVelocity
------------------------------------------------------------------------------

-- 设置正向运动学模式
setFkMode=function()
    -- 禁用平台位置约束：
    sim.setIkElementProperties(mainIkTask,ikModeTipDummy,0)
    -- 将驱动关节设置为被动模式（在IK解析期间不考虑）：
    sim.setJointMode(fkDrivingJoints[1],sim.jointmode_passive,0)
    sim.setJointMode(fkDrivingJoints[2],sim.jointmode_passive,0)
    sim.setJointMode(fkDrivingJoints[3],sim.jointmode_passive,0)
end

-- 设置逆向运动学模式
setIkMode=function()
    -- 确保目标位置与末端位置相同：
    local p=sim.getObjectPosition(ikModeTipDummy,irb360Base)
    sim.setJointPosition(ikDrivingJoints[1],p[1]-initialPosition[1])
    sim.setJointPosition(ikDrivingJoints[2],p[2]-initialPosition[2])
    sim.setJointPosition(ikDrivingJoints[3],p[3]-initialPosition[3])
    -- 启用平台位置约束：
    sim.setIkElementProperties(mainIkTask,ikModeTipDummy,sim.ik_x_constraint+sim.ik_y_constraint+sim.ik_z_constraint)
    -- 将基础关节设置为IK模式（在IK解析期间考虑）：
    sim.setJointMode(fkDrivingJoints[1],sim.jointmode_ik,0)
    sim.setJointMode(fkDrivingJoints[2],sim.jointmode_ik,0)
    sim.setJointMode(fkDrivingJoints[3],sim.jointmode_ik,0)
end

-- 线程主调用函数
function sysCall_threadmain()
    -- 初始化：
    sim.setThreadSwitchTiming(2) -- 自动线程切换的默认时间

    sim.wait(2)

    -- 检索一些值：
    mainIkTask=sim.getIkGroupHandle('irb360_mainTask')
    ikModeTipDummy=sim.getObjectHandle('irb360_ikTip')
    ikModeTargetDummy=sim.getObjectHandle('irb360_ikTarget')
    -- 以下是我们在FK模式下控制的关节：
    fkDrivingJoints={-1,-1,-1,-1}
    fkDrivingJoints[1]=sim.getObjectHandle('irb360_drivingJoint1')
    fkDrivingJoints[2]=sim.getObjectHandle('irb360_drivingJoint2')
    fkDrivingJoints[3]=sim.getObjectHandle('irb360_drivingJoint3')
    fkDrivingJoints[4]=sim.getObjectHandle('irb360_motor')
    -- 以下是我们在IK模式下控制的关节（我们使用关节以便在这里也能使用sim.moveToJointPositions命令）：
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
    angleToLinearCoeff=0.0001 -- 任意小值
    pickupHeight=0.7085 

    -- 'sim.moveToJointPositions'命令，如下面所用，将同时驱动所有关节（它们开始和停止的时间相同）。为了使中央轴比其他关节快得多，我们应用了一个技巧：旋转电机建立在另一个电机（'irb360_motorAux'）之上，该电机与第一个电机线性相关（motorAux=3*motor）。因此，如果我们将轴电机驱动到x度，总旋转将是4x度。因此，我们必须始终记得为中央轴提供所需角度值的1/4。

    -- 首先，确保我们处于初始位置：
    setFkMode()
    --sim.moveToJointPositions(fkDrivingJoints,{12*math.pi/180,12*math.pi/180,12*math.pi/180,0},angularVelocity,angularAccel)
    --sim.wait(99999)
    sim.moveToJointPositions(fkDrivingJoints,{0,0,0,0},angularVelocity,angularAccel)
    initialPosition=sim.getObjectPosition(ikModeTipDummy,irb360Base)
    zeroPos=sim.getObjectPosition(ikModeTipDummy,-1)
    setIkMode()

    stackOccupancy={{nil,nil,nil},{nil,nil,nil}} -- I和T堆栈的占用（每一个都有红色、绿色和蓝色组件）
    dropPositions={{{nil,nil,nil,nil},{nil,nil,nil,nil},{nil,nil,nil,nil}},{{nil,nil,nil,nil},{nil,nil,nil,nil},{nil,nil,nil,nil}}} -- I和T堆栈的投放位置（每一个都有红色、绿色和蓝色组件）

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
                        stackOccupancy[j][2][1]=-9999 -- 这个不是容器，仅为投放位置
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
                        stackOccupancy[j][2][2]=-9999 -- 这个容器不是容器，仅为投放位置
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
                        stackOccupancy[j][2][3]=-9999 -- 这个容器不是容器，仅为投放位置
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
                        stackOccupancy[j][1][1]=-9999 -- 这个容器不是容器，仅为投放位置
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
                        stackOccupancy[j][1][2]=-9999 -- 这个容器不是容器，仅为投放位置
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
                        stackOccupancy[j][1][3]=-9999 -- 这个容器不是容器，仅为投放位置
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
        -- 获取形状位置等信息：
        sim.setThreadAutomaticSwitch(false) -- 临时锁定线程切换，以免我们破坏'shapeInfos'信号（另一个线程也可能在修改该信号）
        info=sim.getStringSignal('shapeInfos')
        if (info) then
            -- 每个形状信息有24个字节（6*4），所以循环遍历所有信息：
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
                        -- 好的，如果投放位置有效，我们可以拾取这个形状！
    
                        rotAngle=-floatData[4]
                        if (intData[1]==1) then
                            rotAngle=rotAngle+math.pi/2 -- I形状
                        end
                        if dropPositions[j][intData[1]][intData[2]][1] then
                            -- 好的，投放位置有效
                            dropPos={dropPositions[j][intData[1]][intData[2]][1],dropPositions[j][intData[1]][intData[2]][2],dropPositions[j][intData[1]][intData[2]][3],dropPositions[j][intData[1]][intData[2]][4]/4+rotAngle/4}

                            -- 首先存储更新的形状信息（我们删除了要拾取的形状的信息）：
                            sim.setStringSignal('shapeInfos',newInfo)
                            sim.setThreadAutomaticSwitch(true) -- 现在我们可以允许线程切换，因为我们更新了'shapeInfos'信号

                            -- 拾取移动将以无限加速度进行，以避免过于繁重的计算：
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
                            -- 我们正好在形状上方。激活吸盘：
                            sim.setScriptSimulationParameter(sim.getScriptAssociatedWithObject(suctionPad),'active','true')
                            -- 现在跟随传送带的移动，持续1/4秒：
                            p[2]=p[2]+0.25*cv
                            p[3]=pickupHeight
                            pt=sim.getObjectPosition(ikModeTargetDummy,-1)
                            dp={p[1]-pt[1],p[2]-pt[2],p[3]-pt[3]}
                            dist=math.sqrt(dp[1]*dp[1]+dp[2]*dp[2]+dp[3]*dp[3])
                            sim.moveToPosition(ikModeTargetDummy,-1,p,nil,dist/0.25) -- 不完全是cv，但可以
                            -- 现在抬起（同时往前移动一点）：
                            p[2]=p[2]+0.25*cv
                            p[3]=pickupHeight+0.1
                            pt=sim.getObjectPosition(ikModeTargetDummy,-1)
                            dp={p[1]-pt[1],p[2]-pt[2],p[3]-pt[3]}
                            dist=math.sqrt(dp[1]*dp[1]+dp[2]*dp[2]+dp[3]*dp[3])
                            sim.moveToPosition(ikModeTargetDummy,-1,p,nil,dist/0.25) -- 不完全是cv，但可以
                            sim.wait(0.25)

                            -- 现在我们用辅助关节移动尖端。
                            -- 这一部分不使用无限加速度：
                            sim.setThreadAutomaticSwitch(false) -- 我们不希望在下一部分中被打断：
                            sim.setObjectPosition(ikModeTargetDummy,sim.handle_parent,{0,0,0})
                            setIkMode()
                            sim.setThreadAutomaticSwitch(true)

                            sim.moveToJointPositions(ikDrivingJoints,dropPos,linearVelocity,linearAccel,angleToLinearCoeff)
                            sim.wait(0.25)
                            -- 禁用吸盘：
                            sim.setScriptSimulationParameter(sim.getScriptAssociatedWithObject(suctionPad),'active','false')
                            stackOccupancy[j][intData[1]][intData[2]]=stackOccupancy[j][intData[1]][intData[2]]+1
                            sim.wait(0.25)
                            -- 返回到零位置：
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
            sim.setThreadAutomaticSwitch(true) -- 重要的是再次允许线程切换
        else
            sim.setThreadAutomaticSwitch(true) -- 重要的是再次允许线程切换
            sim.switchThread() -- 不要浪费时间等待
        end
    end
end