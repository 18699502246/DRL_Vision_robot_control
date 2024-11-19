initialPosition=sim.getObjectPosition(ikModeTipDummy,irb360Base)        当前末端执行器（ikModeTipDummy）相对于机器人基座（irb360Base）的位置信息
zeroPos=sim.getObjectPosition(ikModeTipDummy,-1)                        这一行代码获取末端执行器在全局坐标系中的位置，并将其储存到zeroPos变量中。使用-1表示获取全局坐标系中的位置，
function moveBack()
    zeroPos=INITIAL_WORLD_POSITIONS
    currentWorldPos = sim.getObjectPosition(ikModeTipDummy, irb360Base)
    print("currentWorldPos:", currentWorldPos)
    print("zeroPos:", zeroPos)
    direction = {currentWorldPos[1]-zeroPos[1], currentWorldPos[2]-zeroPos[2], currentWorldPos[3]-zeroPos[3]}
    move([0,0,zeroPos[3]], moveCallback)
    print("backward")
    return {},{},{},''
end