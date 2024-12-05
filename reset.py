import sim  # CoppeliaSim Python API
import time
import logging

def test_stop_and_start_simulation():
    # 日志配置
    logging.basicConfig(level=logging.INFO)
    
    # 连接到仿真环境
    sim.simxFinish(-1)  # 关闭所有打开的连接
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    
    if client_id == -1:
        logging.error('Failed to connect to remote API server')
        raise RuntimeError("Simulation connection failed")
    
    logging.info('Connected to remote API server')
    
    # 启动仿真
    if sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok:
        logging.info('Simulation started successfully')
    else:
        logging.error('Failed to start simulation')
        sim.simxFinish(client_id)
        return
    
    # 运行仿真一段时间
    logging.info('Running simulation for 5 seconds...')
    time.sleep(5)
    
    # 停止仿真
    if sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok:
        logging.info('Simulation stopped successfully')
    else:
        logging.error('Failed to stop simulation')
    
    # 重新启动仿真
    if sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok:
        logging.info('Simulation restarted successfully')
    else:
        logging.error('Failed to restart simulation')
    
    # 再次运行仿真一段时间
    logging.info('Running simulation for another 5 seconds...')
    time.sleep(5)
    
    # 最后停止仿真
    if sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok:
        logging.info('Simulation stopped successfully')
    else:
        logging.error('Failed to stop simulation')
    
    # 关闭连接
    sim.simxFinish(client_id)

if __name__ == '__main__':
    test_stop_and_start_simulation()