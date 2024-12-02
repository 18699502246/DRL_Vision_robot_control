import sim  # CoppeliaSim Python API
import time

def start_simulation(client_id):
    """启动仿真"""
    sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot_wait)
    print("Simulation started")

def stop_simulation(client_id):
    """停止仿真"""
    sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot_wait)
    print("Simulation stopped")

def connect_to_simulator():
    """连接到CoppeliaSim仿真器"""
    sim.simxFinish(-1)  # 确保之前的连接被关闭
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if client_id == -1:
        print('Failed to connect to remote API server')
        raise RuntimeError("Simulation connection failed")
    print('Connected to remote API server')
    return client_id

def main():
    try:
        # 连接到仿真器
        client_id = connect_to_simulator()
        
        # 启动仿真
        start_simulation(client_id)
        
        # 等待一段时间
        time.sleep(2)
        
        # 停止仿真
        stop_simulation(client_id)
        # 等待一段时间
        time.sleep(2)
        # 再次启动仿真
        start_simulation(client_id)
        
        
        
        # 最终停止仿真
        # stop_simulation(client_id)
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 关闭连接
        sim.simxFinish(client_id)
        print("Connection closed")

if __name__ == "__main__":
    main()