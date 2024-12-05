def _calculate_reward(self, new_position):
    # 添加抓取相关的奖励
    grasp_success = self._check_grasp_status()  # 需要实现这个方法
    grasp_reward = 2.0 if grasp_success else 0.0
    
    # 添加任务完成奖励
    task_complete = self._is_terminated(new_position)
    completion_reward = 5.0 if task_complete else 0.0
    
    # 原有奖励计算
    proximity_reward = 1.0 / (1.0 + self.distance)
    orientation_reward = self._calculate_orientation_reward()
    efficiency_reward = 1.0 / (1.0 + self.current_step)
    velocity_penalty = -0.1 * np.linalg.norm(self._calculate_velocity())
    
    return (proximity_reward * 0.3 + 
            orientation_reward * 0.2 + 
            efficiency_reward * 0.1 +
            velocity_penalty * 0.1 +
            grasp_reward * 0.2 +
            completion_reward * 0.1)



def _check_grasp_status(self):
    """检查吸盘是否成功吸附物体"""
    # 调用CoppeliaSim中的脚本函数检查吸盘状态
    res, ret_ints, _, _, _ = sim.simxCallScriptFunction(
        self.client_id,
        self.robot_name,
        self.script_type,
        'checkSuctionStatus',
        [], [], [], bytearray(),
        sim.simx_opmode_oneshot_wait
    )
    return ret_ints[0] == 1 if res == sim.simx_return_ok else False
