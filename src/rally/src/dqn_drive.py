#!/usr/bin/env python
import numpy as np

from dqn.dqn import DQN

class DQN_Drive:
    angle = 0
    speed = 0
    
    def __init__(self, input_size, output_size, hidden_size, model_episode_no, skip_frame=2, stack_frame=5):
        self.dqn = DQN(input_size, output_size, hidden_size, stack_frame=stack_frame, skip_frame=skip_frame)
        self.dqn.model_load(model_episode_no)


    def set_motor(self, angle, speed):
        self.angle = angle
        self.motor = motor


    def set_data(self, data):
        data = np.array(data[:5])
        while len(self.dqn.observation_set) < self.dqn.skip_frame * self.dqn.stack_frame:
            self.dqn.observation_set.append(data)
        
        state = net.skip_stack_frame(data)  # 상태 생성
        action = net.get_action(state, 0)   # 행동 선택

        # 조향각 조정
        if action == 0:
            steering_deg = -30
        elif action == 1:
            steering_deg = 0
        else:
            steering_deg = 30

        speed = 35

        self.set_motor(steering_deg, speed)


    def get_motor(self):
        return self.angle, self.motor

