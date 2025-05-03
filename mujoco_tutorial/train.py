import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import PPO

import mujoco


class PickUpArm(gym.Env):
    def __init__(self, robot_path):
        super().__init__()
        self.robot_path = robot_path
        self.model = mujoco.MjModel.from_xml_path(robot_path)
        self.data = mujoco.MjData(self.model)

        self.box_name = "box"
        self.end_effector_name = "hand"

        self.box_id = self.model.body(self.box_name).id
        self.end_effector = self.model.body(self.end_effector_name).id

    def process_action(self):
        hi = self.model.actuator_ctrlrange

    def get_obs(self):
        box_position = self.data.body(self.box_id).xpos
        obs = np.hstack([self.data.qpos])

    def step(self, action):

        self.data.ctrl = ...
        mujoco.mj_step(self.model, self.data)

    def reset(self):
        pass

    def reward(self):
        pass    

    def truncated(self):
        pass

    def terminated(self):
        pass


if __name__ == "__main__":
    robot_path = '/Users/timii/Developer/repositories/mujoco_menagerie/franka_emika_panda/mjx_single_cube.xml'

    env = PickUpArm(robot_path)