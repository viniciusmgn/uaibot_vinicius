import gym
from gym import spaces
from simulation import *
import numpy as np
from utils import *



class RobotEnv(gym.Env):
    """Robot Environment that follows gym interface"""

    metadata = {'render.modes': ['human']}

    def __init__(self, robot, obstacle, mthd, minqdot=-5, maxqdot=5):

        n = len(robot.links)
        self.robot = robot
        self.mthd = mthd
        self.obstacle = obstacle
        self.oldDistStruct = None
        self.h = 0.001
        self.current_step = 0
        self.last_envInfo = np.zeros((12,))
        self.last_action = np.zeros((n,))
        self.homeq = robot.q

        self.reward_range = (-30, 30)
        # Create action space (joint speed)
        self.action_space = spaces.Box(
            low=minqdot * np.ones((n,)), high=maxqdot * np.ones((n,)), dtype=np.float16)

        self.observation_space = spaces.Box(
            low=-10, high=10, shape=(24 + n,), dtype=np.float16)

    def next_observation(self):
        n = len(self.robot.links)
        r = self.robot.task_function(self.mthd)[0]
        ds = self.robot.compute_dist(obj=self.obstacle, h=self.h, g=self.h, old_dist_struct=self.oldDistStruct)
        obs = np.zeros((24 + n,))
        envInfo = np.zeros((12,))

        envInfo[0:6] = r
        envInfo[6:9] = ds.get_closest_item()["pObjCol"].reshape((3,))
        envInfo[9:12] = ds.get_closest_item()["pObj"].reshape((3,))

        obs[0:12] = self.last_envInfo
        obs[12:24] = envInfo
        obs[24:24 + n] = self.last_action

        self.last_envInfo = envInfo

        return obs

    def reward(obs):
        kTaskPos = 10
        kTaskOri = 10
        kObs = 3

        rTaskPos = kTaskPos * (np.linalg.norm(obs[0:3]) - np.linalg.norm(obs[12:15]))
        rTaskOri = kTaskOri * (np.linalg.norm(obs[3:6]) - np.linalg.norm(obs[15:18]))
        # rObs = -kObs*(np.linalg.norm(obs[6:9])-np.linalg.norm(obs[15:18]))

        if max(abs(obs[12:18])) <= 0.01:
            print("Task achieved")
            return 10
        else:
            if np.linalg.norm(obs[18:21] - obs[21:24]) <= 0.005:
                print("Collision")
                return -10
            else:
                return rTaskPos + rTaskOri

    def isDone(obs):
        return np.linalg.norm(obs[18:21] - obs[21:24]) <= 0.005 or max(abs(obs[12:18])) <= 0.01

    def step(self, action):
        # Execute one time step within the environment
        self.current_step += 1
        dt = 0.01
        self.robot.set_ani_frame(self.robot.q + action * dt)
        self.last_action = action
        obs = self.next_observation()

        return obs, RobotEnv.reward(obs), RobotEnv.isDone(obs), {}

    def reset(self):
        n = len(self.robot.links)

        found_config = False

        while not found_config:
            q = np.random.uniform(0,6.28,(6,))
            dist = self.robot.compute_dist(q = q, obj=self.obstacle).get_closest_item()["hgDistance"]
            found_config = (dist>0.02)


        self.robot.set_ani_frame(q)
        self.current_step = 0
        self.last_task = np.zeros((9,))
        self.last_action = np.zeros((n,))

        obs = self.next_observation()
        print(np.linalg.norm(obs[18:21] - obs[21:24]))
        return obs

    def render(self, mode='human', close=False):
        # Render the environment to the screen
        print("Current task status:")
        print(self.last_task)
        print("Current configuration:")
        print(self.robot.q)
