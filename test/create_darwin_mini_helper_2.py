from uaibot import *
import numpy as np
from robot._create_darwin_mini_arm import _create_darwin_mini_arm


base_3d_obj, links, htm_a, q0, joint_limits = _create_darwin_mini_arm(np.identity(4), "arm_1", "brown", 1)

robot_arm_left = Robot("darwin_mini", links, base_3d_obj, np.identity(4), np.identity(4), q0, True, joint_limits)

sim = Simulation([robot_arm_left],background_color="black",camera_type="orthographic")



sim.save("D:\\","darwin_mini_2")