from uaibot import *
import numpy as np

np.set_printoptions(precision=4, linewidth=90, suppress=True)

robot1 = Robot.create_kuka_kr5()
robot2 = Robot.create_kuka_lbr_iiwa()

sim = Demo.control_demo_1()

sim.save("D:\\","demo1")
