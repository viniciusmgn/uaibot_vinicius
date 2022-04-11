from uaibot import *
from scipy.linalg import null_space
import numpy as np


sim = Demo.control_demo_1()
sim.set_parameters(load_screen_color="#191919", width=1000, height=550)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images","democontrole1")

sim = Demo.control_demo_2()
sim.set_parameters(load_screen_color="#191919", width=1000, height=550)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images","democontrole2")

sim = Demo.control_demo_3()
sim.set_parameters(load_screen_color="#191919", width=1000, height=550)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images","democontrole3")