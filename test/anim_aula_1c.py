from uaibot import *
from scipy.linalg import null_space
import numpy as np


sim = Demo.control_demo_1()
sim.set_parameters(load_screen_color="#191919", width=800, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula1","democontrole1")

sim = Demo.control_demo_2()
sim.set_parameters(load_screen_color="#191919", width=800, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula1","democontrole2")

sim = Demo.control_demo_3()
sim.set_parameters(load_screen_color="#191919", width=800, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula1","democontrole3")