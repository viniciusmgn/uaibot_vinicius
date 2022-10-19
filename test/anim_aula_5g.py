from uaibot import *
import numpy as np


robot = Robot.create_darwin_mini()
ball = Ball(htm=Utils.trn([0.4,0,0.6]), radius=0.15, color="yellow")

right_arm = robot.list_of_objects[0]
left_arm = robot.list_of_objects[1]

right_arm.add_ani_frame(0,q=[ 0.50289982, -0.12686379, -0.0784082])
left_arm.add_ani_frame(0, q=[ 2.63894549, -0.12686379, -0.0784082])
#right_arm.attach_object(ball)

sim = Simulation.create_sim_factory([robot, ball])

sim.set_parameters(load_screen_color="#191919", width=500, height=500, background_color="#191919")
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula5\\","anim8")