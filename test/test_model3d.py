from uaibot import *
import numpy as np


robot = Robot.create_kuka_kr5()
box = Ball(radius=0.1)

sim = Simulation([robot, box])

box.add_ani_frame(0,htm=robot.fkm())
robot.attach_object(box)

for i in range(300):
    robot.add_ani_frame(i*0.01,q=robot.q + np.matrix([0.01,0.01,0.01,0.01,0.01,0.01]).reshape((6,1)))

robot.detach_object(box)

for i in range(300):
    robot.add_ani_frame(3 + i*0.01,q=robot.q + np.matrix([0.01,0.01,0.01,0.01,0.01,0.01]).reshape((6,1)))




sim.save("D://","test_correia")