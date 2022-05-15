from uaibot import *
import numpy as np

from robot._create_darwin_mini_arm import _create_darwin_mini_arm
from robot._create_darwin_mini_leg_left import _create_darwin_mini_leg_left
from robot._create_darwin_mini_leg_right import _create_darwin_mini_leg_right

robot = Robot.create_darwin_mini()

#param = _create_darwin_mini_leg_right(np.identity(4),"a","white",1)
#robot = Robot("aaaa", param[1], param[0],
#              np.identity(4), param[2],
#              param[3], param[4], True,
#              param[5])

#robot.add_ani_frame(0,q = np.random.uniform(0,1,size=(4,1)))
#htm_eef= robot.fkm()
#print(robot.q)

frame1 = Frame(name="f1", htm=robot.list_of_objects[2].fkm(), size=0.2)
frame2 = Frame(name="f2", htm=robot.list_of_objects[3].fkm(), size=0.2)
sim = Simulation.create_sim_factory([robot, frame1, frame2])

print(robot.list_of_objects[2].fkm())

print()


sim.save("D:\\","entire")