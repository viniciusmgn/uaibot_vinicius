# %%
import sys
# insert at 1, 0 is the script path (or '' in REPL)
import numpy as np

sys.path.insert(1, 'D:/PycharmProjects/UAIbot/uaibot')

import robot as rb
from simobjects.frame import Frame
from simulation import Simulation
robot = rb.Robot.create_davinci(opacity=0.5, eef_frame_visible=False)
robot_arm = robot.list_of_objects[0]
junta = 3
r = [0, 0, 0, 0, 0, 0, 0]

sim = Simulation.create_sim_grid([robot_arm])

list_frames = []
for j in range(7):
    list_frames.append( Frame(name="frame"+str(j), htm=robot_arm.fkm(axis="dh")[j], size=0.25))
    sim.add(list_frames[-1])


#frame2 = Frame(htm=robot_arm.fkm(axis='dh')[junta-2], size=0.25, name='frame2')

#sim.run()

t=0
for j in range(7):
    r = [0, 0, 0, 0, 0, 0, 0]
    for i in range(400):
        r[j] = (2*np.pi)*i/399
        t+=0.01
        robot_arm.add_ani_frame(t, r)
        list_frames[j].add_ani_frame(t, htm=robot_arm.fkm(axis="dh")[j])

sim.save("D://teste_felipe//","felipe")

