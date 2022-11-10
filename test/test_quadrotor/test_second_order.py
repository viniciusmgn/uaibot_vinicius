import uaibot as ub
import numpy as np
import scipy.linalg
from control_target_2 import *
from generate_env import *
from ispathfree import *

from scipy.linalg import expm, null_space

texture_gold = ub.Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/gold_metal.png',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

material_robot  = ub.MeshMaterial(texture_map=texture_gold, roughness=1, metalness=1)

dt=0.005

#Create robot
robot = ub.SmoothBox(htm = ub.Utils.trn([0,0,0.3]), width=0.4,depth=0.4,height=0.15,color="red", mesh_material=material_robot)
robot_next = ub.SmoothBox(width=0.4, depth=0.4, height=0.15)
radius_robot = np.sqrt(3)*0.4/2

v = 0 * np.matrix(np.random.uniform(-1,1,(3,1)))
w = np.matrix(np.random.uniform(-1,1,(3,1)))
p = robot.htm[0:3,-1]
Q = robot.htm[0:3,0:3]

p_target = np.matrix([1.225, 1.225, 1.4]).T

for i in range(round(30/dt)):

    t=round(i*dt,3)

    p = p + v * dt
    Q = scipy.linalg.expm(ub.Utils.S(dt * w)) * Q
    htm_new = np.block([[Q, p], [np.matrix([0, 0, 0, 1]).reshape((1, 4))]])
    robot_next.add_ani_frame(t, htm = htm_new)


    T, dotw, error, dist_min =  control_target_2(v, w, robot, robot_next, radius_robot, [], [], p_target)

    if not error:

        z0 = np.matrix([[0],[0],[0]])
        z = robot.htm[0:3,2]

        v = v + 0*(-9.8 * z0 + T * z) * dt

        w = w + dotw * dt

        robot.add_ani_frame(t, htm_new)

        print("Time: "+str(t)+" distance:"+str(round( np.linalg.norm(p-p_target), 4)))

    else:
        print("Error!")





