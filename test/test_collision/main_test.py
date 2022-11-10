from typing import List

import uaibot as ub
import numpy as np

from utils import *
import robot as rb
from cvxopt import matrix, solvers
from graphics.meshmaterial import *
import sys
import time
import plotly.express as px
import plotly.graph_objects as go
from show_slice import *
from control_collision import *
from uaibot import Box

x = [1,2,3,4,5]
y=[-1,0,-2,3,4]



robot = ub.Robot.create_kuka_lbr_iiwa()
robot_copy = ub.Robot.create_kuka_lbr_iiwa()

texture_wall = ub.Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

material_wall = ub.MeshMaterial(texture_map=texture_wall, roughness=1, metalness=1)

wall1 = ub.Box(name="wall1", htm=Utils.trn([0.3, -0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
            mesh_material=material_wall)
wall2 = ub.Box(name="wall2", htm=Utils.trn([0.3, 0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
            mesh_material=material_wall)
wall3 = ub.Box(name="wall3", htm=Utils.trn([0.3, 0, 0.25]), width=0.05, depth=0.4, height=0.5,
            mesh_material=material_wall)
wall4 = ub.Box(name="wall4", htm=Utils.trn([0.3, 0, 1.15]), width=0.05, depth=0.4, height=0.5,
            mesh_material=material_wall)
wall5 = ub.Box(name="wall5", htm=Utils.trn([0, 0, -0.5]), width=4.5, depth=4.5, height=0.98,
            opacity=0)


obstacles: list[Box] = [wall1, wall2, wall3, wall4, wall5]

pose_tg = Utils.trn([0.5, -0.3, 0.7]) @ Utils.rotx(3.14 / 2)
#pose_tg = Utils.trn([0.5, -0.5, 0.7]) @ Utils.rotx(3.14 / 2)


# Create simulation
sim = ub.Simulation.create_sim_factory([robot, wall1, wall2, wall3, wall4])
sim.add(ub.Frame(htm=pose_tg))

dt=0.03
q = robot.q
dict_dist_struct = None

t=0
hist_r = np.matrix(np.zeros((1,0)))
hist_q = np.matrix(np.zeros((7,0)))
N=round(4/dt)

t_last=0
ok=True
mode_temp = False
pose_tg_now = pose_tg


####
#i1=0
#i2=1

#xv = round(180*robot.q[i1,0]/np.pi)
#yv = round(180*robot.q[i2,0]/np.pi)

#mat, mat_obs, fig = show_slice(robot_copy, obstacles, robot.q, i1, i2, pose_tg, 70)


###



while ok and t<150:

    qdot, cont_ok, r, dict_dist_struct = robot.const_control(htm_des=pose_tg_now, obstacles=obstacles, dict_old_dist_struct=dict_dist_struct, max_dist_obs=0.15)
    q+= qdot*dt
    robot.add_ani_frame(t,q)
    t+=dt

    hist_r = np.block([hist_r, np.linalg.norm(r)])
    hist_q = np.block([hist_q, robot.q])

    print("")
    print("Time:" + str(round(t, 3)))
    print("Temporary task" if mode_temp else "Task")
    print("task:" + str(round(np.linalg.norm(r), 3)))


    if t-t_last>4:
        dev_r = round(np.std(np.log(hist_r[0,-1 - N:-1])),5)
        dev_q = round(np.max(np.std(hist_q[:, -1 - N:-1], axis=1)),5)

        print("dev r:" + str(round(dev_r, 3)))
        print("dev q:" + str(round(dev_q, 3)))


        if  np.linalg.norm(r)<0.01:

            if not mode_temp:
                print("Converged to task")
                ok = False
            else:
                print("Converged to temporary task")
                mode_temp = False
                pose_tg_now = pose_tg
                t_last = t

        if dev_r < 0.003 and dev_q < 0.05 and np.linalg.norm(r)>0.01:

            if not mode_temp:
                print("Converged to local minima of task")
                mode_temp = True
                pose_tg_now = robot.fkm() * ub.Utils.htm_rand(1,0.5)
                ok =  False
            else:
                print("Converged to local minima of temporary task")
                mode_temp = False
                pose_tg_now = pose_tg

            t_last=t



###
#i1, i2 = findmoveconfig(robot, robot.q, obstacles, pose_tg[0])
#i1=1
#i2=3

#xv = round(180*robot.q[i1,0]/np.pi)
#yv = round(180*robot.q[i2,0]/np.pi)

#mat, mat_obs, fig = show_slice(robot_copy, obstacles, robot.q, i1, i2, pose_tg, 70)

#fig.add_trace(go.Scatter(x=[xv], y=[yv]))
#fig.show()
###

list_qi1 = []
list_qi2 = []

#
N=50
hint = None



#fig = go.Figure(data=go.Heatmap(z=matdist, x=v, y=v, hoverongaps=False, colorscale='Blues'))
#fig.update_layout(
#    xaxis_title="q"+str(i1+1),
#    yaxis_title="q"+str(i2+1),
#)

#fig.show()


#


dt=0.01
for i in range(0):
    N, d, trued = computenormal(robot, robot.q, obstacles)


    print("True distance: " + str(trued))
    print("Smooth distance: " + str(d))
    print(t)



    Nr = np.matrix(np.zeros(np.shape(N)))
    Nr[i1, 0] = N[i1, 0]
    Nr[i2, 0] = N[i2, 0]
    Nr = Nr/np.linalg.norm(Nr)

    Tr = np.matrix(np.zeros(np.shape(N)))
    Tr[i1, 0] = -Nr[i2, 0]
    Tr[i2, 0] =  Nr[i1, 0]

    #print("Convergent vector: " + str(-Nr))

    G = -0.999*(2/np.pi)*np.arctan(2*d)
    H = np.sqrt(1-G**2)
    qdot = G*Nr+0*H*Tr


    q+= qdot*dt
    robot.add_ani_frame(t,q)
    t+=dt

    list_qi1.append(robot.q[i1,0])
    list_qi2.append(robot.q[i2,0])

#fig.add_trace(go.Scatter(x=list_qi1, y=list_qi2))
#fig.show()

sim.save("D://","test_col_map")



