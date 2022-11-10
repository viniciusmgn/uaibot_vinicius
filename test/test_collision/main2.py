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


x = [1,2,3,4,5]
y=[-1,0,-2,3,4]


def closestpoints(_robot,_obstacles):

    list_st = [_robot.compute_dist(obs) for obs in _obstacles]

    st_min = []
    distmin = 1000

    for st in list_st:
        if st.get_closest_item().distance < distmin:
            distmin = st.get_closest_item().distance
            st_min = st.get_closest_item()

    return st_min

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


obstacles = [wall1, wall2, wall3, wall4, wall5]

#pose_tg = Utils.trn([0.5, -0.3, 0.7]) @ Utils.rotx(3.14 / 2)
pose_tg = Utils.trn([0.5, -0.5, 0.7]) @ Utils.rotx(3.14 / 2)


# Create simulation
sim = ub.Simulation.create_sim_factory([robot, wall1, wall2, wall3, wall4])

b_robot = ub.Ball(radius=0.008, color="red")
b_obs = ub.Ball(radius=0.008, color="blue")


sim.add(ub.Frame(htm=pose_tg))
sim.add(b_robot)
sim.add(b_obs)

dt=0.03
q = robot.q
dict_dist_struct = None

t=0
hist_r = np.matrix(np.zeros((1,0)))
hist_q = np.matrix(np.zeros((7,0)))
N=round(4/dt)

t_last=0
ok=True
pose_tg_now = pose_tg


tree = None
while ok and t<150:

    qdot, cont_ok, r, dict_dist_struct, _, _ = robot.const_control(htm_des=pose_tg_now, obstacles=obstacles, dict_old_dist_struct=dict_dist_struct, max_dist_obs=0.15)
    q+= qdot*dt
    robot.add_ani_frame(t,q)
    t+=dt

    hist_r = np.block([hist_r, np.linalg.norm(r)])
    hist_q = np.block([hist_q, robot.q])

    print("")
    print("Time:" + str(round(t, 3)))
    print("task:" + str(round(np.linalg.norm(r), 3)))


    if t-t_last>4:
        dev_r = round(np.std(np.log(hist_r[0,-1 - N:-1])),5)
        dev_q = round(np.max(np.std(hist_q[:, -1 - N:-1], axis=1)),5)

        print("dev r:" + str(round(dev_r, 3)))
        print("dev q:" + str(round(dev_q, 3)))

        if  np.linalg.norm(r)<0.01:
            print("Converged to task")

        if dev_r < 0.003 and dev_q < 0.05 and np.linalg.norm(r)>0.01:
            print("Converged to local minima of task")
            ok =  False

k=0
dt=0.01
while t<150 and k<600:

    _, _, _, dict_dist_struct, A, b = robot.const_control(htm_des=pose_tg_now, obstacles=obstacles, dict_old_dist_struct=dict_dist_struct, max_dist_obs=0.15)
    st_min = closestpoints(robot,obstacles)
    qdot_N = st_min.jac_distance.T

    d = st_min.distance

    qdot_T = qdot_N.copy()
    qdot_T[0, 0] = -qdot_T[0, 0]
    qdot_T[2, 0] = -qdot_T[2, 0]
    qdot_T[4, 0] = -qdot_T[4, 0]
    qdot_T[6, 0] = 0

    g = -0.99 * (2/np.pi) * np.arctan(300 * (d))
    h = sqrt(1-g**2)
    #qdot_tg = 10*g * qdot_N + h * qdot_T
    qdot_tg = 10* (g * qdot_N + h * qdot_T)

    H = 2 * np.identity(7)
    f = - 2 * qdot_tg

    qdot = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b))['x']

    q+= qdot*dt

    b_robot.add_ani_frame(t,htm=ub.Utils.trn(st_min.point_link))
    b_obs.add_ani_frame(t,htm=ub.Utils.trn(st_min.point_object))
    robot.add_ani_frame(t, q)
    t+=dt
    k+=1
    print("Tangent: "+str(k))
    print("Distance: "+str(round(d,4)))
    print("")


sim.save("D://","test_col_map")



