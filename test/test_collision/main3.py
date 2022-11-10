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

#Initialize simulation

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

mode_task=True



fun_col = lambda qf: robot.check_free_configuration(q=qf, obstacles=obstacles)[0]

def fun_eval_gen(qf, qc):
    rc = np.linalg.norm(robot.task_function(q=qc, htm_des = pose_tg_now)[0])
    rf = np.linalg.norm(robot.task_function(q=qf, htm_des = pose_tg_now)[0])

    if rf<rc:
        return 0
    else:
        dp = np.linalg.norm(robot.fkm(qf, axis='eef')[0:3,-1] - robot.fkm(qc, axis='eef')[0:3,-1])
        return rf/(dp+0.001)

def fun_col(qf):

    htm_box = ub.Utils.trn(qf[0:3,0]) * ub.Utils.rotx(qf[3,0]) * ub.Utils.roty(qf[4,0]) * ub.Utils.rotz(qf[5,0])
    box_col = ub.Box(htm = htm_box, width=0.15, depth=0.15, height=0.15)

    free=True
    kf=0

    while free and kf<len(obstacles):
        if ub.Utils.compute_aabbdist(box_col, obstacles[kf])==0:
            free = ub.Utils.compute_dist(box_col, obstacles[kf])[2] > 0.01

        kf += 1

    return free

def fun_dist(qf1, qf2):

    htm_1 = ub.Utils.trn(qf1[0:3,0]) * ub.Utils.rotx(qf1[3,0]) * ub.Utils.roty(qf1[4,0]) * ub.Utils.rotz(qf1[5,0])
    htm_2 = ub.Utils.trn(qf2[0:3,0]) * ub.Utils.rotx(qf2[3,0]) * ub.Utils.roty(qf2[4,0]) * ub.Utils.rotz(qf2[5,0])

    return np.linalg.norm(htm_1-htm_2)

def fun_eval(qf):

    htm_box = ub.Utils.trn(qf[0:3, 0]) * ub.Utils.rotx(qf[3, 0]) * ub.Utils.roty(qf[4, 0]) * ub.Utils.rotz(qf[5, 0])
    return np.linalg.norm(htm_box[0:3,-1]-pose_tg_now[0:3,-1])


q_lim_inf = np.matrix([-1.5,-1.5,   0,-np.pi,-np.pi,-np.pi]).reshape((6,1))
q_lim_sup = np.matrix([ 1.5, 1.5, 1.5, np.pi, np.pi, np.pi]).reshape((6,1))
htm_0 = robot.fkm()
qb0 = np.matrix([htm_0[0,3], htm_0[1,3], htm_0[2,3],0, 0,0]).reshape((6,1))

tree = ub.RRTTree(qb0, fun_eval, fun_col, q_lim_inf, q_lim_sup, fun_dist, min_dist=0.03, step_size = 0.5)
for i in range(500):
    tree.add_node_tree(1)
    print("Node RRT i: "+str(i))

nsv = tree.node_smallest_value()

list_pose = [ub.Utils.trn(nod.q[0:3, 0]) * ub.Utils.rotx(nod.q[3, 0]) * ub.Utils.roty(nod.q[4, 0]) * ub.Utils.rotz(nod.q[5, 0]) for
             nod in nsv.path_to_root()]
list_pose.append(pose_tg)
ind_task=0

#for pose in list_pose:
#    sim.add(ub.Frame(htm=pose))
#    sim.add(ub.Box(htm=pose, width=0.15, depth=0.15, height=0.15, color="yellow", opacity=0.3))

##
list_pose = [ub.Utils.trn(nod.q[0:3, 0]) * ub.Utils.rotx(nod.q[3, 0]) * ub.Utils.roty(nod.q[4, 0]) * ub.Utils.rotz(nod.q[5, 0]) for
             nod in tree.list_node]

for pose in list_pose:
    sim.add(ub.Frame(htm=pose))
    sim.add(ub.Box(htm=pose, width=0.15, depth=0.15, height=0.15, color="yellow", opacity=0.3))

sim.save("D://", "test_col_map")

while ok and t<150:

    qdot, cont_ok, r, dict_dist_struct, _, _ = robot.const_control(htm_des=list_pose[ind_task], obstacles=obstacles,
                                                                   dict_old_dist_struct=dict_dist_struct,
                                                                   max_dist_obs=0.15)
    q += qdot * dt
    robot.add_ani_frame(t, q)
    t += dt

    hist_r = np.block([hist_r, np.linalg.norm(r)])
    hist_q = np.block([hist_q, robot.q])

    print("")
    print("MODE: TASK FOLLOWING")
    print("Time:" + str(round(t, 3)))
    print("task:" + str(round(np.linalg.norm(r), 3)))
    print("index:" + str(ind_task))

    if t - t_last > 4:
        dev_r = round(np.std(np.log(hist_r[0, -1 - N:-1])), 5)
        dev_q = round(np.max(np.std(hist_q[:, -1 - N:-1], axis=1)), 5)

        print("dev r:" + str(round(dev_r, 3)))
        print("dev q:" + str(round(dev_q, 3)))

        if np.linalg.norm(r) < 0.01:
            print("Converged to task")
            ind_task+=1
            t_last = t
            sim.save("D://", "test_col_map")

        if dev_r < 0.003 and dev_q < 0.05 and np.linalg.norm(r) > 0.01:
            print("Converged to local minima of task")
            ind_task += 1
            t_last = t
            sim.save("D://", "test_col_map")



sim.save("D://","test_col_map")



