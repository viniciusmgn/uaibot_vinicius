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



def distfun(_robot, _q, _obstacles, old_list_str = None):

    d = 1000
    list_str= {}
    for obs in _obstacles:
        ds = _robot.compute_dist(obs, _q, None, None if (old_list_str is None) else old_list_str[obs] )
        d = min(d, np.min(ds.dist_vect))
        list_str[obs] = ds

    return d, list_str

def freefun(_robot, _q, _obstacles, hint):

    _robot.add_ani_frame(0, q=_q)
    _robot.update_col_object(0)

    #Try the hint:
    collision = False
    if not (hint is None):
        link_num = hint[0]
        col_obj_num = hint[1]
        obs_num = hint[2]

        _, _, d = ub.Utils.compute_dist(_robot.links[link_num].col_objects[col_obj_num][0], _obstacles[obs_num])

        collision = d < 0.001

    if not collision:
        list_try = []

        for link_num in range(np.shape(_robot.q)[0]):
            for col_obj_num in range(len(_robot.links[link_num].col_objects)):
                for obs_num in range(len(_obstacles)):
                    list_try.append([link_num, col_obj_num, obs_num])

        hint = None
        k = 0
        while not collision and k < len(list_try):
            link_num = list_try[k][0]
            col_obj_num = list_try[k][1]
            obs_num = list_try[k][2]
            _, _, d = ub.Utils.compute_dist(_robot.links[link_num].col_objects[col_obj_num][0], _obstacles[obs_num])
            collision = d < 0.001

            if collision:
                hint = list_try[k]

            k += 1

    return collision, hint


def generatedp_uniform(_robot, _obstacles, q_base, indA, indB):

    N = 50
    old_dist = None

    mat = np.matrix(np.zeros((N,N)))
    v = []

    for a in range(N):
        v.append( round(-180 + 360*float(a)/float(N-1)) )

    for a in range(N):
        print(a)
        for b in range(N):
            _q = q_base.copy()
            _q[indA, 0] = -np.pi + 2*np.pi*float(a)/float(N-1)
            _q[indB, 0] = -np.pi + 2*np.pi*float(b)/float(N-1)
            d, old_dist = distfun(_robot, _q, _obstacles, old_dist)
            mat[a,b] = d

    return v, mat

def generatefree_uniform(_robot, _obstacles, q_base, indA, indB):

    N = 50
    hint = None

    mat = np.matrix(np.zeros((N,N)))
    v = []

    for a in range(N):
        v.append( round(-180 + 360*float(a)/float(N-1)) )

    for a in range(N):
        print(a)
        for b in range(N):
            _q = q_base.copy()
            _q[indA, 0] = -np.pi + 2*np.pi*float(a)/float(N-1)
            _q[indB, 0] = -np.pi + 2*np.pi*float(b)/float(N-1)
            notfree, hint = freefun(_robot, _q, _obstacles, hint)
            mat[a,b] = 1 if notfree else 0

    return v, mat


#########################

robot = ub.Robot.create_kuka_lbr_iiwa()

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

pose_tg = []
pose_tg.append(Utils.trn([0.5, -0.3, 0.7]) @ Utils.rotx(3.14 / 2))
#pose_tg.append(Utils.trn([0.5, 0, 0.7]) @ Utils.roty(3.14 / 2))
#pose_tg.append(robot.fkm(axis="eef"))

q_inv = robot.ikm(pose_tg[0])

# Create simulation
sim = ub.Simulation.create_sim_factory([robot, wall1, wall2, wall3, wall4])

for k in range(len(pose_tg)):
    sim.add(ub.Frame(name="pose_tg_" + str(k), htm=pose_tg[k]))

t = 0
q0 = robot.q

q = q0.copy()

for k1 in range(100):
    q[0,0] = -np.pi + 2*np.pi * (k1/100)
    robot.add_ani_frame(t, q=q)
    t += 0.01

q=q0
for k2 in range(100):
    q[1, 0] = -np.pi + 2 * np.pi * (k2 / 100)
    robot.add_ani_frame(t, q=q)
    t += 0.01


sim.save("D://","test_col_map")


#v, mat = generatedp_uniform(robot, [wall1, wall2, wall3, wall4], robot.q, 0, 1)
v, mat = generatefree_uniform(robot, [wall1, wall2, wall3, wall4], robot.q, 0, 1)

fig = go.Figure(data=go.Heatmap(z=mat, x=v, y=v, hoverongaps = False, colorscale='Blues'))
fig.show()











