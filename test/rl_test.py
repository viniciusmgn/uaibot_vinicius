from uaibot import *
import numpy as np
from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *
from simobjects.pointlight import *
from simobjects.pointcloud import *
from simobjects.frame import *
from utils import *
import robot as rb
from cvxopt import matrix, solvers
from graphics.meshmaterial import *
import sys
import time



solvers.options['show_progress'] = False
np.set_printoptions(precision=4, suppress=True, linewidth=150)

robot = rb.Robot.create_kuka_lbr_iiwa()

texture_wall = Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

material_wall = MeshMaterial(texture_map=texture_wall, roughness=1, metalness=1)

wall1 = Box(name="wall1", htm=Utils.trn([0.3, -0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
            mesh_material=material_wall)
wall2 = Box(name="wall2", htm=Utils.trn([0.3, 0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
            mesh_material=material_wall)
wall3 = Box(name="wall3", htm=Utils.trn([0.3, 0, 0.25]), width=0.05, depth=0.4, height=0.5,
            mesh_material=material_wall)
wall4 = Box(name="wall4", htm=Utils.trn([0.3, 0, 1.15]), width=0.05, depth=0.4, height=0.5,
            mesh_material=material_wall)

def evaluate_error(r):
    error_pos = max(abs(r[0:3]))
    error_ori = (180 / np.pi) * max(abs(np.arccos(1 - r[3:6])))
    ok1 = error_pos < 0.005
    ok2 = error_ori < 5 if len(r.tolist()) > 3 else True

    return ok1 and ok2, error_pos, error_ori

def dist_computation(q, old_struct, h):
    dist_wall_1 = robot.compute_dist(obj=wall1, q=q, h=h, g=h, old_dist_struct=old_struct[0])
    dist_wall_2 = robot.compute_dist(obj=wall2, q=q, h=h, g=h, old_dist_struct=old_struct[1])
    dist_wall_3 = robot.compute_dist(obj=wall3, q=q, h=h, g=h, old_dist_struct=old_struct[2])
    dist_wall_4 = robot.compute_dist(obj=wall4, q=q, h=h, g=h, old_dist_struct=old_struct[3])

    struct = [dist_wall_1, dist_wall_2, dist_wall_3, dist_wall_4]

    jac_dist = np.block(
        [[dist_wall_1.jac_dist_mat], [dist_wall_2.jac_dist_mat], [dist_wall_3.jac_dist_mat],
         [dist_wall_4.jac_dist_mat]])
    dist_vect = np.block(
        [[dist_wall_1.dist_vect], [dist_wall_2.dist_vect], [dist_wall_3.dist_vect], [dist_wall_4.dist_vect]])

    return dist_vect, jac_dist, struct

# Target pose definition

# Create simulation
sim = Simulation.create_sim_factory([robot, wall1, wall2, wall3, wall4])

# Parameters
dt = 0.02
h=0.001
Ka=2
Kc=1 #1



q_des = robot.ikm(htm_target = Utils.trn([0.5,0,0.7]) * Utils.roty(np.pi/2))

q = robot.q

t=0

# Initializations
struct = [None, None, None, None]

ok = True

norm_u = []
k=0
while ok:


    dist_vect, jac_dist, struct = dist_computation(q, struct, h)
    A = -jac_dist
    b = Ka * (dist_vect-0.01)
    H = 2 * np.identity(7)
    f = 2 * Kc * (q-q_des)
    u = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']

    #u = u/(np.linalg.norm(u)+0.1)

    q = q + u*dt
    t=t+dt

    robot.add_ani_frame(t,q)

    norm_u.append(np.linalg.norm(u))

    if k>40:
        ok = (k <= 500) and (np.mean(norm_u[-40:])>0.001)

    k += 1


sim.save("D://","test_RL")


