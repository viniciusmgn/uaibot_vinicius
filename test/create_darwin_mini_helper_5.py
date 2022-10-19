from uaibot import *
import numpy as np

from robot._create_darwin_mini_arm import _create_darwin_mini_arm
from robot._create_darwin_mini_leg_left import _create_darwin_mini_leg_left
from robot._create_darwin_mini_leg_right import _create_darwin_mini_leg_right

robot = Robot.create_darwin_mini()
ball = Ball(htm=Utils.trn([0.4,0,0.6]), radius=0.15, color="yellow")

right_arm = robot.list_of_objects[0]
left_arm = robot.list_of_objects[1]

right_arm.add_ani_frame(0,q=[ 0.50289982, -0.12686379, -0.0784082])
left_arm.add_ani_frame(0, q=[ 2.63894549, -0.12686379, -0.0784082])
#right_arm.attach_object(ball)

sim = Simulation.create_sim_factory([robot, ball])

htm_right_1 = Utils.trn([0.4,0.15,0.6])
htm_left_1 = Utils.trn([0.4,-0.15,0.6])
dt=0.01
d_arms = 0.3

radius_mov = 0.05
omega_mov = 2*np.pi/3
s_ball_des = lambda tt: np.matrix(
    [0.4, radius_mov * np.cos(omega_mov * tt), 0.6 + radius_mov * np.sin(omega_mov * tt)]).reshape((3, 1))
s_ball_des_der = lambda tt: np.matrix(
    [0, -omega_mov * radius_mov * np.sin(omega_mov * tt), omega_mov * radius_mov * np.cos(omega_mov * tt)]).reshape(
    (3, 1))

s_0 = right_arm.fkm()[0:3, 3] - left_arm.fkm()[0:3, 3]

def fun_F(r):
    A = 0.25
    w_tol = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
    F = np.matrix(np.zeros((4, 1)))
    for i in range(4):
        if abs(r[i, 0]) < w_tol[i]:
            F[i, 0] = -A * (r[i, 0] / w_tol[i])
        elif r[i, 0] >= w_tol[i]:
            F[i, 0] = -A
        else:
            F[i, 0] = A
    return F

for i in range(1000):
    t = i*dt

    Jg_R, fk_R = right_arm.jac_geo()
    Jg_L, fk_L = left_arm.jac_geo()

    s_R = fk_R[0:3, 3]
    s_L = fk_L[0:3, 3]

    r = np.matrix(np.zeros((6,1)))
    r[0:3,0] = s_R-s_L-s_0
    r[3:6,0] = (s_R+s_L)/2 - s_ball_des(t)

    Jr = np.matrix(np.zeros((6,6)))
    Jr[0:3, 0:3] = Jg_R[0:3, :]
    Jr[0:3, 3:6] = -Jg_L[0:3, :]
    Jr[3:6, 0:3] = Jg_R[0:3, :] / 2
    Jr[3:6, 3:6] = Jg_L[0:3, :] / 2

    ff = np.matrix(np.zeros((6,1)))
    ff[3:6,0] = -s_ball_des_der(t)

    qdot = Utils.dp_inv(Jr) * (-r-ff)

    qdot_R = qdot[0:3, 0]
    qdot_L = qdot[3:6, 0]

    qprox_R = right_arm.q + qdot_R * dt
    qprox_L = left_arm.q + qdot_L * dt

    right_arm.add_ani_frame(time=t,q=qprox_R)
    left_arm.add_ani_frame(time=t, q=qprox_L)
    ball.add_ani_frame(t, htm=Utils.trn((s_R + s_L)/2) )

    #print(np.linalg.norm(s_R-s_L))
    #print(np.linalg.norm((s_R + s_L)/2 -  s_ball_des(t)))






sim.save("D:\\","entire")