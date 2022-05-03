import distutils.config

from uaibot import *
from scipy.linalg import null_space
import numpy as np



#Animação da junta revolta


sim = Simulation([], load_screen_color="#191919", background_color="#191919", width=500, height=500,
                 camera_type="orthographic")

robot = Robot.create_kuka_kr5()
vector_v = Vector(name="vector_v", color="cyan", thickness=1, origin=[0, 0, 0], vector=[0.5, 0.5, 0.5])
vector_w = Vector(name="vector_w", color="gold", thickness=1, origin=[0, 0, 0], vector=[0.5, 0.5, 0.5])

sim.add([robot, vector_v, vector_w])

dt=0.01
eps=0.001
hist_q=[]
hist_qdot=[]
hist_tempo=[]


for i in range(2000):
    t = i*dt
    hd = np.array([0,0,0.2*cos(t),0,0,0]).reshape((6,1))
    jg, cd = robot.jac_geo()
    qdot = Utils.dp_inv(jg, eps) @ hd

    qprox = robot.q + qdot*dt
    robot.add_ani_frame(t, q = qprox)

    v = hd[0:3, :]
    w = hd[3:6, :]

    vector_v.add_ani_frame(t, origin=cd[0:3, -1], vector=0.8 * v)
    vector_w.add_ani_frame(t, origin=cd[0:3, -1], vector=0.8 * w)

    hist_tempo.append(t)
    hist_q.append(robot.q)
    hist_qdot.append(qdot)


sim.set_parameters(load_screen_color="#191919", width=500, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula4","anim7")

