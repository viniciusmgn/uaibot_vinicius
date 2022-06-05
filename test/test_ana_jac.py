from uaibot import *
import numpy as np


robot = Robot.create_kuka_kr5()


dx = np.random.uniform(-0.2,0.2)
dy = np.random.uniform(-0.2,0.2)
dz = np.random.uniform(-0.2,0.2)
ax = np.random.uniform(-0.2,0.2)
ay = np.random.uniform(-0.2,0.2)
az = np.random.uniform(-0.2,0.2)
theta = np.random.uniform(-1,1)

htm_des =  Utils.trn([dx,dy,dz]) * robot.fkm() * Utils.rot([ax,ay,az],theta)

s_d = htm_des[0:3,3]
alpha_d, beta_d, gamma_d = Utils.euler_angles(htm_des)
phi_d = np.matrix([[alpha_d],[beta_d],[gamma_d]])

frame = Frame(htm = htm_des)

sim = Simulation([robot, frame])


dt = 0.01
u_hist = np.matrix(np.zeros((6,0)))
t_hist = []
t = 0

K=1

for i in range(500):
    t += dt

    jac_ana, htm_eef, phi = robot.jac_ana()

    r = np.matrix(np.zeros((6,1)))
    r[0:3,0] = htm_eef[0:3,3]-s_d
    r[3:6,0] = phi-phi_d

    u = Utils.dp_inv(jac_ana, 0.001) * (-K*r)
    u_hist = np.block([u_hist, u])
    qprox = robot.q + u*dt
    robot.add_ani_frame(t,qprox)
    t_hist.append(t)

t+=1
robot.add_ani_frame(t, robot.q0)

for i in range(500):
    t += dt

    r, jac_r = robot.task_function(htm_des)

    r[3:6,0] = np.sqrt(2*r[3:6,0])
    u = Utils.dp_inv(jac_r, 0.001) * (-K*r)
    u_hist = np.block([u_hist, u])
    qprox = robot.q + u*dt
    robot.add_ani_frame(t,qprox)
    t_hist.append(t)


fig = Utils.plot(t_hist, u_hist)
sim.save("D://","test_ana_jac")
