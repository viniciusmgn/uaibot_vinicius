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
u_hist1 = np.matrix(np.zeros((6,0)))
u_hist2 = np.matrix(np.zeros((6,0)))
t_hist1 = []
t_hist2 = []
t = 0

K=2
N=300

finished = False


while not finished:
    t += dt

    jac_ana, htm_eef, phi = robot.jac_ana()

    r = np.matrix(np.zeros((6,1)))
    r[0:3,0] = htm_eef[0:3,3]-s_d
    r[3:6,0] = phi-phi_d

    u = Utils.dp_inv(jac_ana, 0.001) * (-K*r)
    u_hist1 = np.block([u_hist1, u])
    qprox = robot.q + u*dt
    robot.add_ani_frame(t,qprox)
    t_hist1.append(t)

    p_error = np.linalg.norm(r[0:3,0])
    a_error = [np.arccos(htm_eef[0:3,j].T * htm_des[0:3,j])*(180/np.pi) for j in range(3)]

    finished = p_error < 0.001 and max(a_error)<3

    print("EULER_ANGLES: p_error = "+str(p_error)+", a_error = "+str(max(a_error)))




t+=1
robot.add_ani_frame(t, robot.q0)
t_settle=t

finished = False
while not finished:
    t += dt

    htm_eef = robot.fkm()
    r, jac_r = robot.task_function(htm_des)

    u = Utils.dp_inv(jac_r, 0.001) * (-2*K*r)
    u_hist2 = np.block([u_hist2, u])
    qprox = robot.q + u*dt
    robot.add_ani_frame(t,qprox)
    t_hist2.append(t-t_settle)

    p_error = np.linalg.norm(r[0:3, 0])
    a_error = [np.arccos(htm_eef[0:3, j].T * htm_des[0:3, j]) * (180 / np.pi) for j in range(3)]

    finished = p_error < 0.001 and max(a_error)<3

    print("VINICIUS: p_error = "+str(p_error)+", a_error = "+str(max(a_error)))

fig1 = Utils.plot(t_hist1, u_hist1)
fig2 = Utils.plot(t_hist2, u_hist2)
sim.save("D://","test_ana_jac")
