import distutils.debug

from uaibot import *
from scipy.linalg import null_space
import numpy as np


#Animação da junta revolta

robot = Robot.create_abb_crb(htm=Utils.rotz(3.14/2))
sim = Simulation([robot], load_screen_color="#191919", show_grid=False, show_world_frame=False, background_color="#191919",
                 camera_type="orthographic", width=500, height=400, camera_start_pose=[1,0.1,0.5,-3.14,3.14/2,-3.14/2,4])

joint = Box(htm = robot.fkm(axis='dh')[0], color="blue", width=0.05, depth=0.05, height=0.4, opacity=0.5)
joint_cylinder = Cylinder(htm = robot.fkm(axis='dh')[0], color="magenta", radius=0.05, height=0.35, opacity=0.5)

sim.add(joint)
sim.add(joint_cylinder)

dt=0.01
q0 = robot.q

htm0 = robot.fkm(axis='dh')[0]
for i in range(3000):
    robot.add_ani_frame(i*dt, q=q0+np.array([0,0.6*cos(i*dt),0,0,0,0]).reshape((6,1)))
    joint.add_ani_frame(i*dt, htm = htm0 @ Utils.rotz(0.6*cos(i*dt)))


sim.save("D:\\HTML Simulations","junta_revoluta_robo")

#Animação da junta prismática

robot = Robot.create_epson_t6(htm=Utils.rotz(3.14/2), color="gray")
sim = Simulation([robot], load_screen_color="#191919", show_grid=False, show_world_frame=False, background_color="#191919",
                 camera_type="orthographic", width=500, height=400, camera_start_pose=[1,0.1,0.35,-3.14,3.14/2,-3.14/2,4])

joint = Box(htm = robot.fkm(axis='dh')[2], color="blue", width=0.05, depth=0.05, height=0.4, opacity=0.5)
joint_cylinder = Cylinder(htm = robot.fkm(axis='dh')[2]@ Utils.trn([0,0,-0.1]), color="magenta", radius=0.05, height=0.25, opacity=0.5)

sim.add(joint)
sim.add(joint_cylinder)

dt=0.01
q0 = robot.q

htm0 = robot.fkm(axis='dh')[2]
for i in range(3000):
    robot.add_ani_frame(i*dt, q=q0+np.array([0,0,0.095+0.075*cos(i*dt)]).reshape((3,1)))
    joint.add_ani_frame(i*dt, htm =  htm0 @ Utils.trn([0,0,0.095+0.075*cos(i*dt)-0.2]))


sim.save("D:\\HTML Simulations","junta_prismatica_robo")

#Animação do robô de seis graus de liberdade

robot = Robot.create_abb_crb(htm=Utils.rotz(3.14/2), opacity=0.5)
sim = Simulation([robot], load_screen_color="#191919", show_grid=False, show_world_frame=False, background_color="#191919",
                 camera_type="orthographic", width=500, height=400)

joints = []
joints_cylinder = []

joints.append(Box(name="joint" + str(i), htm=robot.htm_base_0, color="blue", width=0.03, depth=0.03, height=0.25,
                  opacity=0.5))
joints_cylinder.append(
    Cylinder(name="jointCylinder" + str(i), htm=robot.htm_base_0, color="magenta", radius=0.03, height=0.15,
             opacity=0.5))
sim.add(joints[-1])
sim.add(joints_cylinder[-1])


for i in range(len(robot.links)):
    joints.append(Box(name="joint"+str(i), htm = robot.fkm(axis='dh')[i], color="blue", width=0.03, depth=0.03, height=0.25, opacity=0.5))
    joints_cylinder.append(Cylinder(name="jointCylinder"+str(i), htm = robot.fkm(axis='dh')[i], color="magenta", radius=0.03, height=0.15, opacity=0.5))
    sim.add(joints[-1])
    sim.add(joints_cylinder[-1])

sim.save("D:\\HTML Simulations","robo_seis_graus")

#Animação do robô de três graus de liberdade

robot = Robot.create_epson_t6(htm=Utils.rotz(3.14/2), color="gray", opacity=0.5)
sim = Simulation([robot], load_screen_color="#191919", show_grid=False, show_world_frame=False, background_color="#191919",
                 camera_type="orthographic", width=500, height=400, camera_start_pose=[1,0.1,0.35,-3.14,3.14/2,-3.14/2,4])

joints = []
joints_cylinder = []

joints.append(Box(name="joint" + str(i), htm=Utils.rotz(3.14/2) @ robot.htm_base_0, color="blue", width=0.03, depth=0.03, height=0.25,
                  opacity=0.5))
joints_cylinder.append(
    Cylinder(name="jointCylinder" + str(i), htm=Utils.rotz(3.14/2) @ robot.htm_base_0, color="magenta", radius=0.03, height=0.15,
             opacity=0.5))
sim.add(joints[-1])
sim.add(joints_cylinder[-1])




for i in range(len(robot.links)):
    joints.append(Box(name="joint"+str(i), htm = robot.fkm(axis='dh')[i], color="blue", width=0.03, depth=0.03, height=0.25, opacity=0.5))
    joints_cylinder.append(Cylinder(name="jointCylinder"+str(i), htm = robot.fkm(axis='dh')[i], color="magenta", radius=0.03, height=0.15, opacity=0.5))
    sim.add(joints[-1])
    sim.add(joints_cylinder[-1])

sim.save("D:\\HTML Simulations","robo_tres_graus")

#Sistemas com restrições:

robot = Robot.create_abb_crb(htm=Utils.rotz(3.14/2))
sim = Simulation([robot], load_screen_color="#191919", background_color="#191919",
                 camera_type="orthographic", width=500, height=400, camera_start_pose=[1,0.1,0.5,-3.14,3.14/2,-3.14/2,2])

htm_target = Utils.rotx(3.14) @ Utils.trn([0,0.5,0])
q0 = robot.ikm(htm_target=htm_target)




robot.add_ani_frame(0,q=q0)
dt=0.005
for i in range(3000):
    if i % 100 == 0:
        direction = np.random.uniform(-1,1,size=(3,1))

    r, jac_r = robot.task_function(htm_des=htm_target, q=robot.q)
    r = np.array([[r[2, 0]], [sqrt(r[3, 0])], [sqrt(r[4, 0])]])
    jac_r = np.block([[jac_r[2, :]], [jac_r[3:5, :]]])

    u = -3 * Utils.dp_inv(jac_r) @ r + null_space(jac_r) @ direction
    robot.add_ani_frame(i * dt, robot.q + u * dt)

sim.save("D:\\HTML Simulations","juntas_restricao")