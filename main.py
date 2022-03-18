from uaibot import *
import numpy as np

robot_creator = Robot.create_abb_crb

#Create the object and put into simulation
robot = robot_creator(opacity=0.5, color="white")
sim = Simulation([robot], camera_type="orthographic")

#Create all objects that will be used during the simulation

x_axis=[]
y_axis=[]
z_axis=[]
far = Utils.trn([1000,1000,1000])

vector_length=0.12

lines_purple = Cylinder(name="linePurple", htm=far, radius=0.002, height=2, color="purple")
lines_orange = Cylinder(name="lineOrange", htm=far, radius=0.002, height=2, color="orange")
points_A = Ball(name="pointA", htm=far, radius=0.015, color="cyan")
points_B = Ball(name="pointB", htm=far, radius=0.015, color="brown")

sim.add(lines_purple)
sim.add(lines_orange)
sim.add(points_A)
sim.add(points_B)

for i in range(len(robot.links)+1):
    x_axis.append(Cylinder(name="xAxis"+str(i), color="red", htm = far, radius=0.003, height=vector_length))
    y_axis.append(Cylinder(name="yAxis"+str(i), color="limegreen", htm = far, radius=0.003, height=vector_length))
    z_axis.append(Cylinder(name="zAxis"+str(i), color="blue", htm = far, radius=0.003, height=vector_length))


    sim.add(x_axis[-1])
    sim.add(y_axis[-1])
    sim.add(z_axis[-1])

#Make the animation
k=0
dt=0.01
htm_dh = robot.fkm(axis="dh")
deltak=200

htm = [robot.htm_base_0]
for i in range(len(robot.links)):
    htm.append(htm_dh[i])

for i in range(len(robot.links)+1):

    if i > 0:
        #Show the line_(i-1)
        lines_orange.add_ani_frame(k * dt, htm=htm[i-1])

    #Show the line_i
    k+=deltak
    lines_purple.add_ani_frame(k*dt, htm = htm[i])

    #Show the pointB_i and pointA_i
    k+=deltak
    if i > 0:
        points_A.add_ani_frame(k*dt, htm = htm[i-1] @ Utils.trn([0, 0, robot.links[i-1].d]))

    points_B.add_ani_frame(k*dt, htm = htm[i])

    #Create the vector x_i
    k += deltak
    x_axis[i].add_ani_frame(k*dt, htm = htm[i] @ Utils.roty(3.14/2) @ Utils.trn([0,0,vector_length/2]))

    #Create the vector z_i
    k += deltak
    z_axis[i].add_ani_frame(k*dt, htm = htm[i] @ Utils.trn([0,0,vector_length/2]))

    #Create the vector y_i
    k += deltak
    y_axis[i].add_ani_frame(k*dt, htm = htm[i] @ Utils.rotx(-3.14/2) @ Utils.trn([0,0,vector_length/2]))

    #Erase the lines and points
    k += 2*deltak


    k+=1



k+=300
dt=0.01
steps=100

for i in range(len(robot.links)):

    q = robot.q0
    for j in range(steps):
        q[i]+= dt
        robot.add_ani_frame(k * dt, q=q)
        k+=1

    for j in range(2*steps):
        q[i]-= dt
        robot.add_ani_frame(k * dt, q=q)
        k+=1

    for j in range(steps):
        q[i]+= dt
        robot.add_ani_frame(k * dt, q=q)
        k+=1

    k+= 2*steps

print(k)

sim.save("D://","robot_demonstration")
