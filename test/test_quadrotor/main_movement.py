import uaibot as ub
import numpy as np
import scipy.linalg
from control_target import *
from generate_env import *
from ispathfree import *
from scipy.linalg import expm, null_space

texture_gold = ub.Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/gold_metal.png',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

material_robot  = ub.MeshMaterial(texture_map=texture_gold, roughness=1, metalness=1)

#Create robot
robot = ub.SmoothBox(htm = ub.Utils.trn([0,0,0.3]), width=0.4,depth=0.4,height=0.15,color="red", mesh_material=material_robot)
radius_robot = np.sqrt(3)*0.4/2
p_des = np.matrix([ [1.225, 1.225, 1.4], [-1.225, -1.225, 0.25], [-1.225, 1.225, 0.25]]).T

target_box=[]

for i in range(np.shape(p_des)[1]):
    target_box.append(ub.Box(htm = ub.Utils.trn(p_des[0:3,i]), width=0.3, depth=0.3, height=0.3, color="blue", opacity=0.5))

#Create environment
nobs = 40
obstacle, radius_obstacle = generate_env(robot, target_box, nobs)

sim = ub.Simulation.create_sim_factory(obstacle)
sim.add(robot)
sim.add(target_box)


#Main movement
dt = 0.03
tmax = 200
t=0
tg=0
alpha=0.5
v = np.matrix(np.zeros((3,1)))
w = np.matrix(np.zeros((3,1)))
k=0

error = False
done = False
chasing_target = True

hist_task = np.matrix(np.zeros((1,0)))
hist_p = np.matrix(np.zeros((3,0)))
hist_Q = np.matrix(np.zeros((9,0)))
N = round(4/dt)

p_target =  p_des[0:3, 0]

while not error and not done:

    #Make some checks
    p = robot.htm[0:3, -1]
    Q = robot.htm[0:3, 0:3]

    t = round(k*dt,2)

    #Check if the robot is stuck
    if np.shape(hist_task)[1] > N:
        dev_task = round(np.std(np.log(hist_task[0,-1 - N:-1])),5)
        dev_p = round(np.max(np.std(hist_p[:, -1 - N:-1], axis=1)),5)
        dev_Q = round(np.max(np.std(hist_Q[:, -1 - N:-1], axis=1)),5)

        if k % N==0:
            print("Movement statistics at t: " + str(t) + ": (dev_task: " + str(dev_task) + ", dev_p: " + str(dev_p) + ", dev_Q: " + str(
                dev_Q) + ")")

        if dev_task < 0.005 and dev_p < 0.005 and dev_Q < 0.03 and hist_task[0,-1] > 0.01:

            hist_task = np.matrix(np.zeros((1, 0)))
            hist_p = np.matrix(np.zeros((3, 0)))
            hist_Q = np.matrix(np.zeros((9, 0)))

            if chasing_target:
                p_target = p_des[0:3, tg] + 0.5 * np.random.normal(0,1,(3,1))
                while not ispathfree(obstacle, p, p_target):
                    p_target = p_des[0:3, tg] + 0.5 * np.random.normal(0, 1, (3, 1))

                chasing_target = False
                print("Detected that the robot is stuck at time " + str(t) + "! Changing to other temporary target.")

            else:
                p_target = p_des[0:3, tg]
                chasing_target = True
                print("Returning to chasing target " + str(tg) + "!")




    #Check if task changed
    c_task_error = round(np.linalg.norm(p - p_target),5)
    o_error = round((180/np.pi) * np.arccos(Q[-1,-1]))

    if c_task_error < 0.01:

        hist_task = np.matrix(np.zeros((1, 0)))
        hist_p = np.matrix(np.zeros((3, 0)))
        hist_Q = np.matrix(np.zeros((9, 0)))

        if chasing_target:
            tg += 1
            if tg < len(target_box):
                p_target = p_des[0:3, tg]
                c_task_error = round(np.linalg.norm(p - p_target), 5)
                print("Changed to task " + str(tg) + " at time " + str(t))

            else:
                done = True
                print("Over!")

        else:
            p_target = p_des[0:3, tg]
            c_task_error = round(np.linalg.norm(p - p_target), 5)
            chasing_target = True
            print("Returning to chasing target " + str(tg) + "!")


    done = done or t > tmax

    #Compute control action and act
    if not done:
        v_u, w_u, error, dist_min = control_target(robot, radius_robot, obstacle, radius_obstacle, p_target)

    if error:
        print("Collision error!!!!!!")

    if not done and not error:
        v = (1 - alpha) * v + alpha * v_u
        w = (1 - alpha) * w + alpha * w_u

        p = p + v*dt
        Q = scipy.linalg.expm(ub.Utils.S( dt * w )) * Q
        htm_new = np.block( [[Q, p], [ np.matrix([0, 0, 0, 1]).reshape((1,4)) ] ])

        robot.add_ani_frame(k*dt,htm=htm_new)

        k+=1

        #Store info
        hist_task = np.block([hist_task, c_task_error])
        hist_p = np.block([hist_p, p])
        hist_Q = np.block([hist_Q, Q[0:3,0:3].reshape((9,1))])

        if k%20==0:
            if chasing_target:
                print("Time: "+str(t)+", task_number: "+str(tg)+", task_error: "+str(c_task_error) +", o_error: "+str(o_error))
            else:
                print("Time: " + str(t) + ", chasing temporary target, task_error: " + str(c_task_error) +", o_error: "+str(o_error))

sim.save("D://","test_obj_movement")