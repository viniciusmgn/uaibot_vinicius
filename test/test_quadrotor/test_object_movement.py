import scipy.linalg

import uaibot as ub
import numpy as np
from cvxopt import matrix, solvers
from scipy.linalg import expm, null_space

solvers.options['show_progress'] = False
np.set_printoptions(precision=4, suppress=True, linewidth=150)



texture_gold = ub.Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/gold_metal.png',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

texture_wall = ub.Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

material_obstacle = ub.MeshMaterial(texture_map=texture_wall, roughness=1, metalness=1)
material_robot  = ub.MeshMaterial(texture_map=texture_gold, roughness=1, metalness=1)

#material_glass = ub.MeshMaterial(metalness=0.9, roughness=0.05, env_map_intensity=0.9, clearcoat=1, opacity=1, \
#                              reflectivity=0.2, refraction_ratio=0.985, ior=1.52, specular_intensity=0.1,
#                              specular_color="white", transmission=1, side="BackSide")

#Create the "robot"
robot = ub.SmoothBox(htm = ub.Utils.trn([0,0,0.3]), width=0.2,depth=0.3,height=0.4,color="red", mesh_material=material_robot)
radius_robot = np.sqrt(3)*0.4/2
p_des = np.matrix([ [1.25, 1.25, 1.4], [-1.25, -1.25, 0.25], [-1.25, 1.25, 0.25]]).T

target_box=[]

for i in range(np.shape(p_des)[1]):
    target_box.append(ub.Box(htm = ub.Utils.trn(p_des[0:3,i]), width=0.3, depth=0.3, height=0.3, color="blue", opacity=0.5))

#Create a "random" set of obstacles
nobs=40
nobsfree=40
obstacle = []
radius_obstacle = []

while len(obstacle)<nobs:
    w = np.random.uniform(0.1, 0.7)
    d = np.random.uniform(0.1, 0.7)
    h = np.random.uniform(0.1, 0.7)
    dx = np.random.uniform(-1.5,1.5)
    dy = np.random.uniform(-1.5,1.5)
    dz = np.random.uniform(0.1,1.5)
    rx = np.random.uniform(0, 6.5)
    ry = np.random.uniform(0, 6.5)
    rz = np.random.uniform(0, 6.5)
    htm_b = ub.Utils.trn([dx,dy,dz]) * ub.Utils.rotx(rx) * ub.Utils.roty(ry) * ub.Utils.rotz(rz)
    box = ub.Box(htm=htm_b,width=w,depth=d,height=h,color="blue", mesh_material=material_obstacle)

    _, _, dist = ub.Utils.compute_dist(box, robot)
    collided=dist<0.2

    if len(obstacle) < nobsfree:
        i=0
        while not collided and i < len(obstacle):
            _, _, dist = ub.Utils.compute_dist(box, obstacle[i])
            collided = dist<0.2
            i+=1

    i = 0
    while not collided and i < len(target_box):
        _, _, dist = ub.Utils.compute_dist(box, target_box[i])
        collided = dist<0.2
        i+=1

    if not collided:
        obstacle.append(box)
        radius_obstacle.append(np.sqrt(3)*max(w/2,d/2,h/2))
        print(len(obstacle))

print("End!")


sim = ub.Simulation.create_sim_factory(obstacle)
sim.add(robot)
sim.add(target_box)



obstacle.append(ub.Box(width=3,depth=3,height=0.01,color="yellow"))
radius_obstacle.append(1000)

obstacle.append(ub.Box(htm=ub.Utils.trn([-1.5,0,1.5]), width=0.01,depth=3,height=3,color="yellow"))
radius_obstacle.append(1000)
obstacle.append(ub.Box(htm=ub.Utils.trn([1.5,0,1.5]), width=0.01,depth=3,height=3,color="yellow"))
radius_obstacle.append(1000)

obstacle.append(ub.Box(htm=ub.Utils.trn([0,-1.5,1.5]), width=3,depth=0.01,height=3,color="yellow"))
radius_obstacle.append(1000)
obstacle.append(ub.Box(htm=ub.Utils.trn([0,1.5,1.5]), width=3,depth=0.01,height=3,color="yellow"))
radius_obstacle.append(1000)



dt=0.04

eta=0.1
kp=0.4

tg = 0

k=0

alpha=0.5
v = np.matrix(np.zeros((3,1)))
w = np.matrix(np.zeros((3,1)))


behaviour=0
choose=False
error = False
list_task = []
list_p = np.matrix(np.zeros((3,0)))
list_Q = np.matrix(np.zeros((9,0)))

N=30

while not error and tg<len(target_box) and k < 2500:


    p=robot.htm[0:3,-1]
    Q=robot.htm[0:3,0:3]

    #Change behaviour


    if behaviour == 0:
        if len(list_task) > N:
            dev_task = np.std(np.log(list_task[-1 - N:-1]))
            dev_p = np.max(np.std(list_p[:,-1-N:-1],axis=1))
            dev_Q = np.max(np.std(list_Q[:,-1-N:-1],axis=1))

            if dev_task < 0.005 and dev_p < 0.005 and dev_Q < 0.005 and  list_task[-1]>0.01:
                print("Detected that the robot is stuck at time "+str(k*dt))
                behaviour = 1

        if np.linalg.norm(p-p_des[0:3,tg])<0.01:
            behaviour = 0
            list_task = []
            list_p = np.matrix(np.zeros((3,0)))
            list_Q = np.matrix(np.zeros((9,0)))
            tg += 1
            if tg<len(target_box):
                print("Changed to task " + str(tg) +" at time "+str(k*dt))
            else:
                print("Over!")

    if behaviour == 1 and not choose:
        u_d = np.matrix(np.random.uniform(-1,1,(6,1)))
        choose = True
        print("Changed to tangent behaviour")

    #Compute distances
    dist_min = 1000
    A = np.matrix(np.zeros((0, 6)))
    b = np.matrix(np.zeros((0, 1)))
    list_dist=[]

    for i in range(len(obstacle)):
        if np.linalg.norm(p - obstacle[i].htm[0:3, -1]) - (radius_robot + radius_obstacle[i]) < 0.4:
            p_robot, p_obs, dist = ub.Utils.compute_dist(robot, obstacle[i])

            a = np.block([(p_robot - p_obs).T, (p_robot - p_obs).T * ub.Utils.S(p - p_robot)])
            A = np.block([[A], [a]])
            b = np.block([[b], [-eta * (dist - 0.03)]])

            list_dist.append(dist)
            dist_min = min(dist_min, dist)


    #Behaviour: go to the target
    if behaviour==0 and tg<len(target_box):

        H = 2 * np.diag([1, 1, 1, 0.1, 0.1, 0.1])
        f = np.block([[2 * kp * (p - p_des[0:3, tg])], [0], [0], [0]])

        list_task.append(np.linalg.norm(p - p_des[0:3, tg]))
        list_p = np.block([list_p, p])
        list_Q = np.block([list_Q, Q[0:3,0:3].reshape((9,1))])

        try:
            u = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b))['x']
            u = np.reshape(u, (6, 1))
        except:
            error = True



    #Behaviour: go tangent
    if behaviour==1:
        ind_sort = np.argsort(list_dist)
        An = np.matrix(np.zeros((0, 6)))

        for i in range(min(len(list_dist),3)):
            An = np.block([[An], [ A[ ind_sort[i], :]] ])

        u = u_d - An.T * np.linalg.inv( An * An.T ) * An * u_d



    #Integrate the linear and angular velocity
    if not error:
        v = (1 - alpha) * v + alpha * np.matrix(u[0:3, 0]).reshape((3, 1))
        w = (1 - alpha) * w + alpha * np.matrix(u[3:6, 0]).reshape((3, 1))

        p = p + v*dt
        Q = scipy.linalg.expm(ub.Utils.S( dt * w )) * Q
        htm_new = np.block( [[Q, p], [ np.matrix([0, 0, 0, 1]).reshape((1,4)) ] ])

        robot.add_ani_frame(k*dt,htm=htm_new)


    #Print time
    k+=1
    print(k)

    if dist_min < 0.01:
        print("error!")



sim.save("D://","test_obj_movement")