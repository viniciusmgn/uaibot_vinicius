from uaibot import *
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import numpy as np


#sim = Demo.control_demo_1()
#sim.save("D://","demo_1")


robot = Robot.create_kuka_kr5()

points, triangle = Utils.get_data_from_model("D:\\PycharmProjects\\UAIbot\\contents\\ABBCRB15000\\base_link.stl")
#points, triangle = Utils.get_data_from_model("D:\\PycharmProjects\\UAIbot\\contents\\KukaLBRIIWA\\base_link.dae")
#points, triangle = Utils.get_data_from_model("D:\\PycharmProjects\\UAIbot\\contents\\KukaKR5\\Axis1.obj")

print(len(triangle))
#triangle = np.array([[0,0,0],[0,1,0],[1,1,0]])
point = [1,0,0]

_, _, points_outside, points_inside = Utils.generate_connectivity_info(triangle,0.01)



points_outside = np.array(points_outside)
points_inside = np.array(points_inside)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(points[:,0], points[:,1], points[:,2])
ax.scatter(points_inside[:,0], points_inside[:,1], points_inside[:,2])
#ax.scatter(points_outside[:,0], points_outside[:,1], points_outside[:,2])
plt.show()

#for i in range(1000):
#    print(Utils._smooth_distance_surface(point,triangle,kd_tree,0.05))

sourceFile = open('D:\\mesh_triangle.txt', 'w')
print('mesh={', file = sourceFile)
for i in range(len(triangle)):
    print("["+str(triangle[i][0,0])+" "+str(triangle[i][0,1])+" "+str(triangle[i][0,2])+";", file = sourceFile)
    print(str(triangle[i][1,0])+" "+str(triangle[i][1,1])+" "+str(triangle[i][1,2])+";", file = sourceFile)
    if i == len(triangle)-1:
        print(str(triangle[i][2,0])+" "+str(triangle[i][2,1])+" "+str(triangle[i][2,2])+"]", file = sourceFile)
    else:
        print(str(triangle[i][2, 0]) + " " + str(triangle[i][2, 1]) + " " + str(triangle[i][2, 2]) + "],",
              file=sourceFile)

print('}', file = sourceFile)
sourceFile.close()

sourceFile = open('D:\\points_outside.txt', 'w')

print('points_outside=[', file = sourceFile)
data=""
for i in range(len(points_outside)):
    data+= str(points_outside[i][0])+" "
data+=";"
print(data, file = sourceFile)
data=""
for i in range(len(points_outside)):
    data+= str(points_outside[i][1])+" "
data+=";"
print(data, file = sourceFile)
data=""
for i in range(len(points_outside)):
    data+= str(points_outside[i][2])+" "
data+="];"
print(data, file = sourceFile)
sourceFile.close()






