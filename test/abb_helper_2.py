from uaibot import *
import numpy as np

d = 0

a1 = 0.444-0.6

a2 = 0
a3 = -0.11

a4 = 0.47


a5 = 0
a6 = 0.08

a7=0.101

link_info = [[     0,      0,       0,       0,       0,     0],  # "theta" rotation in z
             [ 0.265,      0,       0,    0.47,       0, 0.101],  # "d" translation in z
             [3.14/2,      0, -3.14/2, -3.14/2, -3.14/2,     0],  # "alfa" rotation in x
             [0     ,  0.444,   -0.11,       0,    0.08,     0],  # "a" translation in x
             [0     ,      0,       0,       0,       0,     0]]  # joint type

opacity=0.3
scale=1
color="white"
n = 6

# Collision model
col_model = [[], [], [], [], [], []]



# Create 3d objects

base_3d_obj = [
    Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/base_link.stl',
            scale,
            np.identity(4),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                         opacity=1))]

link_3d_obj = []

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_1.stl',
             scale,
             Utils.rotx(-3.14/2),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                          opacity=1))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_2.stl',
             scale,
             Utils.trn([-0.444,0,0]) @ Utils.rotx(-3.14/2) @ Utils.roty(3.14/2),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                          opacity=1))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_3.stl',
             scale,
             Utils.trn([0.11,0, 0]) @ Utils.roty(-3.14/2),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                          opacity=1))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_4.stl',
             scale,
             Utils.trn([0,0.47,0]) @ Utils.roty(-3.14/2) @ Utils.rotz(-3.14/2),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                          opacity=1))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_5.stl',
             scale,
             Utils.trn([-0.08,0,0]) @ Utils.rotz(3.14/2) @ Utils.roty(-3.14/2) @ Utils.rotx(3.14/2),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="white",
                          opacity=1))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_6.stl',
             scale,
            Utils.trn([0,0,0]) @ Utils.roty(-3.14/2),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="silver",
                          opacity=1))]
)

# Inertial parameter of the robots


# Create links

links = []
for i in range(n):
    links.append(Link(i, link_info[0][i], link_info[1][i], link_info[2][i], link_info[3][i], link_info[4][i],
                      link_3d_obj[i]))

    for j in range(len(col_model[i])):
        links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

# Define initial configuration
q0 = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]

htm_base_0=np.identity(4)
htm = np.identity(4)

robot = Robot("test", links, base_3d_obj, htm, htm_base_0, q0)



sim = Simulation([robot], camera_type= "orthographic" )
#sim.add(Frame(name="a", htm=robot.fkm(axis="dh")[3]))
#sim.add(Frame(name="b", htm=robot.fkm(axis="dh")[5]))


htm_tg = Utils.trn([0.3,0.3,0.3]) @ Utils.roty(3.14/2)
q_tg = robot.ikm(htm_target = htm_tg)
q0 = robot.q


#sim.add(Frame(name="b", htm=htm_tg))

for i in range(1000):
    alpha = i/999
    robot.add_ani_frame(0.01*i, q= (1-alpha)*q0 + alpha*q_tg)


sim.save("D://","robot_create")