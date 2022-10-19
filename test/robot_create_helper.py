from uaibot import *
import numpy as np

link_info = [[0, 0, 0, 0, 0, 0],  # "theta" rotation in z
             [0, 0, 0, 0, 0, 0],  # "d" translation in z
             [0, 0, 0, 0, 0, 0],  # "alfa" rotation in x
             [0, 0, 0, 0, 0, 0],  # "a" translation in x
             [0, 0, 0, 0, 0, 0]]  # joint type

opacity=1
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
                         opacity=opacity))]

link_3d_obj = []

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_1.stl',
             scale,
             np.identity(4),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                          opacity=0))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_2.stl',
             scale,
             np.identity(4),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                          opacity=0))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_3.stl',
             scale,
             np.identity(4),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                          opacity=0))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_4.stl',
             scale,
             np.identity(4),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                          opacity=0))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_5.stl',
             scale,
             np.identity(4),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                          opacity=0))]
)

link_3d_obj.append(
    [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_6.stl',
             scale,
             np.identity(4),
             MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                          opacity=0))]
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
q0 = [0, 0, 0.000, 0.000, 0, 0.000]

htm_base_0=np.identity(4)
htm = np.identity(4)

robot = Robot("test", links, base_3d_obj, htm, htm_base_0, q0)

sim = Simulation.create_sim_factory([robot])
sim.save("D://","robot_create")