from uaibot import *
import numpy as np

link_info = [[       0,     0,      0],  # "theta" rotation in z
             [       0,     0,      0],  # "d" translation in z
             [       0,     0,      0],  # "alfa" rotation in x
             [       0,     0,      0],  # "a" translation in x
             [       0,     0,      0]]  # joint type

scale = 1
n = 3

opacity = 1

col_model = [[],[],[]]
# Create 3d objects

base_3d_obj = [
    Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/Darwin-mini_frm_ext_01.obj',
        0.004,
        Utils.trn([0,0,0.5]),
        MeshMaterial(metalness=1, clearcoat=0, roughness=0, normal_scale=[1, 1], color="red",
                     opacity=opacity))]

link_3d_obj = []

link_3d_obj.append(
    []
)

link_3d_obj.append(
    []
)

link_3d_obj.append(
    []
)


# Create links

links = []
for i in range(n):
    links.append(Link(i, link_info[0][i], link_info[1][i], link_info[2][i], link_info[3][i], link_info[4][i],
                      link_3d_obj[i]))

    for j in range(len(col_model[i])):
        links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

# Define initial configuration
q0 = [0.000, 0.000, 0.000]

#Create joint limits
joint_limits = (np.pi/180)*np.matrix([[-180,180],[-180,180],[-180,180]])



robot = Robot("darwin_mini", links, base_3d_obj, np.identity(4), np.identity(4), q0, True, joint_limits)

sim = Simulation.create_sim_factory([robot])

sim.save("D:\\","darwin_mini")