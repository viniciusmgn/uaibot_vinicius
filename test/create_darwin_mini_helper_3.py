from uaibot import *
import numpy as np
from robot._create_darwin_mini_arm import _create_darwin_mini_arm

link_info = [[np.pi / 2, 0, 0],  # "theta" rotation in z
             [0.055, 0, 0],  # "d" translation in z
             [np.pi / 2, 0, 0],  # "alfa" rotation in x
             [0.035, 0.185, 0],  # "a" translation in x
             [0, 0, 0]]  # joint type

scale = 1
n = 3
color = "#3e3f42"
opacity = 1

col_model = [[], [], []]
# Create 3d objects

Q01 = Utils.rotz(link_info[0][0]) * Utils.trn([0, 0, link_info[1][0]]) * Utils.rotx(link_info[2][0]) * Utils.trn(
    [link_info[3][0], 0, 0])
Q02 = Utils.rotz(link_info[0][1]) * Utils.trn([0, 0, link_info[1][1]]) * Utils.rotx(link_info[2][1]) * Utils.trn(
    [link_info[3][1], 0, 0])
Q03 = Utils.rotz(link_info[0][2]) * Utils.trn([0, 0, link_info[1][2]]) * Utils.rotx(link_info[2][2]) * Utils.trn(
    [link_info[3][2], 0, 0])

base_3d_obj = []

link_3d_obj = []

link_3d_obj.append(
    [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/SPD_1.obj',
        0.004,
        Utils.inv_htm(Q01)*Utils.trn([-0.487,-0.315,-0.06])*Utils.rotx(-np.pi/2) * Utils.rotz(-0.3),
        MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                     opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/XL-320-2.obj',
            0.004,
            Utils.inv_htm(Q01)* Utils.trn([0.04, 0.73, 0.155]) * Utils.rotx(np.pi/2) * Utils.trn([0.69, -0.09, 0.655]) * Utils.rotx(0.08) * Utils.roty(-3.14 / 2) * Utils.trn(
                [-0.3, -0.179, 0.15]) * Utils.rotz(3.14 / 2) * Utils.rotx(3.14 / 2),
            MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                         opacity=opacity, side="DoubleSide"))
    ]
)

link_3d_obj.append(
    [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/frm_ext_02.obj',
        0.004,
        Utils.inv_htm(Q01 * Q02) * Utils.trn([0.317, -0.01-0.035, -0.435]) * Utils.rotx(0.275) * Utils.roty(-np.pi / 2),
        MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                     opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/SPO.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02)*Utils.trn([0.317,-0.35,-0.345])*Utils.roty(-np.pi/2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/XL-320-2.obj',
            0.004,
            Utils.inv_htm(Q01* Q02) * Utils.trn([0.04, -0.01+0.925, 0.155]) * Utils.rotx(np.pi / 2) * Utils.trn(
                [0.69, -0.09, 0.655]) * Utils.rotx(0.08) * Utils.roty(-3.14 / 2) * Utils.trn(
                [-0.3, -0.179, 0.15]) * Utils.rotz(3.14 / 2) * Utils.rotx(3.14 / 2),
            MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                         opacity=opacity, side="DoubleSide"))
    ]
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

# Create joint limits
joint_limits = (np.pi / 180) * np.matrix([[-180, 180], [-180, 180], [-180, 180]])


robot = Robot("aaa",links, base_3d_obj, np.identity(4), np.identity(4), q0, True, joint_limits)

sim = Simulation([robot],camera_type="orthographic")

sim.save("D:\\","darwin_mini_LEG ")