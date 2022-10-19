from uaibot import *
import numpy as np

link_info = [[       np.pi/2,     -np.pi/90,  np.pi/90],  # "theta" rotation in z
             [       0.08    ,    0,      0],  # "d" translation in z
             [       np.pi/2,     0,      0],  # "alfa" rotation in x
             [       0.05    ,     0.18,      0.23],  # "a" translation in x
             [       0      ,     0,      0]]  # joint type

scale = 1
n = 3

opacity = 1

col_model = [[],[],[]]
# Create 3d objects

Q01 = Utils.rotz(link_info[0][0])*Utils.trn([0,0,link_info[1][0]])*Utils.rotx(link_info[2][0])*Utils.trn([link_info[3][0],0,0])
Q02 = Utils.rotz(link_info[0][1])*Utils.trn([0,0,link_info[1][1]])*Utils.rotx(link_info[2][1])*Utils.trn([link_info[3][1],0,0])
Q03 = Utils.rotz(link_info[0][2])*Utils.trn([0,0,link_info[1][2]])*Utils.rotx(link_info[2][2])*Utils.trn([link_info[3][2],0,0])



base_3d_obj = []

link_3d_obj = []

link_3d_obj.append(
     [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/darwin_ext_01.obj',
        0.004,
        Utils.inv_htm(Q01) * Utils.trn([-0.315,0.435,-0.325]) * Utils.rotz(-3.14) * Utils.roty(-3.14/2),
        MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                      opacity=opacity, side="DoubleSide"))]
)

link_3d_obj.append(
    [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/XL-320.obj',
        0.004,
        Utils.inv_htm(Q01*Q02) * Utils.trn([0.004, 0.01, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
            [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
        MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                     opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPO.obj',
            0.004,
            Utils.inv_htm(Q01*Q02) * Utils.trn([0.004, 0.01, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/frm_ext_02.obj',
            0.004,
            Utils.inv_htm(Q01*Q02) * Utils.trn([0.004, 0.01, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide"))
    ]
)

UU = Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPO.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03)*Utils.roty(-np.pi/2)*Utils.rotx(-np.pi/2)*Utils.trn([-0.325,-0.385,-0.0115]),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide"))

link_3d_obj.append(
    [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/XL-320.obj',
        0.004,
        Utils.inv_htm(Q01*Q02*Q03) * Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
            [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
        MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                     opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPO.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPU.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.004, 0.013, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPD_2.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.004, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPO.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.roty(-np.pi / 2) * Utils.rotx(-np.pi / 2) * Utils.trn(
                [-0.325, -0.385, -0.0115]),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPD_1.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.004, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPL-2b2.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.004, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPL-2b2.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([-0.044, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPL-2b2.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) *  Utils.trn([0.46, 0.502, 0.075]) * Utils.rotx(3.14/2) * Utils.rotz(3.14/2) * Utils.trn([-0.044, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPL-2b2.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0, 0, 0.083]) * Utils.roty(3.14) * Utils.trn([0.46, 0.502, 0.075]) * Utils.rotx(3.14 / 2) * Utils.rotz(
                3.14 / 2) * Utils.trn([-0.044, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPD_1.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.004, 0.035, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPD_3.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.004, 0.035, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPL-2b2.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) *  Utils.trn([0.007, 0.905, 0.043]) * Utils.rotx(3.14) * Utils.roty(-3.14 / 2) * Utils.trn([0, 0, 0.083]) * Utils.roty(3.14) * Utils.trn(
                [0.46, 0.502, 0.075]) * Utils.rotx(3.14 / 2) * Utils.rotz(
                3.14 / 2) * Utils.trn([-0.044, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide")),
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPL-2b2.obj',
            0.004,
            Utils.inv_htm(Q01 * Q02 * Q03) * Utils.trn([0.055, 0.905, 0.043]) * Utils.rotx(3.14) * Utils.roty(
                -3.14 / 2) * Utils.trn([0, 0, 0.083]) * Utils.roty(3.14) * Utils.trn(
                [0.46, 0.502, 0.075]) * Utils.rotx(3.14 / 2) * Utils.rotz(
                3.14 / 2) * Utils.trn([-0.044, 0.0136, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
                [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2),
            MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="purple",
                         opacity=opacity, side="DoubleSide"))
    ]
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



sim = Simulation([robot],background_color="black",camera_type="orthographic")

for i in range(1000):
    robot.add_ani_frame(i*0.01,q=[0,-link_info[0][1],-link_info[0][2]])

sim.save("D:\\","darwin_mini")




























