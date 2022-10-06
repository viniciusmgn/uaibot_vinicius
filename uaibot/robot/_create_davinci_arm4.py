from utils import Utils

from graphics.meshmaterial import MeshMaterial
from graphics.model3d import Model3D
from simobjects.ball import Ball
from simobjects.box import Box
from simobjects.cylinder import Cylinder
from simobjects.rigidobject import RigidObject
from simobjects.group import Group
from robot.links import Link

import numpy as np


def _create_davinci_arm4(color, opacity, name=''):

    if not Utils.is_a_color(color):
        raise Exception(
            "The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception(
            "The parameter 'opacity' should be a float between 0 and 1.")

    theta2 = np.deg2rad(164.8)
    theta3 = np.deg2rad(-131.8)
    theta4 = np.deg2rad(-33)
    theta5 = np.deg2rad(-91)
    theta6 = np.deg2rad(94.5)
    theta7 = np.deg2rad(-169.82)
    theta8 = np.deg2rad(37.1)

    d3 = 96e-3 * 1.25
    d4 = 96e-3 * 3.68
    d5 = 0.919
    d8 = -3.4e-2

    alpha4 = -np.deg2rad(90 + 63.1)
    alpha5 = -np.pi/2
    alpha8 = np.deg2rad(90.7)

    a2 = 0.3
    a3 = 0.3
    a5 = -8e-4
    a6 = 0.27
    a7 = 0.473
    a8 = 0.115

    # Passive joints 1, 2, 3, 4, 7, 8
    # Active joints 5, 6, 9
    link_info = np.array([
        # "theta" rotation in z
        [0,  0,   0,       0,      0,      0,      0,       0, 0],
        # 0,  "d" translation in z
        [0,  0,  d3,      d4,     d5,      0,      0,      d8, 0],
        # 0,  "alfa" rotation in x
        [0,  0,   0,  alpha4, alpha5,      0,      0,  alpha8, 0],
        # 0,  "a" translation in x
        [0, a2,  a3,       0,     a5,     a6,     a7,      a8, 0],
        # 0,  joint type
        [1,  0,   0,       0,      0,      0,      0,       0, 1]
    ])

    scale = 1
    n = link_info.shape[1]
    base_3d_obj = []
    link_3d_obj = []
    mesh = MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[
                        0.5, 0.5], color=color, opacity=opacity, side="DoubleSide")
    b1 = 0.099
    b2 = 0.0922
    b3 = 1.54

    Q00 = Utils.trn([b1, b3, -b2]) * Utils.rotx(-np.pi/2) * \
        Utils.rotz(np.pi)  # change reference frame
    Q01 = Q00 * (Utils.rotz(link_info[0, 0]) * Utils.trn([0, 0, link_info[1, 0]]) * Utils.rotx(link_info[2, 0]) * Utils.trn(
        [link_info[3, 0], 0, 0]))
    Q02 = Q01 * (Utils.rotz(link_info[0, 0] + theta2) * Utils.trn([0, 0, link_info[1, 1]]) * Utils.rotx(link_info[2, 1]) * Utils.trn(
        [link_info[3, 1], 0, 0]))
    Q03 = Q02 * (Utils.rotz(link_info[0, 1] + theta3) * Utils.trn([0, 0, link_info[1, 2]]) * Utils.rotx(link_info[2, 2]) * Utils.trn(
        [link_info[3, 2], 0, 0]))
    Q04 = Q03 * (Utils.rotz(link_info[0, 2] + theta4) * Utils.trn([0, 0, link_info[1, 3]]) * Utils.rotx(link_info[2, 3]) * Utils.trn(
        [link_info[3, 3], 0, 0]))
    Q05 = Q04 * (Utils.rotz(link_info[0, 3] + theta5) * Utils.trn([0, 0, link_info[1, 4]]) * Utils.rotx(link_info[2, 4]) * Utils.trn(
        [link_info[3, 4], 0, 0]))
    Q06 = Q05 * (Utils.rotz(link_info[0, 4] + theta6) * Utils.trn([0, 0, link_info[1, 5]]) * Utils.rotx(link_info[2, 5]) * Utils.trn(
        [link_info[3, 5], 0, 0]))
    Q07 = Q06 * (Utils.rotz(link_info[0, 5] + theta7) * Utils.trn([0, 0, link_info[1, 6]]) * Utils.rotx(link_info[2, 6]) * Utils.trn(
        [link_info[3, 6], 0, 0]))
    Q08 = Q07 * (Utils.rotz(link_info[0, 6] + theta8) * Utils.trn([0, 0, link_info[1, 7]]) * Utils.rotx(link_info[2, 7]) * Utils.trn(
        [link_info[3, 7], 0, 0]))
    Q09 = Q08 * (Utils.rotz(link_info[0, 7]) * Utils.trn([0, 0, link_info[1, 8]]) * Utils.rotx(link_info[2, 8]) * Utils.trn(
        [link_info[3, 8], 0, 0]))

    link1_mth = Utils.inv_htm(Q01)
    link_3d_obj.append([
        # torre movel
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/57.obj',
                scale=scale, htm=link1_mth, mesh_material=mesh),
        # cilindro conector
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/60.obj',
                scale=scale, htm=link1_mth, mesh_material=mesh),
    ])

    link2_mth = Utils.inv_htm(Q02)
    link_3d_obj.append([
        # ligacao de cilindros curta
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/15.obj',
                scale=scale, htm=link2_mth, mesh_material=mesh),
    ])

    link3_mth = Utils.inv_htm(Q03)
    link_3d_obj.append([
        # ligacao de cilindros curta
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/66.obj',
                scale=scale, htm=link3_mth, mesh_material=mesh),
    ])

    link4_mth = Utils.inv_htm(Q04)
    link_3d_obj.append([
        # cotovelo
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/16.obj',
                scale=scale, htm=link4_mth, mesh_material=mesh),
    ])

    link5_mth = Utils.inv_htm(Q05)
    link_3d_obj.append([
        # pá dobrada
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/37.obj',
                scale=scale, htm=link5_mth, mesh_material=mesh),
    ])

    link6_mth = Utils.inv_htm(Q06)
    link_3d_obj.append([
        # conecta pá com bumerangue
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/33.obj',
                scale=scale, htm=link6_mth, mesh_material=mesh),
    ])

    link7_mth = Utils.inv_htm(Q07)
    link_3d_obj.append([
        # bumerangue
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/41.obj',
                scale=scale, htm=link7_mth, mesh_material=mesh),
    ])

    link8_mth = Utils.inv_htm(Q08)
    link_3d_obj.append([
        # envoltoria agulha
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/28.obj',
                scale=scale, htm=link8_mth, mesh_material=mesh),
    ])

    link9_mth = Utils.inv_htm(Q09)
    link_3d_obj.append([
        # guia
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/29.obj',
                    scale=scale, htm=link9_mth, mesh_material=mesh),
        # agulha
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/31.obj',
                scale=scale, htm=link9_mth, mesh_material=mesh),
        # base agulha
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/30.obj',
                scale=scale, htm=link9_mth, mesh_material=mesh),
        # cabo longo
        #Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/32.obj',
        #        scale=scale, htm=link9_mth, mesh_material=mesh),
    ])

    col_model = [[], [], [], [], [], [], [], [], []]

    col_model[0].append(Box(htm=Utils.trn([0, -0.03, -0.33]),
                            name=name + "_C0_0", width=0.14, height=0.4, depth=0.2, color="red", opacity=0.3))
    col_model[0].append(Cylinder(htm=Utils.trn([0, 0, -0.154]) @ Utils.rotz(3.14 / 2) @ Utils.rotx(3.14),
                                 name=name + "_C0_1", radius=0.075, height=0.19, color="red", opacity=0.3))

    col_model[1].append(Box(htm=Utils.trn([-0.15, 0, 0]) @ Utils.roty(np.pi/2),
                            name=name + "_C1_0", width=0.11, height=0.3, depth=0.12, color="green", opacity=0.3))
    col_model[1].append(Cylinder(htm=Utils.trn([0, 0, -0.001]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C1_1", radius=0.075, height=0.123, color="green", opacity=0.3))
    col_model[1].append(Cylinder(htm=Utils.trn([-0.3, 0, 0.002]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C1_2", radius=0.075, height=0.12, color="green", opacity=0.3))

    col_model[2].append(Box(htm=Utils.trn([-0.15, 0, 0.0]) @ Utils.roty(np.pi/2),
                            name=name + "_C2_0", width=0.11, height=0.3, depth=0.12, color="blue", opacity=0.3))
    col_model[2].append(Cylinder(htm=Utils.trn([0, 0, -0.025]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C2_1", radius=0.075, height=0.08, color="blue", opacity=0.3))
    col_model[2].append(Cylinder(htm=Utils.trn([-0.3, 0, 0.002]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C2_2", radius=0.075, height=0.12, color="blue", opacity=0.3))
    col_model[2].append(Cylinder(htm=Utils.trn([0, 0, 0.033]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C2_3", radius=0.062, height=0.05, color="blue", opacity=0.3))

    col_model[3].append(Cylinder(htm=Utils.rotx(-alpha4) @ Utils.trn([0, 0, 0.101-d4]),
                                 name=name + "_C3_0", radius=0.075, height=0.08, color="magenta", opacity=0.3))
    col_model[3].append(Box(htm=Utils.trn([0, 0.02, 0.42]) @ Utils.roty(np.pi/2),
                            name=name + "_C3_1", width=0.23, height=0.14, depth=0.12, color="magenta", opacity=0.3))
    col_model[3].append(Box(htm=Utils.rotx(-alpha4) @ Utils.trn([0, 0.072, -0.259]),
                            name=name + "_C3_2", width=0.14, height=0.07, depth=0.2, color="magenta", opacity=0.3))

    col_model[4].append(Box(htm=Utils.trn([0, 0.355, -0.05]) @ Utils.roty(np.pi/2),
                            name=name + "_C4_0", width=0.2, height=0.08, depth=0.05, color="orange", opacity=0.3))
    col_model[4].append(Box(htm=Utils.trn([0, 0.15, -0.1288]) @ Utils.roty(np.pi/2),
                            name=name + "_C4_1", width=0.05, height=0.09, depth=0.4, color="orange", opacity=0.3))

    col_model[5].append(Box(htm=Utils.trn([-0.2, 0.025, -0.03]),
                            name=name + "_C5_0", width=0.25, height=0.14, depth=0.15, color="cyan", opacity=0.3))
    col_model[5].append(Box(htm=Utils.trn([-0.018, 0.02, -0.09]),
                            name=name + "_C5_1", width=0.14, height=0.02, depth=0.15, color="cyan", opacity=0.3))
    col_model[5].append(Box(htm=Utils.trn([-0.019, 0.02, 0.015]),
                            name=name + "_C5_2", width=0.14, height=0.02, depth=0.15, color="cyan", opacity=0.3))

    col_model[6].append(Box(htm=Utils.trn([-0.42, 0.08, -0.0372]) @ Utils.rotz(np.pi/3),
                            name=name + "_C6_0", width=0.23, height=0.082, depth=0.13, color="#88264a", opacity=0.3))
    col_model[6].append(Box(htm=Utils.trn([-0.22, 0.08, -0.034]) @ Utils.rotz(np.pi*7/18),
                            name=name + "_C6_1", width=0.11, height=0.09, depth=0.39, color="#88264a", opacity=0.3))
    col_model[6].append(Box(htm=Utils.trn([-0.023, 0.008, 0.002]) @ Utils.rotz(np.pi*7/18),
                            name=name + "_C6_2", width=0.08, height=0.01, depth=0.03, color="#88264a", opacity=0.3))
    col_model[6].append(Box(htm=Utils.trn([-0.023, 0.008, -0.071]) @ Utils.rotz(np.pi*7/18),
                            name=name + "_C6_3", width=0.08, height=0.01, depth=0.03, color="#88264a", opacity=0.3))
    col_model[6].append(Cylinder(htm=Utils.trn([0, 0.001, -0.071]),
                                 name=name + "_C6_4", radius=0.035, height=0.01, color="#88264a", opacity=0.3))
    col_model[6].append(Cylinder(htm=Utils.trn([0, 0, 0.002]),
                                 name=name + "_C6_5", radius=0.035, height=0.01, color="#88264a", opacity=0.3))

    col_model[7].append(Box(htm=Utils.trn([-0.065, 0, -0.18]),
                            name=name + "_C7_0", width=0.028, height=0.63, depth=0.12, color="Brown", opacity=0.3))
    col_model[7].append(Cylinder(htm=Utils.trn([-0.115, 0.001, -0.0013]) @ Utils.rotx(np.pi/2),
                                 name=name + "_C7_1", radius=0.035, height=0.056, color="Brown", opacity=0.3))
    col_model[7].append(Box(htm=Utils.trn([-0.09, -0.005*0, 0.0]),
                            name=name + "_C7_2", width=0.04, height=0.07, depth=0.056, color="Brown", opacity=0.3))
    col_model[7].append(Box(htm=Utils.trn([-0.03, -0.001, 0.073]),
                            name=name + "_C7_3", width=0.04, height=0.05, depth=0.04, color="Brown", opacity=0.3))
    col_model[7].append(Cylinder(htm=Utils.trn([0, -0.001, 0.074]),
                                 name=name + "_C7_4", radius=0.02, height=0.056, color="Brown", opacity=0.3))

    col_model[8].append(Box(htm=Utils.trn([-0.005, 0.0, -0.22]),
                            name=name + "_C8_0", width=0.07, height=0.4, depth=0.12, color="MidnightBlue", opacity=0.3))
    col_model[8].append(Cylinder(htm=Utils.trn([0, 0, 0.135]),
                                 name=name + "_C8_1", radius=0.01, height=0.4, color="MidnightBlue", opacity=0.3))

    links = []
    for i in range(n):
        links.append(Link(i, theta=link_info[0, i], d=link_info[1, i], alpha=link_info[2, i], a=link_info[3, i], joint_type=link_info[4, i],
                          list_model_3d=link_3d_obj[i]))
        for j in range(len(col_model[i])):
           links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration
    #     1  2  3  4  5  6  7  8  9
    q0 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    htm_n_eef = Utils.trn([0, 0, 0.1899])  # np.identity(4)
    htm_base_0 = Utils.trn([-b1, -b2, b3])#np.identity(4)

    # Create joint limits
    # joint_limits = (np.pi / 180) * np.matrix([[-180, 180], [-180, 180], [-180, 180]])

    return links, base_3d_obj, htm_base_0, htm_n_eef, q0
