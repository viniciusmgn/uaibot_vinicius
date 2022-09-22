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


def _create_davinci_arm1(color, opacity, name=''):

    if not Utils.is_a_color(color):
        raise Exception(
            "The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception(
            "The parameter 'opacity' should be a float between 0 and 1.")

    theta2 = np.deg2rad(1.875)
    theta3 = np.deg2rad(3.25 - 1.875)
    theta4 = np.deg2rad(75.25)
    theta5 = np.deg2rad(28.75 + 90)
    theta6 = np.deg2rad(-45)
    theta7 = np.deg2rad(94 + 180)
    theta8 = np.deg2rad(-170.5)
    theta9 = np.deg2rad(61.5)

    d3 = -96e-3
    d5 = -96e-3 * 1.9
    d6 = (431.8 * 1.417)/1000
    d9 = -3.75e-2
    d10 = 3e-2

    alpha5 = np.pi*(1/2 + 1/9)
    alpha6 = np.pi/2
    alpha9 = np.pi/2
    alpha10 = np.pi/2*0

    a2 = 0.3
    a3 = 0.415
    a4 = 0.415
    a6 = 3.1e-3
    a7 = 0.27
    a8 = 0.474
    a9 = 0.098

    # Passive joints 1, 2, 3, 4, 5, 8, 9
    # Active joints 6, 7, 10
    link_info = np.array([
        # "theta" rotation in z
        [0, 0,   0,  0,      0,      0,  0,  0,      0,   0],
        # "d" translation in z
        [0, 0,  d3,  0,     d5,     d6,  0,  0,     d9, d10],
        # "alfa" rotation in x
        [0, 0,   0,  0, alpha5, alpha6,  0,  0, alpha9,   0],
        # "a" translation in x
        [0, a2, a3, a4,      0,     a6, a7, a8,     a9,   0],
        # joint type
        [1, 0,   0,  0,      0,      0,  0,  0,      0,   1]
    ])

    scale = 1
    n = link_info.shape[1]
    base_3d_obj = []
    link_3d_obj = []
    mesh = MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[
                        0.5, 0.5], color=color, opacity=opacity, side="DoubleSide")
    b1 = 0.1
    b2 = 0.45
    b3 = 1.1

    Q00 = Utils.trn([b1, b3, -b2]) * Utils.rotx(-np.pi/2) * \
        Utils.rotz(np.pi)  # change reference frame
    Q01 = Q00 * Utils.rotz(link_info[0, 0]) * Utils.trn([0, 0, link_info[1, 0]]) * Utils.rotx(link_info[2, 0]) * Utils.trn(
        [link_info[3, 0], 0, 0])
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
    Q09 = Q08 * (Utils.rotz(link_info[0, 7] + theta9) * Utils.trn([0, 0, link_info[1, 8]]) * Utils.rotx(link_info[2, 8]) * Utils.trn(
        [link_info[3, 8], 0, 0]))
    Q010 = Q09 * (Utils.rotz(link_info[0, 8]) * Utils.trn([0, 0, link_info[1, 9]]) * Utils.rotx(link_info[2, 9]) * Utils.trn(
        [link_info[3, 9], 0, 0]))

    link1_mth = Utils.inv_htm(Q01)
    link_3d_obj.append([
        # torre movel
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/4.obj',
                scale=scale, htm=link1_mth, mesh_material=mesh),
        # cilindro conector
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/5.obj',
                scale=scale, htm=link1_mth, mesh_material=mesh),
    ])

    link2_mth = Utils.inv_htm(Q02)
    link_3d_obj.append([
        # ligacao de cilindros curta
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/8.obj',
                scale=scale, htm=link2_mth, mesh_material=mesh),
    ])

    link3_mth = Utils.inv_htm(Q03)
    link_3d_obj.append([
        # cilindro conector
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/59.obj',
            scale=scale, htm=link3_mth, mesh_material=mesh),
        # ligacao de cilindros longa
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/9.obj',
            scale=scale, htm=link3_mth, mesh_material=mesh),
    ])

    link4_mth = Utils.inv_htm(Q04)
    link_3d_obj.append([
        # ligacao de cilindros longa
        Model3D(

            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/11.obj',
            scale=scale, htm=link4_mth, mesh_material=mesh),
    ])

    link5_mth = Utils.inv_htm(Q05)
    link_3d_obj.append([
        # cotovelo
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/10.obj',
            scale=scale, htm=link5_mth, mesh_material=mesh),
    ])

    link6_mth = Utils.inv_htm(Q06)
    link_3d_obj.append([
        # pá dobrada
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/38.obj',
            scale=scale, htm=link6_mth, mesh_material=mesh),
    ])

    link7_mth = Utils.inv_htm(Q07)
    link_3d_obj.append([
        # conecta pá com bumerangue
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/19.obj',
            scale=scale, htm=link7_mth, mesh_material=mesh),
    ])

    link8_mth = Utils.inv_htm(Q08)
    link_3d_obj.append([
        # bumerangue
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/21.obj',
            scale=scale, htm=link8_mth, mesh_material=mesh),

    ])

    link9_mth = Utils.inv_htm(Q09)
    link_3d_obj.append([
        # envoltoria agulha
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/43.obj',
            scale=scale, htm=link9_mth, mesh_material=mesh),
    ])

    link10_mth = Utils.inv_htm(Q010)
    link_3d_obj.append([
        # guia
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/45.obj',
                scale=scale, htm=link10_mth, mesh_material=mesh),
        # agulha
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/54.obj',
                scale=scale, htm=link10_mth, mesh_material=mesh),
        # abridor de garrafa
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/51.obj',
            scale=scale, htm=link10_mth, mesh_material=mesh),
        # botao
        Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/52.obj',
                scale=scale, htm=link10_mth, mesh_material=mesh),
    ])

    col_model = [[], [], [], [], [], [], [], [], [], []]

    col_model[0].append(Box(htm=Utils.trn([0, 0.05, -0.15]),
                            name=name + "_C0_0", width=0.14, height=0.4, depth=0.2, color="red", opacity=0.3))
    col_model[0].append(Cylinder(htm=Utils.trn([0, 0, -0.16]) @ Utils.rotz(3.14 / 2) @ Utils.rotx(3.14),
                                 name=name + "_C0_1", radius=0.075, height=0.19, color="red", opacity=0.3))

    col_model[2].append(Box(htm=Utils.trn([-0.200, 0, -0.11]) @ Utils.roty(np.pi/2),
                            name=name + "_C1_0", width=0.1, height=0.4, depth=0.12, color="green", opacity=0.3))
    col_model[2].append(Cylinder(htm=Utils.trn([0, 0, -0.12]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C1_1", radius=0.075, height=0.12, color="green", opacity=0.3))
    col_model[2].append(Cylinder(htm=Utils.trn([-0.415, 0, -0.11]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C1_2", radius=0.075, height=0.13, color="green", opacity=0.3))

    col_model[3].append(Box(htm=Utils.trn([-0.200, 0, 0.005]) @ Utils.roty(np.pi/2),
                            name=name + "_C2_0", width=0.1, height=0.4, depth=0.12, color="blue", opacity=0.3))
    col_model[3].append(Cylinder(htm=Utils.trn([0, 0, 0.01]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C2_1", radius=0.075, height=0.12, color="blue", opacity=0.3))
    col_model[3].append(Cylinder(htm=Utils.trn([-0.415, 0, 0.01]) @ Utils.rotz(np.pi/2) @ Utils.rotx(3.14),
                                 name=name + "_C2_2", radius=0.075, height=0.13, color="blue", opacity=0.3))

    col_model[4].append(Cylinder(htm=Utils.trn([0, -0.025, 0.01]) @ Utils.rotx(-alpha5),
                                 name=name + "_C3_0", radius=0.075, height=0.12, color="magenta", opacity=0.3))
    col_model[4].append(Box(htm=Utils.trn([0, -0.02, 0.12]) @ Utils.roty(np.pi/2),
                            name=name + "_C3_1", width=0.2, height=0.14, depth=0.14, color="magenta", opacity=0.3))

    col_model[5].append(Box(htm=Utils.trn([0, -0.355, -0.05]) @ Utils.roty(np.pi/2),
                            name=name + "_C4_0", width=0.2, height=0.08, depth=0.05, color="orange", opacity=0.3))
    col_model[5].append(Box(htm=Utils.trn([0, -0.15, -0.125]) @ Utils.roty(np.pi/2),
                            name=name + "_C4_1", width=0.05, height=0.09, depth=0.4, color="orange", opacity=0.3))

    col_model[6].append(Box(htm=Utils.trn([-0.2, 0.025, -0.025]),
                            name=name + "_C5_0", width=0.25, height=0.14, depth=0.15, color="cyan", opacity=0.3))
    col_model[6].append(Box(htm=Utils.trn([-0.02, 0.025, -0.088]),
                            name=name + "_C5_1", width=0.14, height=0.02, depth=0.15, color="cyan", opacity=0.3))
    col_model[6].append(Box(htm=Utils.trn([-0.019, 0.025, 0.019]),
                            name=name + "_C5_2", width=0.14, height=0.02, depth=0.15, color="cyan", opacity=0.3))

    col_model[7].append(Box(htm=Utils.trn([-0.42, 0.08, -0.034]) @ Utils.rotz(np.pi/3),
                            name=name + "_C6_0", width=0.23, height=0.082, depth=0.13, color="#88264a", opacity=0.3))
    col_model[7].append(Box(htm=Utils.trn([-0.22, 0.08, -0.034]) @ Utils.rotz(np.pi*7/18),
                            name=name + "_C6_1", width=0.11, height=0.09, depth=0.39, color="#88264a", opacity=0.3))
    col_model[7].append(Box(htm=Utils.trn([-0.023, 0.01, 0.005]) @ Utils.rotz(np.pi*7/18),
                            name=name + "_C6_2", width=0.08, height=0.01, depth=0.03, color="#88264a", opacity=0.3))
    col_model[7].append(Box(htm=Utils.trn([-0.023, 0.01, -0.068]) @ Utils.rotz(np.pi*7/18),
                            name=name + "_C6_3", width=0.08, height=0.01, depth=0.03, color="#88264a", opacity=0.3))
    col_model[7].append(Cylinder(htm=Utils.trn([0, 0, -0.068]),
                                 name=name + "_C6_4", radius=0.035, height=0.01, color="#88264a", opacity=0.3))
    col_model[7].append(Cylinder(htm=Utils.trn([0, 0, 0.005]),
                                 name=name + "_C6_5", radius=0.035, height=0.01, color="#88264a", opacity=0.3))

    col_model[8].append(Box(htm=Utils.trn([-0.052, 0.0, -0.042]),
                            name=name + "_C7_0", width=0.02, height=0.31, depth=0.12, color="Brown", opacity=0.3))
    col_model[8].append(Cylinder(htm=Utils.trn([-0.1, -0.005, -0.001]) @ Utils.rotx(np.pi/2),
                                 name=name + "_C7_1", radius=0.035, height=0.056, color="Brown", opacity=0.3))
    col_model[8].append(Box(htm=Utils.trn([-0.08, -0.005, 0]),
                            name=name + "_C7_2", width=0.04, height=0.07, depth=0.056, color="Brown", opacity=0.3))
    col_model[8].append(Box(htm=Utils.trn([-0.03, -0.001, 0.073]),
                            name=name + "_C7_3", width=0.04, height=0.05, depth=0.04, color="Brown", opacity=0.3))
    col_model[8].append(Cylinder(htm=Utils.trn([0, -0.001, 0.074]),
                                 name=name + "_C7_4", radius=0.02, height=0.056, color="Brown", opacity=0.3))

    col_model[9].append(Box(htm=Utils.trn([-0.005, 0.0, -0.22]),
                            name=name + "_C8_0", width=0.07, height=0.4, depth=0.12, color="MidnightBlue", opacity=0.3))
    col_model[9].append(Cylinder(htm=Utils.trn([0, 0, 0.135]),
                                 name=name + "_C8_1", radius=0.01, height=0.4, color="MidnightBlue", opacity=0.3))

    links = []
    for i in range(n):
        links.append(Link(i, theta=link_info[0, i], d=link_info[1, i], alpha=link_info[2, i], a=link_info[3, i], joint_type=link_info[4, i],
                          list_model_3d=link_3d_obj[i]))
        for j in range(len(col_model[i])):
           links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration
    #     1  2  3  4  5  6  7  8  9 10
    q0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    htm_n_eef = Utils.trn([0, 0, 0.3025]) 
    htm_base_0 = Utils.trn([-b1, -b2, b3])

    # Create joint limits
    # joint_limits = (np.pi / 180) * np.matrix([[-180, 180], [-180, 180], [-180, 180]])

    return links, base_3d_obj, htm_base_0, htm_n_eef, q0
