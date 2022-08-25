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


def _create_davinci_arm2(color, opacity):

    if not Utils.is_a_color(color):
        raise Exception(
            "The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception(
            "The parameter 'opacity' should be a float between 0 and 1.")

    theta2 = np.deg2rad(-3)
    theta3 = np.deg2rad(98.5)
    theta4 = np.deg2rad(108.5)
    theta5 = np.deg2rad(-44)
    theta6 = np.deg2rad(-85.5)
    theta7 = np.deg2rad(-171)
    theta8 = np.deg2rad(61.7)

    d2 = 96e-3 * 1.2
    d4 = -96e-3 * 0.925
    d5 = 431.8e-3 * 1.419
    d8 = -2.9e-2

    alpha4 = np.deg2rad(90 + 36.5)
    alpha5 = np.pi/2
    alpha8 = np.deg2rad(90)

    a2 = 0.415
    a3 = 0.415
    a5 = -8e-4
    a6 = 0.27
    a7 = 0.474
    a8 = 0.098

    # Passive joints 1, 2, 3, 4, 7, 8
    # Active joints 5, 6, 9
    link_info = np.array([
        # "theta" rotation in z
        [0,  0,   0,       0,      0,      0,      0,       0, 0],
        # 0,  "d" translation in z
        [0, d2,   0,      d4,     d5,      0,      0,      d8, 0],
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
    b1 = -0.086
    b2 = 0.2634
    b3 = 1.3

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
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/58.obj',
                scale=scale, htm=link1_mth, mesh_material=mesh),
        # cilindro conector
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/63.obj',
                scale=scale, htm=link1_mth, mesh_material=mesh),
    ])

    link2_mth = Utils.inv_htm(Q02)
    link_3d_obj.append([
        # ligacao de cilindros longa
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/64.obj',
                scale=scale, htm=link2_mth, mesh_material=mesh),
    ])

    link3_mth = Utils.inv_htm(Q03)
    link_3d_obj.append([
        # ligacao de cilindros longa
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/61.obj',
                scale=scale, htm=link3_mth, mesh_material=mesh),
    ])

    link4_mth = Utils.inv_htm(Q04)
    link_3d_obj.append([
        # cotovelo
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/14.obj',
                scale=scale, htm=link4_mth, mesh_material=mesh),
    ])

    link5_mth = Utils.inv_htm(Q05)
    link_3d_obj.append([
        # pá dobrada
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/36.obj',
                scale=scale, htm=link5_mth, mesh_material=mesh),
    ])

    link6_mth = Utils.inv_htm(Q06)
    link_3d_obj.append([
        # conecta pá com bumerangue
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/34.obj',
                scale=scale, htm=link6_mth, mesh_material=mesh),
    ])

    link7_mth = Utils.inv_htm(Q07)
    link_3d_obj.append([
        # bumerangue
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/39.obj',
                scale=scale, htm=link7_mth, mesh_material=mesh),
    ])

    link8_mth = Utils.inv_htm(Q08)
    link_3d_obj.append([
        # envoltoria agulha
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/42.obj',
                scale=scale, htm=link8_mth, mesh_material=mesh),
    ])

    link9_mth = Utils.inv_htm(Q09)
    link_3d_obj.append([
        # guia
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/23.obj',
                    scale=scale, htm=link9_mth, mesh_material=mesh),
        # agulha
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/55.obj',
                scale=scale, htm=link9_mth, mesh_material=mesh),
        # abridor de garrafa
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/25.obj',
                scale=scale, htm=link9_mth, mesh_material=mesh),
        # botao
        Model3D(url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/53.obj',
                scale=scale, htm=link9_mth, mesh_material=mesh),
    ])

    links = []
    for i in range(n):
        links.append(Link(i, theta=link_info[0, i], d=link_info[1, i], alpha=link_info[2, i], a=link_info[3, i], joint_type=link_info[4, i],
                          list_model_3d=link_3d_obj[i]))
        # for j in range(len(col_model[i])):
        #    links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration
    #     1  2  3  4  5  6  7  8  9
    q0 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    htm_n_eef = Utils.trn([0, 0, 0.3323])  # np.identity(4)
    htm_base_0 = Utils.trn([-b1, -b2, b3])  # np.identity(4)

    # Create joint limits
    # joint_limits = (np.pi / 180) * np.matrix([[-180, 180], [-180, 180], [-180, 180]])

    return links, base_3d_obj, htm_base_0, htm_n_eef, q0