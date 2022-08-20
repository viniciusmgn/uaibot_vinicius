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


def _create_davinci_chest(name, color, opacity):

    if not Utils.is_a_color(color):
        raise Exception(
            "The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception(
            "The parameter 'opacity' should be a float between 0 and 1.")
    a1 = 0
    a2 = 0
    a3 = 0
    a5 = 0  # 40.09/1000
    a6 = 0

    alpha4 = 0

    d2 = 0
    d4 = 0  # (96)/1000
    d5 = 0  # -(144.54 - 431.8)

    link_info = np.array([
        # "theta" rotation in z
        [0,  0,  0, 0, 0, 0,       0],  # -> changed [0, 3] from -pi/2 to pi/2
        # "d" translation in z
        [0,  d2, 0,  d4,       d5,      0,       0],
        # "alfa" rotation in x
        [0,  0,  0,  alpha4,  -np.pi/2, np.pi/2, 0],
        # "a" translation in x
        [a1, a2, a3, 0,        a5,      a6,      0],
        # joint type
        [0,  0,  0,  0,        0,       1,       1]
    ])

    # link_info = np.array([
    #     # "theta" rotation in z
    #     [0,  0,  0, -np.pi/2, -np.pi/2, 0,       0],
    #     # "d" translation in z
    #     [0,  d2, 0,  d4,       d5,      0,       0],
    #     # "alfa" rotation in x
    #     [0,  0,  0,  alpha4,  -np.pi/2, np.pi/2, 0],
    #     # "a" translation in x
    #     [a1, a2, a3, 0,        a5,      a6,      0],
    #     # joint type
    #     [0,  0,  0,  0,        0,       1,       1]
    # ])

    scale = 1
    n = link_info.shape[1]
    base_3d_obj = []
    chest_obj = []
    mesh = MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5,
                        normal_scale=[0.5, 0.5], color=color,
                        opacity=opacity, side="DoubleSide")
    # original model is rotated (Robot fron = plane X x Y)
    q_ = np.array([1, 0, 0, 0, 0, 0, 0]) * -np.pi/2

    Q01 = Utils.rotx(q_[0]) * Utils.rotz(np.pi) * Utils.rotz(link_info[0, 0]) * Utils.trn([0, 0, link_info[1, 0]]) * Utils.rotx(link_info[2, 0]) * Utils.trn(
        [link_info[3, 0], 0, 0])
    Q12 = Q01 * (Utils.rotx(q_[1]) * Utils.rotz(link_info[0, 1]) * Utils.trn([0, 0, link_info[1, 1]]) * Utils.rotx(link_info[2, 1]) * Utils.trn(
        [link_info[3, 1], 0, 0]))
    Q23 = Q12 * (Utils.rotx(q_[2]) * Utils.rotz(link_info[0, 2]) * Utils.trn([0, 0, link_info[1, 2]]) * Utils.rotx(link_info[2, 2]) * Utils.trn(
        [link_info[3, 2], 0, 0]))
    Q34 = Q23 * (Utils.rotx(q_[3]) * Utils.rotz(link_info[0, 3]) * Utils.trn([0, 0, link_info[1, 3]]) * Utils.rotx(link_info[2, 3]) * Utils.trn(
        [link_info[3, 3], 0, 0]))
    Q45 = Q34 * (Utils.rotx(q_[4]) * Utils.rotz(link_info[0, 4]) * Utils.trn([0, 0, link_info[1, 4]]) * Utils.rotx(link_info[2, 4]) * Utils.trn(
        [link_info[3, 4], 0, 0]))
    Q56 = Q45 * (Utils.rotx(q_[5]) * Utils.rotz(link_info[0, 5]) * Utils.trn([0, 0, link_info[1, 5]]) * Utils.rotx(link_info[2, 5]) * Utils.trn(
        [link_info[3, 5], 0, 0]))
    Q67 = Q56 * (Utils.rotx(q_[6]) * Utils.rotz(link_info[0, 6]) * Utils.trn([0, 0, link_info[1, 6]]) * Utils.rotx(link_info[2, 6]) * Utils.trn(
        [link_info[3, 6], 0, 0]))

    # * Utils.trn([-0.315, 0.435, -0.325]) * Utils.rotz(-3.14) * Utils.roty(-3.14 / 2)
    link1_mth = Utils.inv_htm(Q01)
    #link1_mth = Q01
    chest_obj.extend(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/1.obj', #feet
            scale=scale, htm=link1_mth, mesh_material=mesh),
         ]
    )

    # * Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
    link2_mth = Utils.inv_htm(Q12)
    # [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2)
    #link2_mth = Q12
    chest_obj.extend(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/2.obj', #base rectangle
            scale=scale, htm=link2_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/3.obj', #vertical tower
            scale=scale, htm=link2_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/17.obj',
            scale=scale, htm=link2_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/18.obj', #short cable
            scale=scale, htm=link2_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/32.obj', #long cable
            scale=scale, htm=link2_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/56.obj',
            scale=scale, htm=link2_mth, mesh_material=mesh),
            Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/57.obj',
            scale=scale, htm=link2_mth, mesh_material=mesh),
         ]
    )

    chest = RigidObject(chest_obj, name + "_chest")

    # for j in range(len(col_model[i])):
    #    links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration
    q0 = [0, 0, 0, 0, 0, 0, 0]
    htm_base_0 = np.identity(4)
    htm_n_eef = np.identity(4)

    # Create joint limits
    #joint_limits = (np.pi / 180) * np.matrix([[-180, 180], [-180, 180], [-180, 180]])

    return chest
