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


def _create_davinci_arm(color, opacity):

    if not Utils.is_a_color(color):
        raise Exception(
            "The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception(
            "The parameter 'opacity' should be a float between 0 and 1.")
    a1 = -0.3
    a2 = -0.415#-0.35
    a3 = -0.407
    a5 = 0#40.09/1000
    a6 = 0

    r1 = 1.875 #0
    r2 = 3.25 
    r3 = -90.75

    alpha1 = np.deg2rad(r1)#np.deg2rad(0)
    alpha2 = np.deg2rad(r2 - r1)#np.deg2rad(5)
    alpha3 = np.deg2rad(r3)
    alpha4 = np.pi/2

    d2 = 0
    d4 = 0#(96)/1000
    d5 = 0#-(144.54 - 431.8)

    link_info = np.array([
        # "theta" rotation in z
        [alpha1,  alpha2,  alpha3, np.pi/2, -np.pi/2, 0,       0], # -> changed [0, 3] from -pi/2 to pi/2
        # "d" translation in z
        [0,           d2,       0,  d4,       d5,      0,       0],
        # "alfa" rotation in x
        [0,            0,       0,  alpha4,  -np.pi/2, np.pi/2, 0],
        # "a" translation in x
        [a1,          a2,      a3, 0,        a5,      a6,      0],
        # joint type
        [0,            0,       0,  0,        0,       1,       1]
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
    link_3d_obj = []
    mesh = MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5,
                        normal_scale=[0.5, 0.5], color=color,
                        opacity=opacity, side="DoubleSide")
    q_ = np.array([1, 0, 0, 0, 0, 0, 0]) * -np.pi/2 # original model is rotated (Robot fron = plane X x Y)
 # * Utils.trn([-0.1, -0.45, -1])
    Q01 = Utils.trn([0.1, 1.1, -0.45]) * Utils.rotx(q_[0]) * Utils.rotz(link_info[0, 0]) * Utils.trn([0, 0, link_info[1, 0]]) * Utils.rotx(link_info[2, 0]) * Utils.trn(
        [link_info[3, 0], 0, 0])
    Q02 = Q01 * (Utils.rotx(q_[1]) * Utils.rotz(link_info[0, 1]) * Utils.trn([0, 0, link_info[1, 1]]) * Utils.rotx(link_info[2, 1]) * Utils.trn(
        [link_info[3, 1], 0, 0]))
    Q23 = Q02 * (Utils.rotx(q_[2]) * Utils.rotz(link_info[0, 2]) * Utils.trn([0, 0, link_info[1, 2]]) * Utils.rotx(link_info[2, 2]) * Utils.trn(
        [link_info[3, 2], 0, 0]))
    Q34 = Q23 * (Utils.rotx(q_[3]) * Utils.rotz(link_info[0, 3]) * Utils.trn([0, 0, link_info[1, 3]]) * Utils.rotx(link_info[2, 3]) * Utils.trn(
        [link_info[3, 3], 0, 0]))
    Q45 = Q34 * (Utils.rotx(q_[4]) * Utils.rotz(link_info[0, 4]) * Utils.trn([0, 0, link_info[1, 4]]) * Utils.rotx(link_info[2, 4]) * Utils.trn(
        [link_info[3, 4], 0, 0]))
    Q56 = Q45 * (Utils.rotx(q_[5]) * Utils.rotz(link_info[0, 5]) * Utils.trn([0, 0, link_info[1, 5]]) * Utils.rotx(link_info[2, 5]) * Utils.trn(
        [link_info[3, 5], 0, 0]))
    Q67 = Q56 * (Utils.rotx(q_[6]) * Utils.rotz(link_info[0, 6]) * Utils.trn([0, 0, link_info[1, 6]]) * Utils.rotx(link_info[2, 6]) * Utils.trn(
        [link_info[3, 6], 0, 0]))

    link1_mth = Utils.inv_htm(Q01)
    link_3d_obj.append(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/8.obj',
            scale=scale, htm=link1_mth, mesh_material=mesh),
         ]
    )

    link2_mth = Utils.inv_htm(Q02) #* Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
       # [0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2)
    #link2_mth = Q02
    link_3d_obj.append(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/59.obj',
            scale=scale, htm=link2_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/9.obj',
            scale=scale, htm=link2_mth, mesh_material=mesh),
         ]
    )

    link3_mth = Utils.inv_htm(Q23) #* Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
        #[0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2)
    #link3_mth = Q23
    link_3d_obj.append(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/11.obj',
            scale=scale, htm=link3_mth, mesh_material=mesh),
         ]
    )

    link4_mth = Utils.inv_htm(Q34) #* Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
        #[0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2)
    #link4_mth = Q34
    link_3d_obj.append(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/10.obj',
            scale=scale, htm=link4_mth, mesh_material=mesh),
         ]
    )

    link5_mth = Utils.inv_htm(Q45) #* Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
        #[0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2)
    #link5_mth = Q45
    link_3d_obj.append(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/19.obj',
            scale=scale, htm=link5_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/21.obj',
            scale=scale, htm=link5_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/38.obj',
            scale=scale, htm=link5_mth, mesh_material=mesh),
         ]
    )

    link6_mth = Utils.inv_htm(Q56) #* Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
        #[0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2)
    #link6_mth = Q56
    link_3d_obj.append(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/43.obj',
            scale=scale, htm=link6_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/45.obj',
            scale=scale, htm=link6_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/51.obj',
            scale=scale, htm=link6_mth, mesh_material=mesh),
         Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/52.obj',
            scale=scale, htm=link6_mth, mesh_material=mesh),
         ]
    )

    link7_mth = Utils.inv_htm(Q67) #* Utils.trn([0.004, 0.19, -0.02]) * Utils.rotx(-3.14 / 12) * Utils.trn(
        #[0, 0.2, 0]) * Utils.rotz(3.14) * Utils.trn([0.318, -0.2, -0.3]) * Utils.roty(-3.14 / 2)
    link_3d_obj.append(
        [Model3D(
            url='https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinci3/54.obj',
            scale=scale, htm=link7_mth, mesh_material=mesh),
         ]
    )

    links = []
    for i in range(n):
        links.append(Link(i, link_info[0, i], link_info[1, i], link_info[2, i], link_info[3, i], link_info[4, i],
                          link_3d_obj[i]))

        #for j in range(len(col_model[i])):
        #    links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration
    q0 = [0, 0, 0, 0, 0, 0, 0]
    htm_n_eef = np.identity(4)
    htm_base_0 = np.identity(4)

    # Create joint limits
    #joint_limits = (np.pi / 180) * np.matrix([[-180, 180], [-180, 180], [-180, 180]])

    return links, base_3d_obj, htm_base_0, htm_n_eef, q0
