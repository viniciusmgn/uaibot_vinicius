from utils import *

from graphics.meshmaterial import *
from graphics.model3d import *

from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *

from .links import *


def _create_franka_ermika_3(htm, name, color, opacity):
    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not (Utils.is_a_name(name)):
        raise Exception(
            "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

    if not Utils.is_a_color(color):
        raise Exception("The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception("The parameter 'opacity' should be a float between 0 and 1.")


    #For test only
    hpi = 3.14/2
    link_info = [[0, 0, 0, 0, 0, 0, 0],
                 [0.333, 0, 0.316, 0, 0.384, 0, 0],  # "d" translation in z
                 [0, -hpi, hpi, hpi, -hpi, hpi, hpi],  # "alfa" rotation in x
                 [0, 0, 0, 0.0825, -0.0825, 0, 0.088],  # "a" translation in x 0.25
                 [0, 0, 0, 0, 0, 0, 0]]


    H01 = Utils.rotz(link_info[0][0])*Utils.trn([0,0,link_info[1][0]])*Utils.rotx(link_info[2][0])*Utils.trn([link_info[3][0],0,0])
    H02 = H01*Utils.rotz(link_info[0][1]) * Utils.trn([0, 0, link_info[1][1]]) * Utils.rotx(link_info[2][1]) * Utils.trn(
        [link_info[3][1], 0, 0])
    H03 = H02*Utils.rotz(link_info[0][2]) * Utils.trn([0, 0, link_info[1][2]]) * Utils.rotx(link_info[2][2]) * Utils.trn(
        [link_info[3][2], 0, 0])
    H04 = H03*Utils.rotz(link_info[0][3]) * Utils.trn([0, 0, link_info[1][3]]) * Utils.rotx(link_info[2][3]) * Utils.trn(
        [link_info[3][3], 0, 0])
    H05 = H04*Utils.rotz(link_info[0][4]) * Utils.trn([0, 0, link_info[1][4]]) * Utils.rotx(link_info[2][4]) * Utils.trn(
        [link_info[3][4], 0, 0])
    H06 = H05*Utils.rotz(link_info[0][5]) * Utils.trn([0, 0, link_info[1][5]]) * Utils.rotx(link_info[2][5]) * Utils.trn(
        [link_info[3][5], 0, 0])
    H07 = H06*Utils.rotz(link_info[0][6]) * Utils.trn([0, 0, link_info[1][6]]) * Utils.rotx(link_info[2][6]) * Utils.trn(
        [link_info[3][6], 0, 0])
    n = 7
    scale = 1


    # Collision model
    col_model = [[], [], [], [], [], [], []]


    # Create 3d objects


    base_3d_obj = [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/FrankaErmika/link0.obj',
        scale,
        Utils.trn([0, 0, 0]) ,
        MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5],
                     color=color, opacity=opacity))]

    link_3d_obj = []

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/FrankaErmika/link1.obj',
            scale,
            Utils.trn([0, 0, 0]),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=1))
        ]
    )


    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/FrankaErmika/link2.obj',
            scale,
            Utils.trn([0, 0, 0]),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=1))
        ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/FrankaErmika/link3.obj',
            scale,
            Utils.trn([0, 0, 0]),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=1))
        ]
    )

    link_3d_obj.append(
        [
        ]
    )

    link_3d_obj.append(
        [
        ]
    )
    link_3d_obj.append(
        [
        ]
    )
    link_3d_obj.append(
        [
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

    htm_base_0 = Utils.trn([0, 0, 0])

    q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    #Create joint limits
    joint_limits = (np.pi/180)*np.matrix([[-170,170],[-120,120],[-170,170],[-120,120],[-170,170],[-120,120],[-175,175]])

    return base_3d_obj, links, np.identity(4), np.identity(4), q0, joint_limits
