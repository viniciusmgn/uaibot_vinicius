from utils import *

from graphics.meshmaterial import *
from graphics.model3d import *

from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *

from .links import *


def _create_abb_crb(htm, name, color, opacity):
    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not (Utils.is_a_name(name)):
        raise Exception(
            "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

    if not Utils.is_a_color(color):
        raise Exception("The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception("The parameter 'opacity' should be a float between 0 and 1.")

    link_info = [[       0,     0,      3.14,         0, 0, 0],  # "theta" rotation in z
                 [   0.265,     0,         0,     -0.47, 0, 0.101],  # "d" translation in z
                 [3.14 / 2,     0, -3.14 / 2, -3.14 / 2, -3.14 / 2, 0],  # "alfa" rotation in x
                 [       0, 0.444,      0.11,         0, 0.08, 0],  # "a" translation in x
                 [       0,      0,        0,         0, 0, 0]]  # joint type

    scale = 1
    n = 6

    # Collision model
    col_model = [[], [], [], [], [], []]

    # Create 3d objects

    base_3d_obj = [
        Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/base_link.stl',
            scale,
            np.identity(4),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                         opacity=opacity))]

    link_3d_obj = []

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_1.stl',
                 scale,
                 Utils.rotx(-3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_2.stl',
                 scale,
                 Utils.trn([-0.444, 0, 0]) @ Utils.rotx(-3.14 / 2) @ Utils.roty(3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_3.stl',
                 scale,
                 Utils.trn([-0.11, 0, 0]) @ Utils.roty(3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#606060",
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_4.stl',
                 scale,
                 Utils.trn([0, -0.47, 0]) @ Utils.roty(-3.14 / 2) @ Utils.rotz(3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_5.stl',
                 scale,
                 Utils.trn([-0.08, 0, 0]) @ Utils.rotz(3.14 / 2) @ Utils.roty(-3.14 / 2) @ Utils.rotx(3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/ABBCRB15000/link_6.stl',
                 scale,
                 Utils.trn([0, 0, 0]) @ Utils.roty(-3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="silver",
                              opacity=opacity))]
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
    q0 = [0.000, 0.000, 3.14, 0.000, 0.000, 0.000]

    #Create joint limits
    joint_limits = (np.pi/180)*np.matrix([[-180,180],[-180,180],[-225-200+10,85-200-20],[-180,180],[-180,180],[-180,180]])

    return base_3d_obj, links, np.identity(4), np.identity(4), q0, joint_limits
