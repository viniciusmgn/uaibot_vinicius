from utils import *

from graphics.meshmaterial import *
from graphics.model3d import *

from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *

from .links import *


def _create_staubli_tx60(htm, name, color, opacity):
    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not (Utils.is_a_name(name)):
        raise Exception(
            "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

    if not Utils.is_a_color(color):
        raise Exception("The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception("The parameter 'opacity' should be a float between 0 and 1.")

    c = (np.pi / 180)
    link_info = [[0, 0, 0, 0, 0, 0], #"theta" rotation in z
                 [0, 0, 0.02, 0.31, 0, 0.07],  # "d" translation in z
                 [3.14 / 2, 0, 3.14 / 2, 3.14 / 2, -3.14 / 2, 0],  # "alfa" rotation in x
                 [0, 0.29, 0, 0, 0, 0],  # "a" translation in x 0.25
                 [0, 0, 0, 0, 0, 0]] #joint type

    n = 6
    scale = 1

    # Collision model
    col_model = [[], [], [], [], [], []]

    # col_model[0].append(Box(htm=Utils.trn([-0.37, 0, 0]),
    #                             name=name + "_C0_0", width=0.25, depth=0.16, height=0.52, color="orange", opacity=0.3))


    #col_model[5].append(Ball(htm=np.identity(4),
    #                             name=name + "_C5_0", radius=0.0362, color="red", opacity=0.3))

    col_model[0].append(Cylinder(htm=Utils.trn([0, -0.15, 0]) @ Utils.rotx(-3.14 / 2),
                                 name=name + "_C0_0", radius=0.1, height=0.45, color="red", opacity=0.3))
    col_model[1].append(Box(htm=Utils.trn([-0.15, 0, -0.125]) @ Utils.rotz(3.14 / 2) @ Utils.rotx(-3.14 / 2 - c * 7.3),
                            name=name + "_C1_0", width=0.18, depth=0.1, height=0.45, color="blue", opacity=0.3))
    col_model[2].append(Box(htm=Utils.trn([0, -0.02, 0]),
                            name=name + "_C2_0", width=0.17, depth=0.2, height=0.25, color="green", opacity=0.3))
    col_model[3].append(Box(htm=Utils.trn([0, -0.08, 0]),
                            name=name + "_C3_0", width=0.15, depth=0.27, height=0.13, color="orange", opacity=0.3))

    # Create 3d objects

    base_3d_obj = [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/StaubliTX60/base_link.stl',
        scale,
        np.identity(4),
        MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5],
                     color=color, opacity=opacity))]

    link_3d_obj = []

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/StaubliTX60/link_1.stl',
            scale,
            Utils.trn([0, -0.005, 0]) @ Utils.rotx(-3.14 / 2),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
         ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/StaubliTX60/link_2.stl',
            scale,
            Utils.trn([-0.31, 0, 0.02]) @ Utils.rotz(-3.14 / 2) @ Utils.rotx(-3.14 / 2 + c * 7.3),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
         ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/StaubliTX60/link_3.stl',
            scale,
            Utils.rotz(-3.14) @ Utils.trn([0, 0.045 - 0.05, 0]),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
         ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/StaubliTX60/link_4.stl',
            scale,
            Utils.trn([0, -0.31, 0]) @ Utils.roty(3.14) @ Utils.rotx(-3.14 / 2),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
         ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/StaubliTX60/link_5.stl',
            scale,
            np.identity(4),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#C0C0C0",
                         opacity=opacity))
         ]
    )
    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/StaubliTX60/link_6.stl',
            scale,
            np.identity(4),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="black",
                         opacity=opacity))
         ]
    )

    # Inertial parameter of the robots

    list_com_coordinates = [[0, -0.15, 0],
                            [-0.15, 0, -0.125],
                            [0, -0.02, 0],
                            [0, -0.08, 0],
                            [0.0, 0.0, 0.03],
                            [0.0, 0.0, 0.0]]

    list_inertia_mat = []

    list_inertia_mat.append(np.array([[0.8386, 0.0000, 0.0000],
                                      [0.0000, 0.1001, 0.0002],
                                      [0.0000, 0.0002, 0.8386]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array( [[ 0.2198, 0.0000, -0.2150],
                                       [ 0.0000, 0.6621, -0.0000],
                                       [-0.2150,-0.0000,  0.4614]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array( [[ 0.0965, -0.0000,  0.0000],
                                       [-0.0000,  0.0691,  0.0000],
                                       [ 0.0000,  0.0000,  0.1077]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))


    list_inertia_mat.append(np.array( [[ 0.1070, -0.0001, -0.0000],
                                       [-0.0001,  0.0245,  0.0000],
                                       [-0.0000,  0.0000,  0.1035]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0., 0., 0.],
                                      [0., 0., 0.],
                                      [0., 0., 0.]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0., 0., 0.],
                                      [0., 0., 0.],
                                      [0., 0., 0.]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_mass = [20.026, 11.474, 12.040, 7.458, 0.300, 0.300]

    # Create links

    links = []
    for i in range(n):
        links.append(Link(i, link_info[0][i], link_info[1][i], link_info[2][i], link_info[3][i], link_info[4][i],
                          link_3d_obj[i], list_mass[i], list_com_coordinates[i], list_inertia_mat[i]))

        for j in range(len(col_model[i])):
            links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration

    htm_base_0 = Utils.trn([0, 0, 0.375])

    q0 = [0, 3.14/2, 0, 0, 0, 0]

    #Create joint limits
    joint_limits = (np.pi/180)*np.matrix([[-180,180],[-127.5+90, 127.5+90],[-142.5+90, 142.5+90],[-270,270],[-121,132.5],[-270,270]])

    return base_3d_obj, links, htm_base_0, np.identity(4), q0, joint_limits
