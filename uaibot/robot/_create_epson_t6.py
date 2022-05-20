from utils import *

from graphics.meshmaterial import *
from graphics.model3d import *

from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *

from .links import *


def _create_epson_t6(htm, name, color, opacity):
    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not (Utils.is_a_name(name)):
        raise Exception(
            "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

    if not Utils.is_a_color(color):
        raise Exception("The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception("The parameter 'opacity' should be a float between 0 and 1.")

    link_info = [[0, 0, 0], #"theta" rotation in z
                 [0, 0.000, 0],  # "d" translation in z
                 [0, 3.14, 0],  # "alfa" rotation in x
                 [1.3 / 4, 1.1 / 4, 0],  # "a" translation in x
                 [0, 0, 1]] #joint type

    n = 3

    # Collision model
    col_model = [[], [], []]

    # col_model[0].append(Box(htm=Utils.trn([-0.37, 0, 0]),
    #                             name=name + "_C0_0", width=0.25, depth=0.16, height=0.52, color="orange", opacity=0.3))

    col_model[0].append(Box(htm=Utils.trn([-0.15, 0, 0.03]),
                            name=name + "_C0_0", width=0.375, height=0.05, depth=0.1, color="red", opacity=0.3))

    col_model[1].append(Box(htm=Utils.trn([-0.14, 0, -0.15]),
                            name=name + "_C1_0", width=0.38, height=0.2, depth=0.143, color="green", opacity=0.3))

    col_model[2].append(Cylinder(htm=Utils.trn([0, 0, -0.2]),
                                 name=name + "_C2_0", radius=0.03, height=0.4, color="blue", opacity=0.3))

    # Create 3d objects

    base_3d_obj = [
        Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/EpsonT6/Base.obj',
                0.001,
                Utils.rotx(3.14 / 2),
                MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5],
                             color=color, opacity=opacity))]

    link_3d_obj = []

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/EpsonT6/T6Axis1.obj',
                 0.001,
                 Utils.trn([-1.3 / 4, 0, 0]) @ Utils.rotx(3.14 / 2),
                 MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity)),
         Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/EpsonT6/T6Cable.obj',
                 0.001,
                 Utils.trn([0, 0, 0.22]) @ Utils.roty(3.14 / 2) @ Utils.rotz(3.14 / 2),
                 MeshMaterial(color="black", opacity=opacity))
         ]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/EpsonT6/T6Axis2.obj',
                 0.001,
                 Utils.trn([0, 0, -0.06]) @ Utils.rotx(-3.14 / 2),
                 MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/EpsonT6/T6Axis3.obj',
                 0.001,
                 Utils.trn([0, 0, (-1.6) / 4]) @ Utils.rotx(-3.14 / 2),
                 MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    # Intertial parameters

    list_com_coordinates = [[-0.18, 0, 0.025],
                            [-0.15, 0, -0.15],
                            [0, 0, -0.2]]

    list_inertia_mat = []

    list_inertia_mat.append(np.array([[0.0012, 0., 0.0031],
                                      [0., 0.0392, 0.],
                                      [0.0031, 0., 0.0384]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0.1381, 0., -0.1128],
                                      [0., 0.2946, -0.],
                                      [-0.1128, -0., 0.1899]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0.028, 0., 0.],
                                      [0., 0.028, 0.],
                                      [0., 0., 0.0002]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_mass = [0.8652, 5.0149, 0.5219]

    # Create links

    links = []
    for i in range(n):
        links.append(Link(i, link_info[0][i], link_info[1][i], link_info[2][i], link_info[3][i], link_info[4][i],
                          link_3d_obj[i], list_mass[i], list_com_coordinates[i], list_inertia_mat[i]))

        for j in range(len(col_model[i])):
            links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration

    htm_base_0 = Utils.trn([0.25 / 4, 0, 0.8 / 4])

    q0 = [0.0, 0.0, 0.0]

    #Create joint limits
    c = (np.pi/180)
    joint_limits = np.matrix([[-c*132,c*132],[-c*141,c*141],[0+0.02,0.15+0.02]])

    return base_3d_obj, links, htm_base_0, np.identity(4), q0, joint_limits
