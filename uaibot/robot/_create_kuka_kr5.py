from utils import *

from graphics.meshmaterial import *
from graphics.model3d import *

from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *

from .links import *


def _create_kuka_kr5(htm, name, color, opacity):
    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not (Utils.is_a_name(name)):
        raise Exception(
            "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

    if not Utils.is_a_color(color):
        raise Exception("The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception("The parameter 'opacity' should be a float between 0 and 1.")

    link_info = [[0, 0, 0, 0, 0, 0], #"theta" rotation in z
                 [0.335, 0.000, 0.000, -0.405, 0.000, 0.080],  # "d" translation in z
                 [-1.570, 0.000, 1.570, -1.570, -1.570, 0],  # "alfa" rotation in x
                 [0.075, 0.365, 0.090, 0.000, 0.000, 0],  # "a" translation in x
                 [0, 0, 0, 0, 0, 0]] #joint type

    n = 6

    # Collision model
    col_model = [[], [], [], [], [], []]

    col_model[0].append(Cylinder(htm=Utils.trn([-0.075, 0.170, 0]) @ Utils.roty(3.14 / 2) @ Utils.rotx(3.14 / 2),
                                 name=name + "_C0_0", radius=0.12, height=0.33, color="red", opacity=0.3))

    col_model[0].append(Cylinder(htm=Utils.trn([0, 0, 0.030]) @ Utils.rotz(3.14 / 2) @ Utils.rotx(3.14),
                                 name=name + "_C0_1", radius=0.095, height=0.30, color="red", opacity=0.3))

    col_model[1].append(Box(htm=Utils.trn([-0.200, 0.020, 0.12]) @ Utils.roty(3.14 / 2),
                            name=name + "_C1_0", width=0.1, height=0.5, depth=0.16, color="green", opacity=0.3))

    col_model[1].append(Cylinder(htm=Utils.trn([0, 0, 0.040]) @ Utils.rotz(3.14) @ Utils.rotx(3.14),
                                 name=name + "_C1_1", radius=0.095, height=0.28, color="green", opacity=0.3))

    col_model[2].append(Box(htm=Utils.trn([0, 0, -0.224]) @ Utils.rotz(-3.14 / 2) @ Utils.rotx(-3.14 / 2),
                            name=name + "_C2_0", width=0.143, height=0.12, depth=0.45, color="blue", opacity=0.3))

    # Create 3d objects

    base_3d_obj = [
        Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/KukaKR5/Base.obj',
                0.001,
                Utils.rotz(1.5 * 3.14) @ Utils.rotx(3.14 / 2),
                MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#242526",
                             opacity=opacity))]

    link_3d_obj = []

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/KukaKR5/Axis1.obj',
                 0.001,
                 Utils.trn([-0.08, 0.135, 0]) @ Utils.rotx(3.14),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/KukaKR5/Axis2.obj',
                 0.001,
                 Utils.trn([-0.37, 0, 0.11]) @ Utils.rotz(-3.14 / 2 + 3.14 / 13) @ Utils.rotx(3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/KukaKR5/Axis3.obj',
                 0.001,
                 Utils.trn([-0.09, 0, 0]) @ Utils.rotx(0) @ Utils.rotz(-3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/KukaKR5/Axis4.obj',
                 0.001,
                 Utils.trn([0, -0.185, 0]) @ Utils.rotz(3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/KukaKR5/Axis5.obj',
                 0.001,
                 Utils.trn([0, 0, 0]) @ Utils.roty(3.14),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                              opacity=opacity))]
    )

    link_3d_obj.append(
        [Model3D('https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/KukaKR5/Axis6.obj',
                 0.001,
                 Utils.trn([0, 0, -0.012]) @ Utils.rotx(3.14 / 2),
                 MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#242526",
                              opacity=opacity))]
    )

    # Inertial parameter of the robots

    list_com_coordinates = [[-0.035, 0.08, 0.016],
                            [-0.106, 0.011, 0.082],
                            [0.0, 0.0, -0.114],
                            [0.0, -0.08, 0.0],
                            [0.0, 0.0, 0.03],
                            [0.0, 0.0, 0.0]]

    list_inertia_mat = []

    list_inertia_mat.append(np.array([[1.028, 0.28, 0.],
                                      [0.28, 0.466, 0.],
                                      [0., 0., 0.991]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[1.309, 0.115, 0.69],
                                      [0.115, 1.91, -0.069],
                                      [0.69, -0.069, 1.926]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0.177, 0., 0., ],
                                      [0., 0.188, 0.],
                                      [0., 0., 0.049]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0.17, 0., 0.],
                                      [0., 0.087, 0.],
                                      [0., 0., 0.132]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0.002, 0., 0.],
                                      [0., 0.001, 0.],
                                      [0., 0., 0.001]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_inertia_mat.append(np.array([[0., 0., 0.],
                                      [0., 0., 0.],
                                      [0., 0., 0.]]) + np.diag([3.0e-4, 3.0e-4, 4.5e-4]))

    list_mass = [46.454, 54.361, 11.101, 14.185, 0.899, 0]

    # Create links

    links = []
    for i in range(n):
        links.append(Link(i, link_info[0][i], link_info[1][i], link_info[2][i], link_info[3][i], link_info[4][i],
                          link_3d_obj[i], list_mass[i], list_com_coordinates[i], list_inertia_mat[i]))

        for j in range(len(col_model[i])):
            links[i].attach_col_object(col_model[i][j], col_model[i][j].htm)

    # Define initial configuration
    q0 = [1.570, -1.570, 0.000, 0.000, 0, 0.000]

    return base_3d_obj, links, np.identity(4), q0
