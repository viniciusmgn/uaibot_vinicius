from utils import *

from graphics.meshmaterial import *
from graphics.model3d import *

from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *

from .links import *


def _create_kuka_lbr_iiwa(htm, name, color, opacity):
    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not (Utils.is_a_name(name)):
        raise Exception(
            "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

    if not Utils.is_a_color(color):
        raise Exception("The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception("The parameter 'opacity' should be a float between 0 and 1.")

    link_info = [[0, 0, 0, 0, 0, 0, 0],
                 [0.3105 + 0.0495, 0, 0.4 + 0.02, 0, 0.39 + 0.01, 0, 0.078 + 0.05],  # "d" translation in z
                 [3.14 / 2, -3.14 / 2, -3.14 / 2, 3.14 / 2, 3.14 / 2, -3.14 / 2, 0],  # "alfa" rotation in x
                 [0, 0, 0, 0, 0, 0, 0],  # "a" translation in x 0.25
                 [0, 0, 0, 0, 0, 0, 0]]

    n = 7
    scale = 1

    # Collision model
    col_model = [[], [], [], [], [], [], []]

    col_model[0].append(Cylinder(htm=Utils.trn([0, -0.14, 0]) @ Utils.rotx(3.14 / 2),
                                 name=name + "_C0_0", radius=0.10, height=0.12, color="red", opacity=0.3))
    col_model[0].append(Ball(htm=Utils.trn([0, -0.03, 0]) @ Utils.rotx(3.14 / 2),
                             name=name + "_C0_1", radius=0.12, color="red", opacity=0.3))
    col_model[1].append(Ball(htm=Utils.trn([0, -0.035, 0.01]),
                             name=name + "_C1_0", radius=0.1, color="green", opacity=0.3))
    col_model[1].append(Cylinder(htm=Utils.trn([0, 0, 0.14]),
                                 name=name + "_C1_1", radius=0.075, height=0.125, color="green", opacity=0.3))
    col_model[2].append(Cylinder(htm=Utils.trn([0, 0.17, 0]) @ Utils.rotx(3.14 / 2),
                                 name=name + "_C2_0", radius=0.08, height=0.1, color="blue", opacity=0.3))
    col_model[2].append(Ball(htm=Utils.trn([0, 0.035, -0.01]),
                             name=name + "_C2_1", radius=0.11, color="blue", opacity=0.3))
    col_model[3].append(Ball(htm=Utils.trn([0, 0.035, 0.01]),
                             name=name + "_C3_0", radius=0.08, color="orange", opacity=0.3))
    col_model[3].append(Cylinder(htm=Utils.trn([0, 0, 0.12]),
                                 name=name + "_C3_1", radius=0.075, height=0.125, color="orange", opacity=0.3))
    col_model[4].append(Cylinder(htm=Utils.trn([0, -0.165 + 0.005, 0]) @ Utils.rotx(3.14 / 2),
                                 name=name + "_C4_0", radius=0.075, height=0.11, color="magenta", opacity=0.3))
    col_model[4].append(Box(htm=Utils.trn([0, -0.03, 0.08]),
                            name=name + "_C4_1", width=0.12, depth=0.17, height=0.04, color="magenta", opacity=0.3))
    col_model[5].append(Cylinder(htm=Utils.trn([0, 0, 0.023]),
                                 name=name + "_C5_0", radius=0.075, height=0.21, color="cyan", opacity=0.3))

    # Create 3d objects

    base_3d_obj = [Model3D(
        'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/base_link.dae',
        scale,
        Utils.rotx(3.14 / 2),
        MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5],
                     color=color, opacity=opacity))]

    link_3d_obj = []

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/link_1.dae',
            scale,
            Utils.trn([0, -(0.3105 + 0.0495), 0]) @ Utils.roty(-3.14),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
        ]
    )

    # Utils.trn([-0.29,0,0]) @ Utils.rotz(-3.14/2) @ Utils.rotx(-3.14/2+c*7.3)
    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/link_2.dae',
            scale,
            Utils.rotz(3.14) @ Utils.rotx(3.14 / 2),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
        ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/link_3.dae',
            scale,
            Utils.trn([0, 0.4 + 0.02, 0]) @ Utils.rotz(3.14),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
        ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/link_4.dae',
            scale,
            Utils.rotz(3.14) @ Utils.rotx(3.14 / 2),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
        ]
    )

    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/link_5.dae',
            scale,
            Utils.trn([0, -(0.39 + 0.01), 0]),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
        ]
    )
    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/link_6.dae',
            scale,
            Utils.rotx(3.14 / 2),
            MeshMaterial(metalness=0.3, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                         opacity=opacity))
        ]
    )
    link_3d_obj.append(
        [Model3D(
            'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/KukaLBRIIWA/link_7.dae',
            scale,
            Utils.trn([0, 0, -(0.08 + 0.05)]) @ Utils.rotx(3.14 / 2),
            MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="#707070",
                         opacity=opacity))
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
