from utils import *

from graphics.meshmaterial import *
from graphics.model3d import *

from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *
from simobjects.rigidobject import *
from simobjects.group import *

#from .links import *

#from robot._create_darwin_mini_arm import _create_darwin_mini_arm
#from robot._create_darwin_mini_leg_left import _create_darwin_mini_leg_left
#from robot._create_darwin_mini_leg_right import _create_darwin_mini_leg_right

import pickle




def _create_davinci(htm, name, color, opacity):

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not (Utils.is_a_name(name)):
        raise Exception(
            "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

    if not Utils.is_a_color(color):
        raise Exception("The parameter 'color' should be a HTML-compatible color.")

    if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
        raise Exception("The parameter 'opacity' should be a float between 0 and 1.")

    desl_z = htm * Utils.trn([0, 0, -0.18])

    #param_arm_left = _create_darwin_mini_arm(np.identity(4), "arm_left", color, 1)
    #param_arm_right = _create_darwin_mini_arm(np.identity(4), "arm_right", color, 1)
    #param_leg_left = _create_darwin_mini_leg_left(np.identity(4), "leg_left", color, 1)
    #param_leg_right = _create_darwin_mini_leg_right(np.identity(4), "leg_right", color, 1)
    parts = []
    with open('/home/fbartelt/Documents/UFMG/TCC/Sim/uaibot/contents/DaVinciSi/mths', 'rb') as f:
        mths = pickle.load(f)

    for i in range(106):
        path = 'https://raw.githubusercontent.com/fbartelt/uaibot/master/contents/DaVinciSi/p' + str(i) + '.obj'
        parts.append(Model3D(path, htm=desl_z * mths['p' + str(i)],
        mesh_material=MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                     opacity=opacity, side="DoubleSide")
        ))


    chest = RigidObject(parts, name + "_chest")
    return chest
