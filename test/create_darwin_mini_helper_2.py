from uaibot import *
import numpy as np
from robot._create_darwin_mini_arm import _create_darwin_mini_arm

opacity = 1
color = "silver"
name = "darwin_mini"

base_3d_obj, links, htm_a, q0, joint_limits = _create_darwin_mini_arm(np.identity(4), "arm_left", color, 1)
robot_arm_left = Robot(name+"__arm_left", links, base_3d_obj, Utils.trn([0,0.14,1]) * Utils.rotx(-3.14/2), np.identity(4), q0, True, joint_limits)
base_3d_obj, links, htm_a, q0, joint_limits = _create_darwin_mini_arm(np.identity(4), "arm_right", color, 1)
robot_arm_right = Robot(name+"__arm_right", links, base_3d_obj, Utils.rotz(3.14) * Utils.trn([0,0.14,1]) * Utils.rotx(-3.14/2), np.identity(4), q0, True, joint_limits)

chestpart1 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/darwin_chest.obj',
    0.004,
    Utils.trn([-0.3,-0.179,0.55]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                 opacity=opacity, side="DoubleSide"))
chestpart2 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/frm_body_01.obj',
    0.004,
    Utils.trn([-0.3,-0.179,0.55]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                 opacity=opacity, side="DoubleSide"))
chestpart3 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/XL-320.obj',
    0.004,
    Utils.trn([0.265, 0.405, 0.5]) * Utils.rotz(-3.14/2) * Utils.rotx(3.14/3+0.25) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=1, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chestpart4 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/XL-320.obj',
    0.004,
    Utils.trn([0.265, -0.405, 1.485]) * Utils.rotx(3.14) * Utils.rotz(-3.14/2) * Utils.rotx(3.14/3+0.25) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=1, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chest = RigidObject([chestpart1, chestpart2,chestpart3,chestpart4],name+"_chest")

headpart1 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/DarwinMini/darwin_head.obj',
    0.004,
    Utils.trn([-0.3,-0.179,0.55]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                 opacity=opacity, side="DoubleSide"))

head = RigidObject([headpart1],name+"_head")

sim = Simulation.create_sim_factory([robot_arm_left, robot_arm_right, chest, head])




sim.save("D:\\","darwin_mini_2")