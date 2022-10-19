from uaibot import *
import numpy as np
from robot._create_darwin_mini_arm import _create_darwin_mini_arm
from robot._create_darwin_mini_leg_left import _create_darwin_mini_leg_left
from robot._create_darwin_mini_leg_right import _create_darwin_mini_leg_right

opacity = 1
color = "#3e3f42"
name = "darwin_mini"

desl_z = Utils.trn([0,0,-0.18])

base_3d_obj, links, htm_a, q0, joint_limits = _create_darwin_mini_arm(np.identity(4), "arm_left", color, 1)
robot_arm_left = Robot(name+"__arm_left", links, base_3d_obj, desl_z*Utils.trn([0,0.14,1]) * Utils.rotx(-3.14/2), np.identity(4), q0, True, joint_limits)

base_3d_obj, links, htm_a, q0, joint_limits = _create_darwin_mini_arm(np.identity(4), "arm_right", color, 1)
robot_arm_right = Robot(name+"__arm_right", links, base_3d_obj, desl_z*Utils.rotz(3.14) * Utils.trn([0,0.14,1]) * Utils.rotx(-3.14/2), np.identity(4), q0, True, joint_limits)

base_3d_obj, links, htm_a, q0, joint_limits = _create_darwin_mini_leg_right(np.identity(4), "leg_right", color, 1)
robot_leg_right= Robot(name+"__leg_right", links, base_3d_obj, desl_z*Utils.trn([0,0.1,0.7])*Utils.roty(np.pi/2) * Utils.rotz(-np.pi/2), np.identity(4), q0, True, joint_limits)

base_3d_obj, links, htm_a, q0, joint_limits = _create_darwin_mini_leg_left(np.identity(4), "leg_left", color, 1)
robot_leg_left= Robot(name+"__leg_left", links, base_3d_obj, desl_z*Utils.trn([0,-0.1,0.7])*Utils.roty(np.pi/2) * Utils.rotz(-np.pi/2), np.identity(4), q0, True, joint_limits)

chestpart1 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/darwin_chest.obj',
    0.004,
    desl_z*Utils.trn([-0.3,-0.179,0.55]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                 opacity=opacity, side="DoubleSide"))
chestpart2 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/frm_body_01.obj',
    0.004,
    desl_z*Utils.trn([-0.3,-0.179,0.55]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                 opacity=opacity, side="DoubleSide"))
chestpart3 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/XL-320.obj',
    0.004,
    desl_z*Utils.trn([0.265, 0.405, 0.5]) * Utils.rotz(-3.14/2) * Utils.rotx(3.14/3+0.25) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=1, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chestpart4 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/XL-320.obj',
    0.004,
    desl_z*Utils.trn([0.265, -0.405, 1.485]) * Utils.rotx(3.14) * Utils.rotz(-3.14/2) * Utils.rotx(3.14/3+0.25) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=1, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chestpart5 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPU-2.obj',
    0.004,
    desl_z*Utils.trn([0.05,0.13,0]) * Utils.rotz(3.14/2) * Utils.trn([-0.42,-0.179,0.575]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chestpart6 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/XL-320-2.obj',
    0.004,
    desl_z*Utils.trn([0.69,0.1,0.655]) * Utils.rotx(0.08) * Utils.roty(-3.14/2) * Utils.trn([-0.3,-0.179,0.15]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chestpart7 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/SPU-2.obj',
    0.004,
    desl_z*Utils.trn([0.05,-0.06,0]) * Utils.rotz(3.14/2) * Utils.trn([-0.42,-0.179,0.575]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chestpart8 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/XL-320-2.obj',
    0.004,
    desl_z*Utils.trn([0.69,-0.09,0.655]) * Utils.rotx(0.08) * Utils.roty(-3.14/2) * Utils.trn([-0.3,-0.179,0.15]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

chest = RigidObject([chestpart1, chestpart2, chestpart3, chestpart4, chestpart5, chestpart6, chestpart7, chestpart8],
                    name + "_chest")


headpart1 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/darwin_head.obj',
    0.004,
    desl_z*Utils.trn([-0.3,-0.179,0.55]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.5, clearcoat=0, roughness=0.5, normal_scale=[0.5, 0.5], color=color,
                 opacity=opacity, side="DoubleSide"))

headpart2 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/eye.obj',
    0.0009,
    desl_z*Utils.trn([0.177,-0.043,1.162]) * Utils.rotz(3.14/2-0.3),
    MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="blue",
                 opacity=opacity, side="DoubleSide"))
headpart3 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/eye.obj',
    0.0009,
    desl_z*Utils.trn([0.171, 0.053,1.162]) * Utils.rotz(3.14/2+0.4),
    MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="blue",
                 opacity=opacity, side="DoubleSide"))
headpart4 = Model3D(
    'https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/DarwinMini/XL-320-2.obj',
    0.004,
    desl_z*Utils.trn([-0.3,-0.179,0.55]) * Utils.rotz(3.14/2) * Utils.rotx(3.14/2),
    MeshMaterial(metalness=0.8, clearcoat=0.5, roughness=0.5, normal_scale=[0.5, 0.5], color="#302b2b",
                 opacity=opacity, side="DoubleSide"))

head = RigidObject([headpart1,headpart2,headpart3,headpart4],name+"_head")

sim = Simulation.create_sim_factory([robot_arm_left, robot_arm_right, chest, head, robot_leg_left, robot_leg_right])
#sim = Simulation([robot_arm_left, robot_arm_right, chest, head],camera_type="orthographic")



sim.save("D:\\","darwin_mini_2")