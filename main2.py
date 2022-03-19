from uaibot import *

robot = Robot.create_kuka_lbr_iiwa()

texture_wall = Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/Textures/metal.png',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

material_wall = MeshMaterial(texture_map=texture_wall, roughness=1, metalness=1)

wall1 = Box(name="wall1", htm=Utils.trn([0.3, -0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
            mesh_material=material_wall)
wall2 = Box(name="wall2", htm=Utils.trn([0.3, 0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
            mesh_material=material_wall)
wall3 = Box(name="wall3", htm=Utils.trn([0.3, 0, 0.25]), width=0.05, depth=0.4, height=0.5,
            mesh_material=material_wall)
wall4 = Box(name="wall4", htm=Utils.trn([0.3, 0, 1.15]), width=0.05, depth=0.4, height=0.5,
            mesh_material=material_wall)



# Create simulation
sim = Simulation.create_sim_factory([robot, wall1, wall2, wall3, wall4])

sim.save("D://","test_texture")