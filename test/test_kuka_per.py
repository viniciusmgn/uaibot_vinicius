import uaibot as ub
import numpy as np


robot = ub.Robot.create_kuka_kr5()

material_box = ub.MeshMaterial(color="#242526", roughness=1, metalness=1)
material_glass = ub.MeshMaterial(metalness=0.9, roughness=0.05, env_map_intensity=0.9, clearcoat=1, opacity=1, \
                              reflectivity=0.2, refraction_ratio=0.985, ior=1.52, specular_intensity=0.1,
                              specular_color="white", transmission=1, side="BackSide")
texture_wall = ub.Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

material_wall = ub.MeshMaterial(texture_map=texture_wall, roughness=1, metalness=1)

box1 = ub.Box(width=0.5, height=0.3, depth=0.2, color="blue", mesh_material=material_wall)
box2 = ub.Box(width=0.25, height=0.4, depth=0.6, color="red", mesh_material=material_wall)
cil1 = ub.Cylinder(radius=0.3, height=0.4, color="purple", mesh_material=material_wall)
ball1 = ub.Ball(radius=0.4, color="brown", mesh_material=material_wall)
smoothBox1 = ub.SmoothBox(width=0.2, height=0.3, depth=0.4, color="blue", mesh_material=material_wall)

obj1=smoothBox1
obj2=ball1

pointA = ub.Ball(radius=0.035, color="yellow")
pointB = ub.Ball(radius=0.035, color="green")

sim = ub.Simulation.create_sim_factory([obj1, obj2, pointA, pointB])

dt=0.001

obj1.add_ani_frame(0, htm=ub.Utils.trn([0,0,0.8]))
obj2.add_ani_frame(0, htm=ub.Utils.trn([0,0,0.3]))

for i in range(3000):

    if i%300==0:
        t1 = np.random.uniform(0 ,6)
        t2 = np.random.uniform(0, 6)
        t3 = np.random.uniform(0, 6)
        t4 = np.random.uniform(-2, 2)
        t5 = np.random.uniform(-2, 2)
        t6 = np.random.uniform(-2, 2)
        htm1 = ub.Utils.rotx(dt*t1)*ub.Utils.roty(dt*t2)*ub.Utils.rotz(dt*t3)*ub.Utils.trn([dt*t4,dt*t5,dt*t6])

        t1 = np.random.uniform(0 ,6)
        t2 = np.random.uniform(0, 6)
        t3 = np.random.uniform(0, 6)
        t4 = np.random.uniform(-2, 2)
        t5 = np.random.uniform(-2, 2)
        t6 = np.random.uniform(-2, 2)
        htm2 = ub.Utils.rotx(dt*t1)*ub.Utils.roty(dt*t2)*ub.Utils.rotz(dt*t3)*ub.Utils.trn([dt*t4,dt*t5,dt*t6])


    obj1.add_ani_frame(10 * dt * i, htm=obj1.htm*htm1)
    obj2.add_ani_frame(10 * dt * i, htm=obj2.htm*htm2)

    [pa,pb,d] = ub.Utils.compute_dist(obj1,obj2)

    pointA.add_ani_frame(10 * dt * i, htm=ub.Utils.trn(pa))
    pointB.add_ani_frame(10 * dt * i, htm=ub.Utils.trn(pb))

sim.save("D://","test_obj_col")
