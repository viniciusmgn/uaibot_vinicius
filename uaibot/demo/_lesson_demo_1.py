from simobjects.box import *
import numpy as np

mesh_box = jb.MeshMaterial(texture_map='http://127.0.0.1:8887/metal.png', roughness=1, metalness=1, opacity=0.5)
box = jb.Box(name="movingBox", width=0.3, height=0.4, depth=0.2, mesh_material=mesh_box)

light1 = jb.PointLight(name="light1", color="white", intensity=6, position=[2, 0, 3])
light2 = jb.PointLight(name="light2", color="white", intensity=6, position=[-2, 0, 3])
light3 = jb.PointLight(name="light3", color="white", intensity=6, position=[0, 0, 3])
light4 = jb.PointLight(name="light4", color="white", intensity=6, position=[0, 1, 1])

lin_velocity = jb.Vector(name="linVelocity", color="cyan")
ang_velocity = jb.Vector(name="angVelocity", color="orange")

sim = jb.Simulation([box, light1, light2, light3, light4, lin_velocity, ang_velocity])

dt = 0.01

pose = lambda t: jb.Utils.trn([np.cos(0.3 * t), np.sin(0.5 * t), 0.5 * np.sin(0.5 * t) + 1]) @ jb.Utils.rotz(
    0.2 * t) @ jb.Utils.rotx(0.1 * t)

for i in range(3000):
    htm = pose(i * dt)
    dot_htm = (pose((i + 1) * dt) - pose((i - 1) * dt)) / (2 * dt)
    mat_s = dot_htm[0:3, 0:3] @ np.transpose(htm[0:3, 0:3])

    lin_v = dot_htm[0:3, 3]
    lin_w = [2 * mat_s[2][1], 2 * mat_s[0][2], 2 * mat_s[1][0]]

    box.add_ani_frame(time=i * dt, htm=htm)
    lin_velocity.add_ani_frame(time=i * dt, origin=htm[0:3, 3], vector=lin_v)
    ang_velocity.add_ani_frame(time=i * dt, origin=htm[0:3, 3], vector=lin_w)

sim.run()
