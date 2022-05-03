from uaibot import *
from scipy.linalg import null_space
import numpy as np



#Animação da junta revolta


sim = Simulation([], load_screen_color="#191919", background_color="#191919", width=500, height=500,
                 camera_type="orthographic")

robot = Robot.create_kuka_kr5()

light1 = PointLight(name="light1", color="white", intensity=4, htm=Utils.trn([-1, -1, 1.5]))
light2 = PointLight(name="light2", color="white", intensity=4, htm=Utils.trn([-1, 1, 1.5]))
light3 = PointLight(name="light3", color="white", intensity=4, htm=Utils.trn([1, -1, 1.5]))
light4 = PointLight(name="light4", color="white", intensity=4, htm=Utils.trn([1, 1, 1.5]))

vector_v = Vector(name="vector_v", color="cyan", thickness=1, origin=[0, 0, 0], vector=[0.5, 0.5, 0.5])
vector_w = Vector(name="vector_w", color="gold", thickness=1, origin=[0, 0, 0], vector=[0.5, 0.5, 0.5])


sim.add([robot, vector_v, vector_w])

jg, cd = robot.jac_geo()
h = null_space(jg.transpose())


v = h[0:3,:]
w = h[3:6,:]
print(v)
print(w)
vector_v.add_ani_frame(0, origin=cd[0:3, -1], vector=0.4 * v)
vector_w.add_ani_frame(0, origin=cd[0:3, -1], vector=0.4 * w)


sim.set_parameters(load_screen_color="#191919", width=500, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula4","anim5")

print(8.72370728/4.88843579)