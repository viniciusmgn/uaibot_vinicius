from uaibot import *
from scipy.linalg import null_space
import numpy as np


#Animação da junta revolta


sim = Simulation([], load_screen_color="#191919", background_color="#191919", width=500, height=500,
                 camera_type="orthographic")

texture_box = Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/gold_metal.png',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[1, 1])

material_box = MeshMaterial(texture_map=texture_box, roughness=1, metalness=1, opacity=0.8)
box = Box(htm = np.identity(4), width=0.1, depth=0.2, height=0.3, mesh_material=material_box)

light1 = PointLight(name="light1", color="white", intensity=4, htm=Utils.trn([-1, -1, 1.5]))
light2 = PointLight(name="light2", color="white", intensity=4, htm=Utils.trn([-1, 1, 1.5]))
light3 = PointLight(name="light3", color="white", intensity=4, htm=Utils.trn([1, -1, 1.5]))
light4 = PointLight(name="light4", color="white", intensity=4, htm=Utils.trn([1, 1, 1.5]))

style = "top:" + str(0.8 * sim.height) + "px;right:" + str(0) + "px;width:" + str(
    sim.width) + "px;position:absolute;text-align:center;background-color:#191919;color:white;font-smooth:always;font-family:arial"
explanation = HTMLDiv(html_text="", style=style)

frame = Frame(htm = np.identity(4), axis_color=["#FFA6A6", "#D4EA6B", "#729FCF"])
sim.set_parameters(load_screen_color="#191919", width=500, height=500)

sim.add([frame,explanation])

H1 = Utils.trn([1,0,0])
H2 = Utils.rotx(3.14/2)
H3 = Utils.trn([0,0.5,0])
H4 = Utils.roty(-3.14/4)

dt=0.01
k=0

explanation.add_ani_frame(k*dt, html_text= "<b style=\'color:#34eb5e\'>R<sub>x</sub>(&#960/4)</b>D<sub>y</sub>(-0.25)R<sub>z</sub>(&#960/2)D<sub>x</sub>(1)")
for i in range(400):
    htm = frame.htm  @ Utils.rotx((3.14/4)*(dt/4))
    frame.add_ani_frame(k*dt, htm = htm)
    k+=1

k+=400

explanation.add_ani_frame(k*dt, html_text= "<b style=\'color:#34eb5e\'>R<sub>x</sub>(&#960/4)D<sub>y</sub>(-0.25)</b>R<sub>z</sub>(&#960/2)D<sub>x</sub>(1)")
for i in range(400):
    htm = frame.htm @ Utils.trn([0,-0.25 * dt/4,0])
    frame.add_ani_frame(k*dt, htm = htm)
    k+=1

k+=400

explanation.add_ani_frame(k*dt, html_text= "<b style=\'color:#34eb5e\'>R<sub>x</sub>(&#960/4)D<sub>y</sub>(-0.25)R<sub>z</sub>(&#960/2)</b>D<sub>x</sub>(1)")
for i in range(400):
    htm = frame.htm @ Utils.rotz((3.14 / 2) * (dt / 4))
    frame.add_ani_frame(k*dt, htm = htm)
    k+=1

k+=400
explanation.add_ani_frame(k*dt, html_text= "<b style=\'color:#34eb5e\'>R<sub>x</sub>(&#960/4)D<sub>y</sub>(-0.25)R<sub>z</sub>(&#960/2)D<sub>x</sub>(1)</b>")
for i in range(400):
    htm = frame.htm @ Utils.trn([dt/4,0,0])
    frame.add_ani_frame(k*dt, htm = htm)
    k+=1

sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula2","anim3")
