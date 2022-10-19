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

frame = Frame(htm = np.identity(4))
style = "top:" + str(0.8 * sim.height) + "px;right:" + str(0) + "px;width:" + str(
    sim.width) + "px;position:absolute;text-align:center;background-color:#191919;color:white;font-smooth:always;font-family:arial"
explanation = HTMLDiv(html_text="", style=style)

sim.add([box, frame,light1,light2,light3,light4, explanation])

dt=0.01
htm = box.htm

for i in range(3000):
    htm =  Utils.trn([0.9*cos(i*dt),0.9*sin(i*dt), 0.5+0.15*sin(0.3*i*dt)]) @ Utils.rotx(1*i*dt) @ Utils.roty(0.5*i*dt)
    box.add_ani_frame(i*dt,htm=htm)
    frame.add_ani_frame(i*dt,htm=htm)
    H = box.htm
    table = "<div style=\'item-align:center;text-align:center;width:500px\'><div style=\'width:20%;display: inline-block;float:left;vertical-align: middle\'>H(" + str(
        round(
            100 * i * dt) / 100) + "s) = </div><div style=\'width:80%;display: inline-block;float:right;\'><table style=\'color:white;text-align:right\'><tr><td width=\'60px\'>" + str(
        round(100 * H[0, 0]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[0, 1]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[0, 2]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[0, 3]) / 100) + "</td></tr><tr><td width=\'60px\'>" + str(
        round(100 * H[1, 0]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[1, 1]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[1, 2]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[1, 3]) / 100) + "</td></tr><tr><td width=\'60px\'>" + str(
        round(100 * H[2, 0]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[2, 1]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[2, 2]) / 100) + "</td><td width=\'60px\'>" + str(
        round(100 * H[
            2, 3]) / 100) + "</td></tr><tr><td width=\'60px\'>   0</td><td width=\'60px\'>   0</td><td width=\'60px\'>   0</td><td width=\'60px\'>   1</td></tr></table></div></div>"
    explanation.add_ani_frame(i*dt, html_text=table)


sim.set_parameters(load_screen_color="#191919", width=500, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula2","anim1")
