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

vector_v = Vector(name="vector_v", color="cyan", thickness=1, origin=[0, 0, 0], vector=[0.5, 0.5, 0.5])
vector_w = Vector(name="vector_w", color="gold", thickness=1, origin=[0, 0, 0], vector=[0.5, 0.5, 0.5])

frame = Frame(htm = np.identity(4))
style = "top:" + str(0.8 * sim.height) + "px;right:" + str(0) + "px;width:" + str(
    sim.width) + "px;position:absolute;text-align:center;background-color:#191919;color:white;font-smooth:always;font-family:arial"
explanation = HTMLDiv(html_text="", style=style)

sim.add([box,light1,light2,light3,light4, explanation, vector_v, vector_w])

dt=0.01
htm = box.htm

fun = lambda t: Utils.trn([0.9*cos(t),0.9*sin(t), 0.5+0.15*sin(0.3*t)]) @ Utils.rotx(1*t) @ Utils.roty(0.5*t)
fun = lambda t: Utils.trn([1.1*cos(t),0, 0]) @ Utils.rotz(1*t)

for i in range(3000):
    htm =  fun(i*dt)
    d_htm_dt = (fun((i+1)*dt)-fun((i-1)*dt))/(2*dt)
    Q = htm[0:3,0:3]
    S_omega = d_htm_dt[0:3,0:3] @ np.transpose(Q)
    v = d_htm_dt[0:3,-1]
    w = np.array([S_omega[2,1] , S_omega[0,2], S_omega[1,0]]).reshape((3,1))

    box.add_ani_frame(i*dt,htm=htm)
    frame.add_ani_frame(i*dt,htm=htm)
    vector_v.add_ani_frame(i * dt, origin=htm[0:3,-1], vector=0.4*v)
    vector_w.add_ani_frame(i * dt, origin=htm[0:3,-1], vector=0.4*w)

    H = box.htm
    info = "<p style=\'color:cyan\'> v = [" + str(round(100 * v[0]) / 100) + " " + str(
        round(100 * v[1]) / 100) + " " + str(round(100 * v[2]) / 100) + "]</p>"
    info += "<p style=\'color:gold\'> w = [" + str(round(100 * w[0,0]) / 100) + " " + str(
        round(100 * w[1,0]) / 100) + " " + str(round(100 * w[2,0]) / 100) + "]</p>"
    explanation.add_ani_frame(i*dt, html_text=info)


sim.set_parameters(load_screen_color="#191919", width=500, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula4","anim2")
