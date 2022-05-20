from uaibot import *
from scipy.linalg import null_space
import numpy as np


#Animação da junta revolta


sim = Simulation([], load_screen_color="#191919", background_color="#191919", width=500, height=500)

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
sim.add([box, frame,light1,light2,light3,light4])

dt=0.01
htm = box.htm

for i in range(3000):
    htm =  Utils.trn([0.9*cos(i*dt),0.9*sin(i*dt), 0.5+0.15*sin(0.3*i*dt)]) @ Utils.rotx(1*i*dt) @ Utils.roty(0.5*i*dt)
    box.add_ani_frame(i*dt,htm=htm)
    frame.add_ani_frame(i*dt,htm=htm)

##sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images","descricao_espacial")

#Controle cinemático

robot = Robot.create_kuka_lbr_iiwa()

sim = Simulation([robot], background_color="#191919", load_screen_color="#191919", width=500, height=500)

htm_des = Utils.trn([0.2, 0.2, -0.4]) @ robot.fkm()@ Utils.rotx(-3.14/2)

frame = Frame(htm=htm_des)

sim.add(frame)

k=0
dt=0.005
for i in range(8):
    #alpha = np.random.uniform(0,1)
    #q0 = alpha * robot.joint_limit[:,0] + (1-alpha) * robot.joint_limit[:,1]

    found = False
    while not found:
        try:
            desl = np.random.uniform(-0.4,0.4,size=(3,))
            htm_ikm = htm_des @ Utils.trn(desl)
            q0 = robot.ikm(htm_target=htm_ikm, ignore_orientation=True)
            robot.add_ani_frame(k*dt,q=q0)
            print(q0)
            found = True
        except:
            found = False


    for j in range(1400):
        r, jac_r = robot.task_function(htm_des = htm_des)
        r[3:6,:] = np.sqrt(r[3:6,:])
        u = Utils.dp_inv(jac_r,0.001) @ (-r)
        robot.add_ani_frame(k*dt, q = robot.q + u*dt)
        k+= 1

sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images","controlecinematico")