from uaibot import *
from scipy.linalg import null_space
import numpy as np



#Animação da junta revolta


sim = Simulation([], load_screen_color="#191919", background_color="#191919", width=500, height=500,
                 camera_type="orthographic")

robot = Robot.create_kuka_lbr_iiwa()

light1 = PointLight(name="light1", color="white", intensity=4, htm=Utils.trn([-1, -1, 1.5]))
light2 = PointLight(name="light2", color="white", intensity=4, htm=Utils.trn([-1, 1, 1.5]))
light3 = PointLight(name="light3", color="white", intensity=4, htm=Utils.trn([1, -1, 1.5]))
light4 = PointLight(name="light4", color="white", intensity=4, htm=Utils.trn([1, 1, 1.5]))

robot.add_ani_frame(0, q = [0,np.pi/4,np.pi/2,np.pi/4,0,0,np.pi/2,])

sim.add([robot])

dt=0.01

for i in range(2000):
    jg,_ = robot.jac_geo()
    qdot_null = null_space(jg)
    g = np.shape(qdot_null)[1]

    qprox = robot.q+np.reshape(qdot_null @ np.ones((g,1)),(7,1))*dt
    robot.add_ani_frame(i*dt, q = qprox)

sim.set_parameters(load_screen_color="#191919", width=500, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula4","anim6")

