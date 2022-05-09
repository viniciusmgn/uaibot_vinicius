import uaibot as ub
import numpy as np


dt = 0.01
t = 0
tmax = 10

def get_configuration(robot):
  return robot.q

def set_configuration_speed(robot, qdot_des):
  q_next = robot.q + qdot_des*dt
  robot.add_ani_frame(time = t+dt, q = q_next)

#Inicializações
robot = ub.Robot.create_staubli_tx60()
sim = ub.Simulation([robot])
htm_des = ub.Utils.trn([0,-0.2,-0.5]) * robot.fkm()
q_ik = robot.ikm(htm_des)
K = 1

#Vamos colocar uma configuração inicial aleatória para o
#robô, para testar a capacidade dele de convergir para
#diferentes configurações
q0 = robot.q0+np.random.uniform(-0.3,0.3,size=(6,1))

for i in range(round(tmax/dt)):
  #Lógica de controle

  q = get_configuration(robot)
  u = -K*(q-q_ik)
  set_configuration_speed(robot, u)

  #O tempo sempre vai passar no final do ciclo
  t+=dt

sim.set_parameters(load_screen_color="#191919", width=500, height=500, background_color="#191919")
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula5","anim1")