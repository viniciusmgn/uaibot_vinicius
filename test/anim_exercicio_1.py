import uaibot as ub
import numpy as np

#Cria o cenário
robot = ub.Robot.create_abb_crb(ub.Utils.trn([0,0,0.2]))

texture_table = ub.Texture(
    url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
    wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[1, 1])

material_table = ub.MeshMaterial(texture_map=texture_table, roughness=1, metalness=1, opacity=1)
table1 = ub.Box(name="table1", htm = ub.Utils.trn([0.8,0,0.15]), width=0.5, depth=0.5, height=0.4, mesh_material=material_table)
table2 = ub.Box(name="table2", htm = ub.Utils.trn([0,-0.8,0.15]), width=0.5, depth=0.5, height=0.4, mesh_material=material_table)
table3 = ub.Box(name="table3", htm = ub.Utils.trn([0,0,0.1]), width=0.3, depth=0.3, height=0.2, mesh_material=material_table)
obstacle = ub.Box(name="obstacle", htm = ub.Utils.trn([0.8,-0.8,0.5]), width=0.6, depth=0.6, height=1, mesh_material=material_table)

material_cube = ub.MeshMaterial(roughness=1, metalness=1, opacity=1, color="purple")
cube = ub.Box(name="cube", htm = ub.Utils.trn([0.8,0,0.4]), width=0.1, depth=0.1, height=0.1, mesh_material=material_cube)

sim = ub.Simulation.create_sim_factory([robot, table1, table2, table3, obstacle, cube])
sim.set_parameters(load_screen_color="#191919", width=500, height=500)
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\exercicios3","anim1")

#Resolve o problema

#Encontra a primeira pose alvo
target = table1.htm @ ub.Utils.trn([0,0,0.3]) @ ub.Utils.rotx(np.pi)

#Encontra, por cinemática inversa, uma configuração próxima da atual
#para pegar o bloco por cima
q_target = robot.ikm(htm_target = target, q0 = robot.q)

#Faz o movimento linear no espaço de juntas até pegar o bloco
dt = 0.01
tmax=5
t=0
for i in range(round(tmax/dt)):
  t += dt
  q_c = (1-t/tmax)*robot.q0 + (t/tmax)*q_target
  robot.add_ani_frame(t,q=q_c)

#Pega o bloco no efetuador
robot.attach_object(cube)

#Escolhe uma pose alvo para escapar do obstáculo
target = ub.Utils.trn([0,0,1]) @ ub.Utils.rotx(np.pi/2)

#Encontra, por cinemática inversa, uma configuração próxima da atual
#para chegar lá
q_target = robot.ikm(htm_target = target, q0 = robot.q)


#Faz o movimento linear no espaço de juntas até chegar no alvo atual
dt = 0.01
tmax=5
q0_new = robot.q
t_new = t
for i in range(round(tmax/dt)):
  t += dt
  q_c = (1-(t-t_new)/tmax)*q0_new + ((t-t_new)/tmax)*q_target
  robot.add_ani_frame(t,q=q_c)

#Encontra, por cinemática inversa, uma configuração próxima da atual
#para entregar o bloco
target = table2.htm @ ub.Utils.trn([0,0,0.3]) @ ub.Utils.rotx(np.pi)

#Encontra, por cinemática inversa, uma configuração próxima da atual
#para entregar o bloco, também por cima

q_target = robot.ikm(htm_target = target, q0 = robot.q)

#Faz o movimento linear no espaço de juntas até entregar o bloco
dt = 0.01
tmax=5
q0_new = robot.q
t_new = t
for i in range(round(tmax/dt)):
  t += dt
  q_c = (1-(t-t_new)/tmax)*q0_new + ((t-t_new)/tmax)*q_target
  robot.add_ani_frame(t,q=q_c)



sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\exercicios3","anim2")
