import uaibot as ub
import numpy as np

dt = 0.01
t = 0
tmax = 12


def get_configuration(robot):
  return robot.q

def set_configuration_speed(robot, qdot_des):
  q_next = robot.q + qdot_des * dt
  robot.add_ani_frame(time=t + dt, q=q_next)

# As inicializações (ex: parâmetros do controlador) virão aqui
robot = ub.Robot.create_darwin_mini()
ball = ub.Ball(htm=ub.Utils.trn([0.4,0,0.6]), radius=0.15, color="yellow")

#Pega as variáveis referentes aos dois braços
right_arm = robot.list_of_objects[0]
left_arm = robot.list_of_objects[1]

#Coloca as configurações iniciais do cenário
right_arm.add_ani_frame(0,q=[ 0.50289982, -0.12686379, -0.0784082])
left_arm.add_ani_frame(0, q=[ 2.63894549, -0.12686379, -0.0784082])

#Parâmetros da trajetória
radius_mov = 0.05
omega_mov = 2*np.pi/3
s_bd = lambda tt: np.matrix(
    [0.35, radius_mov * np.cos(omega_mov * tt), 0.6 + radius_mov * np.sin(omega_mov * tt)]).reshape((3, 1))
dot_s_bd = lambda tt: np.matrix(
    [0, -omega_mov * radius_mov * np.sin(omega_mov * tt), omega_mov * radius_mov * np.cos(omega_mov * tt)]).reshape(
    (3, 1))

s_ED0 = left_arm.fkm()[0:3, 3] - right_arm.fkm()[0:3, 3]

#Cria a simulação
sim = ub.Simulation.create_sim_factory([robot, ball])


# Cria a função F:
def fun_F(r):
  K = 5
  return -K*r

# Cria uma matriz para o histórico de função de tarefa, da ação de controle
# e do tempo
hist_r = np.matrix(np.zeros((6, 0)))
hist_u = np.matrix(np.zeros((6, 0)))
hist_t = []

# Colocaremos aqui nosso "main" do controlador, que ficará em um laço
# durante um tempo tmax
for i in range(round(tmax / dt)):
  # Lógica de controle

  # Mede as configurações dos sensores
  q_E = get_configuration(left_arm)
  q_D = get_configuration(right_arm)

  #Calcula as Jacobianas geométricas
  Jg_E, fk_E = left_arm.jac_geo(q_E)
  Jg_D, fk_D = right_arm.jac_geo(q_D)

  s_E = fk_E[0:3, 3]
  s_D = fk_D[0:3, 3]

  #Monta as funções de tarefa e Jacobianas
  r = np.matrix(np.zeros((6, 1)))
  r[0:3, 0] = s_E - s_D - s_ED0
  r[3:6, 0] = (s_E + s_D) / 2 - s_bd(t)

  Jr = np.matrix(np.zeros((6, 6)))
  Jr[0:3, 0:3] = Jg_E[0:3, :]
  Jr[0:3, 3:6] = -Jg_D[0:3, :]
  Jr[3:6, 0:3] = Jg_E[0:3, :] / 2
  Jr[3:6, 3:6] = Jg_D[0:3, :] / 2

  ff = np.matrix(np.zeros((6, 1)))
  ff[3:6, 0] = -dot_s_bd(t)

  #Calcula a velocidade de configuração
  u = ub.Utils.dp_inv(Jr,0.001) * (fun_F(r) - ff)

  u_E = u[0:3, 0]
  u_D = u[3:6, 0]

  # Guarda informações no histórico
  hist_r = np.block([hist_r, r])
  hist_u = np.block([hist_u, u])
  hist_t.append(t)

  # Manda a ação de controle para o robô
  set_configuration_speed(left_arm, u_E)
  set_configuration_speed(right_arm, u_D)

  # O tempo sempre vai passar no final do ciclo
  t += dt

  #Atualiza a posição da bola (apenas para visualização)
  ball.add_ani_frame(t, htm=ub.Utils.trn((s_E + s_D) / 2))

# Roda a simulação
sim.run()
# plota os gráficos



sim.set_parameters(load_screen_color="#191919", width=500, height=500, background_color="#191919")
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula5","anim9")

#plota os gráficos
fig1 = ub.Utils.plot(hist_t, hist_r, "", "Tempo (s)", "Função de tarefa", "r")
#fig1.update_xaxes(title_text="t")
#fig1.update_yaxes(title_text="r")
fig1.update_xaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")
fig1.update_yaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")
fig1.update_layout({'plot_bgcolor': 'rgba(25,25,25,1)','paper_bgcolor': 'rgba(25,25,25,1)', 'font_color' : 'white', 'width': 500, 'height': 500})
fig1.show()

fig = ub.Utils.plot(hist_t, hist_u, "", "Tempo (s)", "Ação de controle (rad/s ou m/s)", "u")
#fig.update_xaxes(title_text="t")
#fig.update_yaxes(title_text="u")
fig.update_xaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")
fig.update_yaxes(showgrid=True, gridwidth=0.3, gridcolor="rgba(25,25,25,1)")


fig.update_layout({'plot_bgcolor': 'rgba(25,25,25,1)','paper_bgcolor': 'rgba(25,25,25,1)', 'font_color' : 'white', 'width': 500, 'height': 500})
fig.show()