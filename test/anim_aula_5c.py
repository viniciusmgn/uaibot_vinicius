import uaibot as ub
import numpy as np

dt = 0.01
t = 0
tmax = 6


def get_configuration(robot):
  return robot.q


def set_configuration_speed(robot, qdot_des):
  q_next = robot.q + qdot_des * dt
  robot.add_ani_frame(time=t + dt, q=q_next)


# As inicializações (ex: parâmetros do controlador) virão aqui
robot = ub.Robot.create_kuka_kr5()

# Especifica a pose desejada para o efetuador
htm_d = ub.Utils.trn([-0.3, 0.2, -0.3]) * robot.fkm()
# Faz a extração dos elementos x_d, y_d, z_d e s_d
x_d = htm_d[0:3, 0]
y_d = htm_d[0:3, 1]
z_d = htm_d[0:3, 2]
s_d = htm_d[0:3, 3]

# Cria um referencial no cenário para que seja possível ver se ele alcancou a
# pose
frame_d = ub.Frame(htm=htm_d)

# Cria a simulação
sim = ub.Simulation([robot, frame_d])

# Captura o número de juntas do robô
n = np.shape(robot.q)[0]


# Cria a função F:
def fun_F(r):
  K = 2
  return -K * r


# Cria uma matriz para o histórico de função de tarefa, da ação de controle
# e do tempo
hist_r = np.matrix(np.zeros((6, 0)))
hist_u = np.matrix(np.zeros((n, 0)))
hist_t = []

# Colocaremos aqui nosso "main" do controlador, que ficará em um laço
# durante um tempo tmax
for i in range(round(tmax / dt)):
  # Lógica de controle

  # Mede a configuração dos sensores
  q = get_configuration(robot)

  # Calcula a cinemática direta e Jacobiana para o efetuador nessa
  # configuração
  Jg, fk = robot.jac_geo(q)
  # Faz a extração de x_e, y_e, z_e e s_e
  x_e = fk[0:3, 0]
  y_e = fk[0:3, 1]
  z_e = fk[0:3, 2]
  s_e = fk[0:3, 3]

  # Monta o vetor de tarefa
  r = np.matrix(np.zeros((6, 1)))
  r[0:3] = s_e - s_d
  r[3] = 1 - x_d.T * x_e
  r[4] = 1 - y_d.T * y_e
  r[5] = 1 - z_d.T * z_e

  # Monta a Jacobiana de tarefa
  Jr = np.matrix(np.zeros((6, n)))

  Jr[0:3, :] = Jg[0:3, :]
  Jr[3, :] = x_d.T * ub.Utils.S(x_e) * Jg[3:6, :]
  Jr[4, :] = y_d.T * ub.Utils.S(y_e) * Jg[3:6, :]
  Jr[5, :] = z_d.T * ub.Utils.S(z_e) * Jg[3:6, :]

  # Calcula a ação de controle
  u = ub.Utils.dp_inv(Jr, 0.001) * fun_F(r)

  # Guarda informações no histórico
  hist_r = np.block([hist_r, r])
  hist_u = np.block([hist_u, u])
  hist_t.append(t)

  # Manda a ação de controle para o robô
  set_configuration_speed(robot, u)

  # O tempo sempre vai passar no final do ciclo
  t += dt

# Roda a simulação
sim.run()
# plota os gráficos



sim.set_parameters(load_screen_color="#191919", width=500, height=500, background_color="#191919")
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula5","anim2")

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