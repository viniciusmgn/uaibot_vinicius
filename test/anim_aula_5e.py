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
robot = ub.Robot.create_kuka_kr5()

# Especifica a posição desejada para o efetuador
#como uma função de t
#Também fornece a derivada, que vai ser utilizada no feedforward
R = 0.2
y_c = 0.6
z_c = 0.6
omega_d = np.pi/2
s_d = lambda tt: np.matrix([R * np.cos(omega_d * tt), y_c, z_c + R * np.sin(omega_d * tt)]).reshape((3, 1))
s_d_dot = lambda tt: np.matrix([-R * omega_d * np.sin(omega_d * tt), 0, R * omega_d * np.cos(omega_d * tt)]).reshape((3, 1))


# Especifica a orientação desejada para o eixo z do efetuador
z_d = np.matrix([0,1,0]).reshape((3,1))

# Cria uma nuvem de pontos para visualizarmos a parte de posição da referência
pc = np.matrix(np.zeros((3,0)))
for i in range(200):
  pc = np.block([pc, s_d(2*np.pi*i/199)])

target_pos_pc = ub.PointCloud(points=pc, size=0.03, color="purple")

#Cria uma esfera para mostrar que o robô está rastreando a trajetória
ball_tr = ub.Ball(htm = np.identity(4), radius=0.02, color="cyan")

# Cria a simulação
sim = ub.Simulation([robot, target_pos_pc, ball_tr])

# Captura o número de juntas do robô
n = np.shape(robot.q)[0]

# Cria a função F:
def fun_F(r):
  K = 2
  return -K * r



# Cria uma matriz para o histórico de função de tarefa, da ação de controle
# e do tempo
hist_r = np.matrix(np.zeros((4, 0)))
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
  # Faz a extração de z_e e s_e
  z_e = fk[0:3, 2]
  s_e = fk[0:3, 3]

  # Monta o vetor de tarefa
  r = np.matrix(np.zeros((4, 1)))
  r[0:3] = s_e - s_d(t)
  r[3] = 1 - z_d.T * z_e

  # Monta a Jacobiana de tarefa
  Jr = np.matrix(np.zeros((4, n)))

  # Monta o termo de feedforward
  ff = np.block([[-s_d_dot(t)],[0]])

  Jr[0:3, :] = Jg[0:3, :]
  Jr[3, :] = z_d.T * ub.Utils.S(z_e) * Jg[3:6, :]


  # Calcula a ação de controle
  u = ub.Utils.dp_inv(Jr, 0.001) * (fun_F(r)-ff)

  # Guarda informações no histórico
  hist_r = np.block([hist_r, r])
  hist_u = np.block([hist_u, u])
  hist_t.append(t)

  # Manda a ação de controle para o robô
  set_configuration_speed(robot, u)

  # O tempo sempre vai passar no final do ciclo
  t += dt

  #Atualiza a posição da bola (apenas para visualização)
  ball_tr.add_ani_frame(time = t, htm = ub.Utils.trn(s_d(t)))

# Roda a simulação
sim.run()
# plota os gráficos



sim.set_parameters(load_screen_color="#191919", width=500, height=500, background_color="#191919")
sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula5","anim6")

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