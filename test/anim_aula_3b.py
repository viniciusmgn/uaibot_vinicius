from uaibot import *
import numpy as np

width=800
height=600

robot = Robot.create_kuka_kr5(opacity=0.7, color="gray", eef_frame_visible=False)
#robot = Robot.create_abb_crb(opacity=0.7, color="gray", eef_frame_visible=False)
#robot = Robot.create_epson_t6(opacity=0.7, color="gray", eef_frame_visible=False)

#robot.add_ani_frame(0,q=[np.pi/4,-np.pi/3,0.1])
robot.add_ani_frame(0,q=[np.pi/4,-np.pi/4,np.pi/9,-np.pi/9,0,np.pi/2])

sim = Simulation([robot], load_screen_color="#191919", background_color="#222224", camera_type="orthographic",
                 show_world_frame=False, width=width, height=height)

# Create all objects that will be used during the simulation

far = Utils.trn([1000, 1000, 1000])


def clone_link_as_rigid_body(name, link):
    list_model_3d = []
    for model3d in link.list_model_3d:
        list_model_3d.append(
            Model3D(model3d.url, model3d.scale, model3d.htm, MeshMaterial(color="gold", opacity=0.8)))

    return RigidObject(list_model_3d, name=name, htm=far)


links_alternative = []
for i in range(len(robot.links)):
    links_alternative.append(clone_link_as_rigid_body("rigid_object_link_" + str(i), robot.links[i]))
    sim.add(links_alternative[-1])

htm_dh = robot.fkm(axis="dh")

htm = [robot.htm_base_0]
for i in range(len(robot.links)):
    htm.append(htm_dh[i])

x_axis = []
y_axis = []
z_axis = []

vector_length = 0.12

lines_purple = Cylinder(name="linePurple", htm=far, radius=0.002, height=2, color="purple")
lines_orange = Cylinder(name="lineOrange", htm=far, radius=0.002, height=2, color="#FFA500")
points_A = Ball(name="pointA", htm=far, radius=0.015, color="cyan")


line_between_points = []
start_lines = []
end_lines = []
dist_between_points = 0.01

for i in range(len(robot.links)):
    p_A = (htm[i] @ Utils.trn([0, 0, robot.links[i].d]))[0:3, 3]
    p_B = htm[i + 1][0:3, 3]

    n_points = floor(np.linalg.norm(p_A - p_B) / dist_between_points)
    start_lines.append(len(line_between_points))

    for j in range(n_points):
        line_between_points.append((1 - j / n_points) * p_A + (j / n_points) * p_B)

    end_lines.append(len(line_between_points))

n = len(robot.links)
line_between_points = np.array(line_between_points).transpose()
if np.shape(line_between_points)[0]==0:
    line_between_points = np.zeros((3,0))

line_between_points_pc = PointCloud(name="lineBetweenPoints", points=line_between_points, size=2, color="white")
line_between_points_pc.add_ani_frame(0, 0, 0)

sim.add(points_A)


for i in range(len(robot.links) + 1):
    x_axis.append(Cylinder(name="xAxis" + str(i), color="red", htm=far, radius=0.003, height=vector_length))
    y_axis.append(Cylinder(name="yAxis" + str(i), color="limegreen", htm=far, radius=0.003, height=vector_length))
    z_axis.append(Cylinder(name="zAxis" + str(i), color="blue", htm=far, radius=0.003, height=vector_length))

    sim.add(x_axis[-1])
    sim.add(y_axis[-1])
    sim.add(z_axis[-1])

style = "top:" + str(0.8 * height) + "px;right:" + str(0) + "px;width:" + str(
    width) + "px;position:absolute;text-align:center;color:white;background-color:#222224;font-smooth:always;font-family:arial"

explanation = HTMLDiv(name="expl", html_text="", style=style)

style = "top:" + str(0.05 * height) + "px;right:20px;width:300px;position:absolute;text-align:center;color:white;background-color:#222224;font-smooth:always;font-family:arial"

table_html = HTMLDiv(name="table", html_text="", style=style)

sim.add([explanation,table_html])


def txt_link(i):
    return "<b>link<sub>" + str(i) + "</sub></b>"


def txt_frame(i):
    return "<b>F<sub>DH" + str(i) + "</sub></b>"


def txt_joint(i):
    return "<b>junta<sub>" + str(i) + "</sub></b>"


def txt_line_purple(i):
    return "<span style='color:#FF7AFF'><b>linha<sub>" + str(i) + "</sub></b></span>"


def txt_line_orange(i):
    return "<span style='color:#FFA500'><b>linha<sub>" + str(i) + "</sub></b></span>"


def txt_pointA(i):
    return "<span style='color:cyan'><b>ponto<sub>" + str(i) + "," + str(i + 1) + "</sub></b></span>"


def txt_pointB(i):
    return "<span style='color:#7bb572'><b>ponto<sub>" + str(i) + "," + str(i - 1) + "</sub></b></span>"


def txt_axis_x(i):
    return "<span style='color:red'><b>x<sub>DH" + str(i) + "</sub></b></span>"


def txt_axis_y(i):
    return "<span style='color:limegreen'><b>y<sub>DH" + str(i) + "</sub></b></span>"


def txt_axis_z(i):
    return "<span style='color:blue'><b>z<sub>DH" + str(i) + "</sub></b></span>"

def txt_theta(i):
    return "&#952<sub>" + str(i) + "</sub>"

def txt_d(i):
    return "d<sub>" + str(i) + "</sub>"

def txt_alpha(i):
    return "&#945<sub>" + str(i) + "</sub>"

def txt_a(i):
    return "a<sub>" + str(i) + "</sub>"

def create_table(table, i_max, j_max):
    strtable = "<table style=\'color:white; width:300px; text-align:right; border: 1px solid white\'>"
    strtable += "<tr> <th>i</th> <th>&#952<sub>i</sub></th> <th>d<sub>i</dub></th> <th>&#945<sub>i</sub></th> <th>a<sub>i</dub></th> </tr>"
    for i in range(len(table)):
        strtable += "<tr>"
        strtable += "<td width=\'8%\'>" + str(i+1) + "</td>"
        strtable += "<td width=\'23%\'>" + (table[i][0] if i+1<=i_max or (i <= i_max and j_max > 0) else "") + "</td>"
        strtable += "<td width=\'23%\'>" + (table[i][1] if i+1<=i_max or (i <= i_max and j_max > 1) else "") + "</td>"
        strtable += "<td width=\'23%\'>" + (table[i][2] if i+1<=i_max or (i <= i_max and j_max > 2) else "") + "</td>"
        strtable += "<td width=\'23%\'>" + (table[i][3] if i+1<=i_max or (i <= i_max and j_max > 3) else "") + "</td>"
        strtable += "</tr>"

    strtable += "</table>"
    return strtable

table_info = []
for i in range(len(robot.links)):
    table_info.append([])
    if robot.links[i].joint_type == 0:
        table_info[i].append("<b style=\'color:gold\'>"+str(round((180/np.pi)*robot.q[i,0]))+"<sup>o</sup></b>")
    else:
        table_info[i].append(str(round((180/np.pi)*robot.links[i].theta))+"<sup>o</sup>")

    if robot.links[i].joint_type == 1:
        table_info[i].append("<b style=\'color:gold\'>"+str(round(1000*robot.q[i,0])/1000)+"m</b>")
    else:
        table_info[i].append( str(round(1000*robot.links[i].d)/1000)+"m" )

    table_info[i].append( str(round((180/np.pi)*robot.links[i].alpha))+"<sup>o</sup>")
    table_info[i].append( str(round(1000*robot.links[i].a)/1000)+"m" )





# Make the animation
k = 0
dt = 0.01
deltak = 200

# Show all DH frames for a while

explanation.add_ani_frame(k * dt, "Considere todos os eixos DH do rob&ocirc. Faremos a transforma&ccedil&atildeo de um para o outro.")

for i in range(len(robot.links) + 1):
    x_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    y_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    z_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, vector_length / 2]))

k+= 2*deltak

for i in range(len(robot.links) + 1):
    x_axis[i].add_ani_frame(k * dt, htm=far @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    y_axis[i].add_ani_frame(k * dt, htm=far @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    z_axis[i].add_ani_frame(k * dt, htm=far @ Utils.trn([0, 0, vector_length / 2]))

for i in range(len(robot.links)):

    table_html.add_ani_frame(k*dt, create_table(table_info,i,0))
    explanation.add_ani_frame(k * dt,
                              "Faremos a transforma&ccedil&atildeo de " + txt_frame(i) + " para " + txt_frame(i+1)+".")

    x_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    y_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    z_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, vector_length / 2]))

    x_axis[i+1].add_ani_frame(k * dt, htm=htm[i+1] @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    y_axis[i+1].add_ani_frame(k * dt, htm=htm[i+1] @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
    z_axis[i+1].add_ani_frame(k * dt, htm=htm[i+1] @ Utils.trn([0, 0, vector_length / 2]))

    k += 2 * deltak

    #Início theta
    explanation.add_ani_frame(k * dt,
                              "Come&ccedilamos com uma rota&ccedil&atildeo no eixo z atual de " + txt_theta(
                                  i+1) + ".  O objetivo &eacute alinhar os dois eixos x.")
    k += 2 * deltak

    if robot.links[i].joint_type == 0:
        q_c = round((180/np.pi) * robot.q[i,0])
        explanation.add_ani_frame(k * dt,
                                  "Como a " + txt_joint(
                                      i+1) + " &eacute rotativa, esse valor &eacute <span style=\'color:gold\'>vari&aacutevel</span>.<br> No caso, &eacute <b><span style=\'color:gold\'>"
                                  + str(q_c) + "</span></b> graus.")
    else:
        q_c = round( (180/np.pi) * robot.links[i].theta )
        explanation.add_ani_frame(k * dt,
                                  "No caso, &eacute <b>"+ str(q_c) + "</b> graus.")

    table_html.add_ani_frame(k * dt, create_table(table_info, i, 1))
    for j in range(3 * deltak):
        htm[i] = htm[i] @ Utils.rotz((np.pi/180) * q_c/(3 * deltak-1))
        k+= 1
        x_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        y_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        z_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, vector_length / 2]))


    explanation.add_ani_frame(k * dt,"Note que os dois eixos x est&atildeo alinhados!")

    k += 2 * deltak

    #Início d
    explanation.add_ani_frame(k * dt,
                              "Come&ccedilamos com uma transla&ccedil&atildeo no eixo z atual de " + txt_d(
                                  i+1) + ".  O objetivo &eacute fazer com que o centro do referencial atual v&aacute para "+txt_pointA(i+1)+".")

    if robot.links[i].joint_type == 0:
        points_A.add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, robot.links[i].d]))
    else:
        points_A.add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, robot.q[i,0]]))

    k += 2 * deltak

    if robot.links[i].joint_type == 1:
        q_c = round(100 * robot.q[i,0])/100
        explanation.add_ani_frame(k * dt,
                                  "Como a " + txt_joint(
                                      i+1) + " &eacute linear, esse valor &eacute <span style=\'color:gold\'>vari&aacutevel</span>.<br> No caso, &eacute <b><span style=\'color:gold\'>"
                                  + str(q_c) + "</span></b> metros.")
    else:
        q_c = round( 1000 * robot.links[i].d )/1000
        explanation.add_ani_frame(k * dt,
                                  "No caso, &eacute <b>"+ str(q_c) + "</b> metros.")

    table_html.add_ani_frame(k * dt, create_table(table_info, i, 2))
    for j in range(3 * deltak):
        htm[i] = htm[i] @ Utils.trn([0,0, q_c/(3 * deltak-1)])
        k+= 1
        x_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        y_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        z_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, vector_length / 2]))

    explanation.add_ani_frame(k * dt,"Note que o centro do referencial coincide com "+txt_pointA(i+1)+".")

    k += 2 * deltak
    points_A.add_ani_frame(k * dt, htm=far)

    #Início alpha
    explanation.add_ani_frame(k * dt,
                              "Come&ccedilamos com uma rota&ccedil&atildeo no eixo x atual de " + txt_alpha(
                                  i+1) + ".  O objetivo &eacute fazer com que os eixos z dos dois referenciais fiquem alinhados.")
    k += 2 * deltak

    q_c = round((180 / np.pi) * robot.links[i].alpha)
    explanation.add_ani_frame(k * dt,
                                  "No caso, &eacute <b>"+ str(q_c) + "</b> graus.")
    table_html.add_ani_frame(k * dt, create_table(table_info, i, 3))

    for j in range(3 * deltak):
        htm[i] = htm[i] @ Utils.rotx((np.pi/180) * q_c/(3 * deltak-1))
        k+= 1
        x_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        y_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        z_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, vector_length / 2]))

    explanation.add_ani_frame(k * dt,"Note que todos os eixos est&atildeo alinhados agora!")

    k += 2 * deltak

    #Início a
    explanation.add_ani_frame(k * dt,
                              "Come&ccedilamos com uma transla&ccedil&atildeo no eixo x atual de " + txt_a(
                                  i+1) + ".  O objetivo &eacute fazer com que os dois referenciais finalmente coincidam.")
    k += 2 * deltak

    q_c = round(1000 * robot.links[i].a)/1000
    explanation.add_ani_frame(k * dt,
                                  "No caso, &eacute <b>"+ str(q_c) + "</b> metros.")

    table_html.add_ani_frame(k * dt, create_table(table_info, i, 4))

    for j in range(3 * deltak):
        htm[i] = htm[i] @ Utils.trn([q_c/(3 * deltak-1),0,0])
        k+= 1
        x_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        y_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        z_axis[i].add_ani_frame(k * dt, htm=htm[i] @ Utils.trn([0, 0, vector_length / 2]))

    explanation.add_ani_frame(k * dt,"Note que todos os dois referenciais s&atildeo iguais agora!")

    k += 2 * deltak

    for j in range(len(robot.links) + 1):
        x_axis[j].add_ani_frame(k * dt, htm=far @ Utils.roty(3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        y_axis[j].add_ani_frame(k * dt, htm=far @ Utils.rotx(-3.14 / 2) @ Utils.trn([0, 0, vector_length / 2]))
        z_axis[j].add_ani_frame(k * dt, htm=far @ Utils.trn([0, 0, vector_length / 2]))

sim.save("D:\\PycharmProjects\\aulas_manipuladores\\presentation\\images\\aula3","anim5")