from simulation import *
import numpy as np
from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *
from simobjects.pointlight import *
from simobjects.pointcloud import *
from simobjects.frame import *
from utils import *
import robot as rb
from cvxopt import matrix, solvers
from graphics.meshmaterial import *
import sys
import time


def _control_demo_3():
    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)

    robot = rb.Robot.create_kuka_lbr_iiwa()

    texture_wall = Texture(
        url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
        wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

    material_wall = MeshMaterial(texture_map=texture_wall, roughness=1, metalness=1)

    wall1 = Box(name="wall1", htm=Utils.trn([0.3, -0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
                mesh_material=material_wall)
    wall2 = Box(name="wall2", htm=Utils.trn([0.3, 0.5, 0.7]), width=0.05, depth=0.6, height=1.4,
                mesh_material=material_wall)
    wall3 = Box(name="wall3", htm=Utils.trn([0.3, 0, 0.25]), width=0.05, depth=0.4, height=0.5,
                mesh_material=material_wall)
    wall4 = Box(name="wall4", htm=Utils.trn([0.3, 0, 1.15]), width=0.05, depth=0.4, height=0.5,
                mesh_material=material_wall)

    def evaluate_error(r):
        error_pos = max(abs(r[0:3]))
        error_ori = (180 / np.pi) * max(abs(np.arccos(1 - r[3:6])))
        ok1 = error_pos < 0.005
        ok2 = error_ori < 5 if len(r.tolist()) > 3 else True

        return ok1 and ok2, error_pos, error_ori

    def dist_computation(q, old_struct, h):
        dist_wall_1 = robot.compute_dist(obj=wall1, q=q, h=h, g=h, old_dist_struct=old_struct[0])
        dist_wall_2 = robot.compute_dist(obj=wall2, q=q, h=h, g=h, old_dist_struct=old_struct[1])
        dist_wall_3 = robot.compute_dist(obj=wall3, q=q, h=h, g=h, old_dist_struct=old_struct[2])
        dist_wall_4 = robot.compute_dist(obj=wall4, q=q, h=h, g=h, old_dist_struct=old_struct[3])

        struct = [dist_wall_1, dist_wall_2, dist_wall_3, dist_wall_4]

        jac_dist = np.block(
            [[dist_wall_1.jac_dist_mat], [dist_wall_2.jac_dist_mat], [dist_wall_3.jac_dist_mat],
             [dist_wall_4.jac_dist_mat]])
        dist_vect = np.block(
            [[dist_wall_1.dist_vect], [dist_wall_2.dist_vect], [dist_wall_3.dist_vect], [dist_wall_4.dist_vect]])

        return dist_vect, jac_dist, struct

    # Target pose definition
    pose_tg = []
    pose_tg.append(Utils.trn([0.5, -0.3, 0.7]) @ Utils.rotx(3.14 / 2))
    pose_tg.append(Utils.trn([0.5, 0, 0.7]) @ Utils.roty(3.14 / 2))
    pose_tg.append(robot.fkm(axis="eef"))

    # Create simulation
    sim = Simulation.create_sim_factory([robot, wall1, wall2, wall3, wall4])

    for k in range(len(pose_tg)):
        sim.add(Frame(name="pose_tg_" + str(k), htm=pose_tg[k]))

    # Parameters
    dt = 0.03
    alpha = 2
    beta = 0.005
    gamma = 0.5
    sigma = 0.5
    dist_safe = 0.2
    qdot_max = (np.pi / 180) * np.matrix([[85], [85], [100], [75], [130], [135], [135]])
    qdot_min = -qdot_max
    xi = 0.5
    h = 0.05
    q = robot.q
    qdot = np.zeros((7, 1))

    # Initializations
    struct = [None, None, None, None]

    i = 0
    imax = round(47.4/dt)

    hist_dist = []
    hist_time = []
    hist_r = np.matrix(np.zeros((6,0)))
    hist_q = np.matrix(np.zeros((7, 0)))
    hist_qddot = np.matrix(np.zeros((7,0)))
    hist_qdot = np.matrix(np.zeros((7,0)))
    hist_V = []

    error_qp = False
    iteration_end = False

    #Main loop
    for k in range(len(pose_tg)):
        converged = False
        while not converged and not error_qp and not iteration_end:

            #This is just for showing progress when the simulation is run
            if i % 50 == 0 or i == imax - 1:
                sys.stdout.write('\r')
                sys.stdout.write("[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
                sys.stdout.flush()

            #Compute r and r_dot
            [r, jac_r] = robot.task_function(pose_tg[k], q=q)
            [r_next, jac_r_next] = robot.task_function(pose_tg[k], q=q + qdot * dt)

            r_dot = (r_next - r) / dt
            jac_r_dot = (jac_r_next - jac_r) / dt

            #Compute dist_vect, dist_vect_dot and the distance Jacobian

            dist_vect, jac_dist, struct = dist_computation(q, struct, h)
            dist_vect_next, jac_dist_next, struct = dist_computation(q + qdot * dt, struct, h)

            dist_vect_dot = (dist_vect_next - dist_vect) / dt
            jac_dist_dot = (jac_dist_next - jac_dist) / dt

            #Create the quadratic program parameters
            A = jac_dist
            b = -jac_dist_dot @ qdot -2 * gamma * dist_vect_dot - (gamma ** 2) * (dist_vect - dist_safe)
            H = 2 * (np.transpose(jac_r) * jac_r) + 2 * beta * np.identity(7)
            f = 2 * np.transpose(jac_r) * (
                    jac_r_dot @ qdot + 2 * alpha * r_dot + (alpha ** 2) * r) + 2 * beta * sigma * qdot

            A = np.block([[A], [np.identity(7)]])
            A = np.block([[A], [-np.identity(7)]])


            b = np.block([[b], [-xi * (qdot-qdot_min)], [xi * (qdot-qdot_max)]  ])


            #Solve the quadratic program
            try:
                qddot = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b))['x']
            except:
                qddot = np.matrix(np.zeros((7, 1)))
                error_qp = True

            qddot = np.reshape(qddot, (7, 1))

            #First order explicit Euler simulation
            q += qdot * dt
            qdot += qddot * dt

            #Add animation to simulation
            robot.add_ani_frame(i * dt, q)

            #Store data for showing the graphs later
            dist_vect, _, _ = dist_computation(q, struct, 0.000001)

            hist_time.append(i * dt)
            hist_dist.append(np.amin(dist_vect))
            hist_r = np.block([hist_r,r])
            hist_q = np.block([hist_q,q])
            hist_qdot = np.block([hist_qdot,qdot])
            hist_qddot = np.block([hist_qddot,qddot])
            V=alpha * (np.linalg.norm(r_dot + alpha * r) ** 2) + sigma * beta * (np.linalg.norm(qdot) ** 2)
            hist_V.append(V)

            #Continue the loop, check if converged
            i += 1
            converged, error_pos, error_ori = evaluate_error(r)

            #iteration_end  = i>500

    # Run simulation
    sim.run()

    # Plot graphs
    Utils.plot(hist_time, hist_dist, "", "Time (s)", "True distance (m)", "dist")
    Utils.plot(hist_time, hist_q, "", "Time (s)", "Joint configuration (rad)", "q")
    Utils.plot(hist_time, hist_qdot, "", "Time (s)", "Joint speed (rad/s)", "qdot")
    Utils.plot(hist_time, hist_qddot, "", "Time (s)", "Joint acceleration (rad/s²)", "u")
    Utils.plot(hist_time, hist_V, "", "Time (s)", "Lyapunov function", "V")
    fig = Utils.plot(hist_time, hist_r, "", "Time (s)", "Task function",
                     ["posx", "posy", "posz", "orix", "oriy", "oriz"])

    return sim