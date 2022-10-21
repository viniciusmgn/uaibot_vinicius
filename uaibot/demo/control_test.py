from simulation import Simulation
import numpy as np
from simobjects.box import Box
from simobjects.frame import Frame
from utils import Utils
import robot as rb
from cvxopt import matrix, solvers
from graphics.meshmaterial import MeshMaterial, Texture
import sys
import time


def _control_demo_davinci(arm=0):
    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)

    robot_arm1, robot_arm2, robot_arm3, robot_arm4, chest = rb.Robot.create_davinci().list_of_objects
    arms = [robot_arm1, robot_arm2, robot_arm3, robot_arm4]
    robot = arms[arm]

    texture_wall = Texture(
        url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
        wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

    material_wall = MeshMaterial(texture_map=texture_wall, roughness=1, metalness=1)

    wall1 = Box(name="wall1", htm=Utils.trn([0.6 - 0.9, 0.5, 0.8]) @ Utils.roty(-np.pi/2), width=0.05, depth=0.2, height=1.4,
                mesh_material=material_wall)
    wall2 = Box(name="wall2", htm=Utils.trn([0.6 - 0.9, 1.1, 0.8]) @ Utils.roty(-np.pi/2), width=0.05, depth=0.2, height=1.4,
                mesh_material=material_wall)
    wall3 = Box(name="wall3", htm=Utils.trn([0. - 0.9, 0.75, 0.8]) @ Utils.roty(-np.pi/2), width=0.05, depth=0.6, height=0.2,
                mesh_material=material_wall)
    wall4 = Box(name="wall4", htm=Utils.trn([0.7 - 0.9, 0.75, 0.8]) @ Utils.roty(-np.pi/2), width=0.05, depth=0.6, height=0.5,
                mesh_material=material_wall)
    wall5 = Box(name="wall5", htm=Utils.trn([1.4 - 0.9, 0.8, 0.8]) @ Utils.roty(-np.pi/2), width=0.05, depth=0.8, height=0.2,
                mesh_material=material_wall)

    def evaluate_error(r, tol_pos=5e-3):
        error_pos = max(abs(r[0:3]))
        #error_pos = np.linalg.norm(r[0:3])
        error_ori = (180 / np.pi) * max(abs(np.arccos(1 - r[3:6])))
        ok1 = error_pos < tol_pos
        ok2 = error_ori < 5 if len(r.tolist()) > 3 else True

        return ok1 and ok2, error_pos, error_ori

    def dist_computation(q, old_struct):
        dist_wall_1 = robot.compute_dist(
            obj=wall1, q=q, old_dist_struct=old_struct[0])
        dist_wall_2 = robot.compute_dist(
            obj=wall2, q=q, old_dist_struct=old_struct[1])
        dist_wall_3 = robot.compute_dist(
            obj=wall3, q=q, old_dist_struct=old_struct[2])
        dist_wall_4 = robot.compute_dist(
            obj=wall4, q=q, old_dist_struct=old_struct[3])

        struct = [dist_wall_1, dist_wall_2, dist_wall_3, dist_wall_4]

        gradD_list = np.block(
            [[dist_wall_1.jac_dist_mat], [dist_wall_2.jac_dist_mat], [dist_wall_3.jac_dist_mat],
             [dist_wall_4.jac_dist_mat]])
        dist_vect = np.block(
            [[dist_wall_1.dist_vect], [dist_wall_2.dist_vect], [dist_wall_3.dist_vect], [dist_wall_4.dist_vect]])

        return dist_vect, gradD_list, struct

    # Target pose definition
    pose_tg = []
    #pose_tg.append(Utils.trn([0.2, 0.8, 0.7]) @ Utils.rotz(np.deg2rad(-200)) @ Utils.rotx(np.deg2rad(250)))
    pose_tg.append(Utils.trn([0.2, 0.8, 0.7]) @ Utils.rotx(np.deg2rad(200)) @ Utils.rotz(np.deg2rad(-130))) # rotz -100 ou -130
    # pose_tg.append(Utils.trn([-0.1, 0.8, 1]) @ Utils.rotx(np.deg2rad(200)) @ Utils.rotz(np.deg2rad(-130)))
    # pose_tg.append(Utils.trn([-0.5, 0.8, 0.7]) @ Utils.rotx(np.deg2rad(200)) @ Utils.rotz(np.deg2rad(-150)))

    # Create simulation
    sim = Simulation.create_sim_grid(
        [robot, wall1, wall2, wall3, wall4, wall5, chest])

    for k in range(len(pose_tg)):
        sim.add(Frame(name="pose_tg_" + str(k), htm=pose_tg[k]))
    
    # Parameters
    dt = 0.03
    alpha = 2
    beta = 0.005
    K = 2
    eta = 0.5
    sigma = -1
    dist_safe = 0.2
    # qdot_max = np.matrix([[2], [np.deg2rad(85)], [np.deg2rad(100)], [np.deg2rad(75)], [np.deg2rad(130)], [np.deg2rad(135)], [np.deg2rad(135)], [np.deg2rad(135)], [np.deg2rad(135)], [2]])
    # qdot_min = -qdot_max
    xi = 1
    h = 0.00002
    q = robot.q.astype(float)
    n = len(robot.links)
    qdot = np.zeros((n, 1)).astype(float)

    # Initializations
    struct = [None, None, None, None]

    i = 1
    imax = round((47.4/2)/dt)

    hist_dist = []
    hist_time = []
    hist_r = np.matrix(np.zeros(shape=(6, 0)))
    hist_q = np.matrix(np.zeros(shape=(n, 0)))
    hist_qddot = np.matrix(np.zeros(shape=(n, 0)))
    hist_qdot = np.matrix(np.zeros(shape=(n, 0)))
    hist_V = []

    error_qp = False
    iteration_end = False

    # Changing Initial configuration
    qaux = q.copy()
    qaux[4] = -np.pi/2
    qaux[5] = -np.deg2rad(120)
    qaux[6] = -np.deg2rad(110)
    qaux[7] = 0
    robot.add_ani_frame(0, qaux)
    q65_c = qaux[6] - qaux[5]
    q75_c = qaux[7] - qaux[5]
    print(q65_c, q75_c)

    # Main loop
    for k in range(len(pose_tg)):
        converged = False
        while not converged and not error_qp and not iteration_end:
            
            qdot_max = robot.joint_limit[:, 1] - q.reshape(-1, 1)
            qdot_min = robot.joint_limit[:, 0] - q.reshape(-1, 1)

            # This is just for showing progress when the simulation is run
            if i % 50 == 0 or i == imax - 1:
                sys.stdout.write('\r')
                sys.stdout.write(
                    "[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
                sys.stdout.flush()

            # Compute r and r_dot
            [r, jac_r] = robot.task_function(pose_tg[k], q=q)
            [r_next, jac_r_next] = robot.task_function(pose_tg[k], q=q + qdot * dt)

            r_dot = (r_next - r) / dt
            jac_r_dot = (jac_r_next - jac_r) / dt

            # Compute dist_vect, dist_vect_dot and the distance Jacobian

            dist_vect, gradD_list, struct = dist_computation(q, struct)
            dist_vect_next, gradD_list_next, struct = dist_computation(q + qdot * dt, struct)

            dist_vect_dot = (dist_vect_next - dist_vect) / dt
            jac_dist_dot = (gradD_list_next - gradD_list) / dt

            # Create the quadratic program parameters
            A = -gradD_list
            b = eta * (dist_vect - 0.02)
            H = 2 * (jac_r.T * jac_r) + 0.01 * np.identity(n)
            f = (2 * K * r.T @ jac_r).T
            q6_rest = np.array([0, 0, 0, 0, 0, 1, -1, 0, 0]).reshape(1, -1)
            q7_rest = np.array([0, 0, 0, 0, 0, 1, 0, -1, 0]).reshape(1, -1)
            A = np.block([[A], [np.identity(n)], [-np.identity(n)], [q6_rest], [q7_rest]])
            #A = np.block([[A], ])
            
            b = np.block([[b], [xi * qdot_max], [-xi * qdot_min], [-sigma * (q[6] - q[5] - q65_c)], [-sigma * (q[7] - q[5] - q75_c)]])

            # Solve the quadratic program
            try:
                qdot = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b))['x']
            except:
                qdot = np.matrix(np.zeros((n, 1)))
                error_qp = True

            qdot = np.array(qdot).reshape(n, 1)

            # First order explicit Euler simulation
            q += qdot * dt

            # Add animation to simulation
            robot.add_ani_frame(i * dt, q)

            # Store data for showing the graphs later
            dist_vect, _, _ = dist_computation(q, struct)

            hist_time.append(i * dt)
            hist_dist.append(np.amin(dist_vect))
            hist_r = np.block([hist_r, r])
            hist_q = np.block([hist_q, q])
            hist_qdot = np.block([hist_qdot, qdot])

            # Continue the loop, check if converged
            i += 1
            converged, error_pos, error_ori = evaluate_error(r, 5e-4)
            if converged:
                print(error_pos, error_ori)

            iteration_end  = i>imax
    print(converged, error_qp, iteration_end, i)
    # for i in robot.links:
    #     for j in i.col_objects:
    #         sim.add(j[0])
    # robot.update_col_object(0)
    # Run simulation
    sim.run()

    # Plot graphs
    Utils.plot(hist_time, hist_dist, "", "Time (s)", "True distance (m)", "dist")
    Utils.plot(hist_time, hist_q, "", "Time (s)", "Joint configuration (rad)", "q")
    Utils.plot(hist_time, hist_qdot, "", "Time (s)", "Joint speed (rad/s)", "qdot")
    #Utils.plot(hist_time, hist_qddot, "", "Time (s)", "Joint acceleration (rad/s²)", "u")
    #Utils.plot(hist_time, hist_V, "", "Time (s)", "Lyapunov function", "V")
    fig = Utils.plot(hist_time, hist_r, "", "Time (s)", "Task function",
                     ["posx", "posy", "posz", "orix", "oriy", "oriz"])

    return sim, (hist_time, hist_dist, hist_r, hist_q, hist_qdot)


def _control_demo_davinci2(arm=0):
    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)

    robot_arm1, robot_arm2, robot_arm3, robot_arm4, chest = rb.Robot.create_davinci().list_of_objects
    arms = [robot_arm1, robot_arm2, robot_arm3, robot_arm4]
    robot = arms[arm]

    # texture_wall = Texture(
    #     url='https://raw.githubusercontent.com/viniciusmgn/uaibot_vinicius/master/contents/Textures/rough_metal.jpg',
    #     wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

    material_wall = MeshMaterial(roughness=1, metalness=1)

    wall1 = Box(name="wall1", htm=Utils.trn([1.3, -1.5, 0.7]), width=0.05, depth=0.6, height=1.4,
                mesh_material=material_wall)
    wall2 = Box(name="wall2", htm=Utils.trn([1.3, 1.5, 0.7]), width=0.05, depth=0.6, height=1.4,
                mesh_material=material_wall)
    wall3 = Box(name="wall3", htm=Utils.trn([1.3, 1, 0.25]), width=0.05, depth=0.4, height=0.5,
                mesh_material=material_wall)
    wall4 = Box(name="wall4", htm=Utils.trn([1.3, 1, 1.15]), width=0.05, depth=0.4, height=0.5,
                mesh_material=material_wall)

    def evaluate_error(r):
        error_pos = max(abs(r[0:3]))
        error_ori = (180 / np.pi) * max(abs(np.arccos(1 - r[3:6])))
        ok1 = error_pos < 0.005
        ok2 = error_ori < 5 if len(r.tolist()) > 3 else True

        return ok1 and ok2, error_pos, error_ori

    def dist_computation(q, old_struct, h):
        dist_wall_1 = robot.compute_dist(
            obj=wall1, q=q, old_dist_struct=old_struct[0])
        dist_wall_2 = robot.compute_dist(
            obj=wall2, q=q, old_dist_struct=old_struct[1])
        dist_wall_3 = robot.compute_dist(
            obj=wall3, q=q, old_dist_struct=old_struct[2])
        dist_wall_4 = robot.compute_dist(
            obj=wall4, q=q, old_dist_struct=old_struct[3])

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
    sim = Simulation.create_sim_grid(
        [robot, wall1, wall2, wall3, wall4, chest])

    for k in range(len(pose_tg)):
        sim.add(Frame(name="pose_tg_" + str(k), htm=pose_tg[k]))

    # Parameters
    dt = 0.03
    alpha = 2
    beta = 0.005
    gamma = 0.5
    sigma = 0.5
    dist_safe = 0.2
    qdot_max = (np.pi / 180) * np.matrix([[85], [85], [100],
                                          [75], [130], [135], [135], [135], [135], [135]])
    qdot_min = -qdot_max
    xi = 0.5
    h = 0.05
    q = robot.q.astype(float)
    n = len(robot.links)
    qdot = np.zeros((n, 1)).astype(float)

    # Initializations
    struct = [None, None, None, None]

    i = 0
    imax = round(47.4/dt)

    hist_dist = []
    hist_time = []
    hist_r = np.matrix(np.zeros((6, 0)))
    hist_q = np.matrix(np.zeros((n, 0)))
    hist_qddot = np.matrix(np.zeros((n, 0)))
    hist_qdot = np.matrix(np.zeros((n, 0)))
    hist_V = []

    error_qp = False
    iteration_end = False

    # Main loop
    for k in range(len(pose_tg)):
        converged = False
        while not converged and not error_qp and not iteration_end:

            # This is just for showing progress when the simulation is run
            if i % 50 == 0 or i == imax - 1:
                sys.stdout.write('\r')
                sys.stdout.write(
                    "[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
                sys.stdout.flush()

            # Compute r and r_dot
            [r, jac_r] = robot.task_function(pose_tg[k], q=q)
            [r_next, jac_r_next] = robot.task_function(
                pose_tg[k], q=q + qdot * dt)

            r_dot = (r_next - r) / dt
            jac_r_dot = (jac_r_next - jac_r) / dt

            # Compute dist_vect, dist_vect_dot and the distance Jacobian

            dist_vect, jac_dist, struct = dist_computation(q, struct, h)
            dist_vect_next, jac_dist_next, struct = dist_computation(
                q + qdot * dt, struct, h)

            dist_vect_dot = (dist_vect_next - dist_vect) / dt
            jac_dist_dot = (jac_dist_next - jac_dist) / dt

            # Create the quadratic program parameters
            A = jac_dist
            b = -jac_dist_dot @ qdot - 2 * gamma * dist_vect_dot - \
                (gamma ** 2) * (dist_vect - dist_safe)
            H = 2 * (np.transpose(jac_r) * jac_r) + 2 * beta * np.identity(n)
            f = 2 * np.transpose(jac_r) * (
                jac_r_dot @ qdot + 2 * alpha * r_dot + (alpha ** 2) * r) + 2 * beta * sigma * qdot

            A = np.block([[A], [np.identity(n)]])
            A = np.block([[A], [-np.identity(n)]])

            b = np.block(
                [[b], [-xi * (qdot-qdot_min)], [xi * (qdot-qdot_max)]])

            # Solve the quadratic program
            try:
                qddot = solvers.qp(matrix(H), matrix(
                    f), matrix(-A), matrix(-b))['x']
            except:
                qddot = np.matrix(np.zeros((n, 1)))
                error_qp = True

            qddot = np.reshape(qddot, (n, 1))

            # First order explicit Euler simulation
            q += qdot * dt
            qdot += qddot * dt

            # Add animation to simulation
            robot.add_ani_frame(i * dt, q)

            # Store data for showing the graphs later
            dist_vect, _, _ = dist_computation(q, struct, 0.000001)

            hist_time.append(i * dt)
            hist_dist.append(np.amin(dist_vect))
            hist_r = np.block([hist_r, r])
            hist_q = np.block([hist_q, q])
            hist_qdot = np.block([hist_qdot, qdot])
            hist_qddot = np.block([hist_qddot, qddot])
            V = alpha * (np.linalg.norm(r_dot + alpha * r) ** 2) + \
                sigma * beta * (np.linalg.norm(qdot) ** 2)
            hist_V.append(V)

            # Continue the loop, check if converged
            i += 1
            converged, error_pos, error_ori = evaluate_error(r)

            #iteration_end  = i>500

    # Run simulation
    sim.run()

    # Plot graphs
    # Utils.plot(hist_time, hist_dist, "", "Time (s)", "True distance (m)", "dist")
    # Utils.plot(hist_time, hist_q, "", "Time (s)", "Joint configuration (rad)", "q")
    # Utils.plot(hist_time, hist_qdot, "", "Time (s)", "Joint speed (rad/s)", "qdot")
    # Utils.plot(hist_time, hist_qddot, "", "Time (s)", "Joint acceleration (rad/s²)", "u")
    # Utils.plot(hist_time, hist_V, "", "Time (s)", "Lyapunov function", "V")
    # fig = Utils.plot(hist_time, hist_r, "", "Time (s)", "Task function",
    #                  ["posx", "posy", "posz", "orix", "oriy", "oriz"])

    return sim
