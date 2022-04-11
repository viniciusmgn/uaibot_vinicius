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


def _control_demo_3_old():
    solvers.options['show_progress'] = False

    # Create obstacle object and robots

    # Create simulation and add objects to the scene
    robot = rb.Robot.create_kuka_kr5(np.identity(4), "robot")
    mesh_obstacle = MeshMaterial(metalness=0.7, clearcoat=1, roughness=0.5, normal_scale=[0.5, 0.5], color="green")
    obstacle = Box(htm=Utils.trn([0.09, 0.5, 0.6]), name='obstacle', width=0.14, height=0.14, depth=0.14,
                   mesh_material=mesh_obstacle)


    desired_eef_frame = Frame(axis_color=["darkred", "green", "darkblue"], size=0.33)

    sim = Simulation.create_sim_factory([robot, obstacle, desired_eef_frame])

    # Starting pose for the end-effector
    htm0 = Utils.trn([-0.2, 0, -0.2]) @ robot.fkm()

    # Use inverse kinematics to set the robot to the starting pose
    q = robot.ikm(htm0, q0=robot.q)
    robot.add_ani_frame(0, q)

    # Set desired pose
    htm_des = Utils.trn([0.5, 0, 0]) @ robot.fkm()
    desired_eef_frame.set_ani_frame(htm_des)

    # Simulation parameters
    time_max = 16  # 20
    dt = 0.01
    alpha = 1.5  # 3
    gamma = 0.1  # 0.5 0.3 0.1
    beta = 0.005
    h = 0.05  # 0.05
    qddotmax = 5

    # Initializations
    hist_dist = []
    hist_time = []
    hist_r = []
    hist_qddot = []
    hist_qdot = []
    hist_q = []
    hp = []
    dist_struct = None
    imax = round(time_max / dt)

    qdot = np.zeros((6,1))
    end_loop = False
    i = 0

    # Main loop
    while (not end_loop) and (i < imax):

        if i % 50 == 0 or i == imax - 1:
            sys.stdout.write('\r')
            sys.stdout.write("[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
            sys.stdout.flush()

        # Compute the task function and task jacobian
        r, jac_r = robot.task_function(htm_des, np.array(q))
        # r[3:6] = np.sqrt(r[3:6])

        # Compute the task function and task jacobian in the next configuration
        r_next, jac_r_next = robot.task_function(htm_des, np.array(q + qdot * dt))
        # r_next[3:6] = np.sqrt(r_next[3:6])

        # Compute the time derivative of the jacobian and task function
        r_dot = (r_next - r) / dt
        jac_r_dot = (jac_r_next - jac_r) / dt

        # Create the h distance struct data
        dist_struct = robot.compute_dist(obstacle, h, h, np.array(q), old_dist_struct=dist_struct)
        dist_struct_next = robot.compute_dist(obstacle, h, h, np.array(q + qdot * dt), old_dist_struct=dist_struct)

        jac_dist = dist_struct.jac_dist_mat
        jac_dist_next = dist_struct_next.jac_dist_mat

        dist_vect = dist_struct.dist_vect
        dist_vect_next = dist_struct_next.dist_vect

        jac_dist_dot = (jac_dist_next - jac_dist) / dt
        dist_vect_dot = (dist_vect_next - dist_vect) / dt

        # Create the true distance struct data
        true_dist_struct = robot.compute_dist(obstacle, 0.0001, 0.0001, np.array(q), old_dist_struct=dist_struct)

        # Create the optimization problem and solve it for the joint acceleration
        a_hat = -Utils.dp_inv(jac_r, 0.001) @ (jac_r_dot @ qdot + alpha * r_dot)

        H = 2 * (np.transpose(jac_r) @ jac_r) + 2 * beta * np.identity(6)
        f = 2 * np.transpose(jac_r) @ (jac_r_dot @ qdot + 2 * alpha * r_dot + alpha * alpha * r) - 2 * beta * a_hat
        A = jac_dist
        b = -jac_dist_dot @ qdot - 2 * gamma * dist_vect_dot - gamma * gamma * dist_vect

        A = A[4, :]
        b = b[4, :]

        A = np.vstack((A, np.identity(6)))
        A = np.vstack((A, -np.identity(6)))

        m = np.shape(b)[0]

        b = np.vstack((b.reshape((m, 1)), -qddotmax * np.ones((6, 1))))
        b = np.vstack((b.reshape((m + 6, 1)), -qddotmax * np.ones((6, 1))))

        try:
            # u = quadprog.solve_qp(H, -f, np.transpose(A), b.reshape(m + 12, ))[0]
            qddot = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b.reshape(m + 12, )))['x']
            qddot = np.reshape(qddot, (6,1))

            # Integrate the joint acceleration
            q = q + qdot * dt
            qdot = qdot + qddot * dt
            robot.add_ani_frame(i * dt, q)


            # Record data for later plotting
            hist_dist.append(true_dist_struct[4].distance)
            hist_time.append((i - 1) * dt)
            hist_r.append(r.reshape((6,)))
            hist_q.append(q.reshape((6,)))
            hist_qdot.append(qdot.reshape((6,)))
            hist_qddot.append(qddot.reshape((6,)))
            hp.append(dist_struct[4].point_object)

            #print("time = "+str((i-1)*dt)+", q="+str(q.tolist()))

        except:
            end_loop = True

        i += 1

    # Run simulation
    sim.run()

    # Plot graphs

    Utils.plot(hist_time, hist_dist, "", "Time (s)", "True distance (m)", "dist")
    Utils.plot(hist_time, np.transpose(hist_q), "", "Time (s)", "Joint configuration (rad)", "q")
    Utils.plot(hist_time, np.transpose(hist_qdot), "", "Time (s)", "Joint speed (rad/s)", "qdot")
    Utils.plot(hist_time, np.transpose(hist_qddot), "", "Time (s)", "Joint acceleration (rad/s²)", "u")
    fig = Utils.plot(hist_time, np.transpose(hist_r), "", "Time (s)", "Task function",
                     ["posx", "posy", "posz", "orix", "oriy", "oriz"])

    return sim
