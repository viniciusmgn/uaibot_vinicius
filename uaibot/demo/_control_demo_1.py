from simulation import *
import numpy as np
from simobjects.ball import *
from simobjects.box import *
from simobjects.cylinder import *
from simobjects.pointlight import *
from simobjects.pointcloud import *
from utils import *
from graphics.meshmaterial import *
import robot as rb
import sys


def _control_demo_1():
    # Create simulation and add objects to the scene
    robot = rb.Robot.create_kuka_lbr_iiwa(np.identity(4), "robo")
    n = len(robot.links)

    mesh_board = MeshMaterial(roughness=1, metalness=0.9)
    board = Box(htm=Utils.trn([0.6, 0, 0.5]), width=0.05, depth=0.9, height=0.8, color="white",
                mesh_material=mesh_board)

    sim = Simulation.create_sim_factory([robot, board])


    # Create curve
    theta = np.linspace(0, 2 * np.pi, num=300)
    curve = []
    for t in theta:
        curve.append([0.575, 0.2 * cos(t), 0.2 * sin(t) + 0.5])

    # Create vector field
    vf = rb.Robot.vector_field(curve, 10, 0.3)

    # Parameters
    dt = 0.01
    time_max = 20
    K = 1
    imax = round(time_max / dt)

    # Initializations
    hist_time = []
    hist_qdot = []
    hist_q = []
    hist_error_ori = []

    x_des = np.array([0, 0, 1]).reshape((3, 1))
    y_des = np.array([0, -1, 0]).reshape((3, 1))
    z_des = np.array([1, 0, 0]).reshape((3, 1))

    # Main loop
    draw_points = np.zeros((3, 0))
    reached_board = False

    for i in range(imax):

        if i % 50 == 0 or i == imax - 1:
            sys.stdout.write('\r')
            sys.stdout.write("[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
            sys.stdout.flush()

        jac_eef, htm_eef = robot.jac_geo()

        p_eef = htm_eef[0:3, 3]
        x_eef = htm_eef[0:3, 0]
        y_eef = htm_eef[0:3, 1]
        z_eef = htm_eef[0:3, 2]

        target = np.zeros((6,))
        target[0:3] = vf(p_eef).reshape((3,))
        target[3] = -K * sqrt(max(1 - np.transpose(x_des) @ x_eef, 0))
        target[4] = -K * sqrt(max(1 - np.transpose(y_des) @ y_eef, 0))
        target[5] = -K * sqrt(max(1 - np.transpose(z_des) @ z_eef, 0))

        jac_target = np.zeros((6, n))
        jac_target[0:3, :] = jac_eef[0:3, :]
        jac_target[3, :] = np.transpose(x_des) @ Utils.S(x_eef) @ jac_eef[3:6, :]
        jac_target[4, :] = np.transpose(y_des) @ Utils.S(y_eef) @ jac_eef[3:6, :]
        jac_target[5, :] = np.transpose(z_des) @ Utils.S(z_eef) @ jac_eef[3:6, :]

        qdot = Utils.dp_inv(jac_target, 0.002) @ target

        q_prox = np.array(robot.q).reshape((n,)) + qdot * dt

        robot.add_ani_frame(i * dt, q_prox)

        hist_time.append(i * dt)
        hist_q.append(robot.q.reshape((n,)))
        hist_error_ori.append([(180 / (np.pi)) * acos(1 - min(num * num / (K * K),2)) for num in target[3:6]])
        hist_qdot.append(qdot)

        # See if the end-effector is close to the board to add to the point cloud
        draw_points = np.hstack((draw_points, p_eef.reshape((3, 1))))

        if (not reached_board) and (abs(p_eef[0] - board.htm[0][3]) < board.width / 2 + 0.001):
            reached_board = True
            ind_reached = i

    # Set up the cloud of points
    point_cloud = PointCloud(name="drawing", points=draw_points, size=0.025)
    sim.add(point_cloud)
    for i in range(imax):
        if i < ind_reached:
            point_cloud.add_ani_frame(i * dt, 0, 0)
        else:
            point_cloud.add_ani_frame(i * dt, ind_reached, i)

    # Run simulation
    sim.run()

    # Plot graphs
    Utils.plot(hist_time, np.transpose(hist_q), "", "Time (s)", "Joint configuration  (rad)", "q")
    Utils.plot(hist_time, np.transpose(hist_qdot), "", "Time (s)", "Joint speed (rad/s)", "u")
    Utils.plot(hist_time, np.transpose(hist_error_ori), "", "Time (s)", "Orientation error (degrees)", ['x', 'y', 'z'])

    return sim
