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
    robot = rb.Robot.create_abb_crb(Utils.trn([-0.2,0,0.3]), "robo")
    n = len(robot.links)

    mesh_board = MeshMaterial(roughness=1, metalness=0.9)
    board = Box(htm=Utils.trn([0.6, 0, 0.5]), width=0.05, depth=0.9, height=0.8, color="white",
                mesh_material=mesh_board)
    material_box = MeshMaterial(color="#242526", roughness=1, metalness=1)
    base = Cylinder(htm=Utils.trn([-0.2, 0, 0.15]), radius=0.1, height=0.3, mesh_material = material_box)
    sim = Simulation.create_sim_factory([robot, board, base])


    # Create curve
    theta = np.linspace(0, 2 * np.pi, num=300)
    curve = np.matrix(np.zeros(  (3, len(theta))))
    for i in range(len(theta)):
        t = theta[i]
        curve[:,i] = np.matrix([ [0.56], [0.2 * cos(t)], [0.2 * sin(t) + 0.5]])

    # Create vector field
    vf = rb.Robot.vector_field(curve, 10, 0.3)

    # Parameters
    dt = 0.01
    time_max = 20
    K = 1
    imax = round(time_max / dt)

    # Initializations
    hist_time = []
    hist_qdot = np.matrix(np.zeros((6,0)))
    hist_q = np.matrix(np.zeros((6,0)))
    hist_error_ori = np.matrix(np.zeros((3,0)))

    x_des = np.matrix([0, 0, 1]).reshape((3, 1))
    y_des = np.matrix([0, -1, 0]).reshape((3, 1))
    z_des = np.matrix([1, 0, 0]).reshape((3, 1))

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

        target = np.matrix(np.zeros((6,1)))
        target[0:3] = vf(p_eef)
        target[3] = -K * sqrt(max(1 - x_des.T * x_eef, 0))
        target[4] = -K * sqrt(max(1 - y_des.T * y_eef, 0))
        target[5] = -K * sqrt(max(1 - z_des.T * z_eef, 0))

        jac_target = np.matrix(np.zeros((6, n)))
        jac_target[0:3, :] = jac_eef[0:3, :]
        jac_target[3, :] = x_des.T * Utils.S(x_eef) * jac_eef[3:6, :]
        jac_target[4, :] = y_des.T * Utils.S(y_eef) * jac_eef[3:6, :]
        jac_target[5, :] = z_des.T * Utils.S(z_eef) * jac_eef[3:6, :]

        qdot = Utils.dp_inv(jac_target, 0.002) * target

        q_prox = robot.q + qdot * dt

        robot.add_ani_frame(i * dt, q_prox)

        hist_time.append(i * dt)
        hist_q = np.block([hist_q, robot.q])
        error_ori = np.matrix([(180 / (np.pi)) * acos(1 - min(num * num / (K * K),2)) for num in target[3:6]]).reshape((3,1))
        hist_error_ori = np.block([hist_error_ori, error_ori])
        hist_qdot = np.block([hist_qdot, qdot])

        # See if the end-effector is close to the board to add to the point cloud
        draw_points = np.block([draw_points, p_eef])


        if (not reached_board) and (abs(p_eef[0,0] - board.htm[0,3]) < board.width / 2 + 0.001):
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
    Utils.plot(hist_time, hist_q, "", "Time (s)", "Joint configuration  (rad)", "q")
    Utils.plot(hist_time, hist_qdot, "", "Time (s)", "Joint speed (rad/s)", "u")
    Utils.plot(hist_time, hist_error_ori, "", "Time (s)", "Orientation error (degrees)", ['x', 'y', 'z'])

    return sim
