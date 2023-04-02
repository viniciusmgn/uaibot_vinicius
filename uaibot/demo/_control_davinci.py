import sys
import time
import numpy as np
import robot as rb
import plotly.graph_objects as go
from simulation import Simulation
from utils import Utils
from simobjects.box import Box
from simobjects.ball import Ball
from simobjects.frame import Frame
from cvxopt import matrix, solvers
from graphics.meshmaterial import MeshMaterial, Texture
from plotly.subplots import make_subplots


def create_ballpoints(N):
    balls = []
    for i in range(N):
        ball_link = Ball(name=f'LinkBall{i}', radius=0.02, color='black')
        ball_obj = Ball(name=f'ObjBall{i}', radius=0.02, color='red')
        balls.append((ball_link, ball_obj))

    return balls


def add_witnesspoints(balls, closest_idxs, time, dist_struct):
    for i, idx in enumerate(closest_idxs):
        htm_point_link = Utils.trn(
            dist_struct[idx//28].__getitem__(idx % 28).point_link)
        htm_point_object = Utils.trn(
            dist_struct[idx//28].__getitem__(idx % 28).point_object)
        balls[i][0].add_ani_frame(time=time, htm=htm_point_link)
        balls[i][1].add_ani_frame(time=time, htm=htm_point_object)


def dist_computation_heatmap(robot, q, obstacles, old_struct=[None]):
    d = np.inf
    msg = ''
    struct = []

    for i, obstacle in enumerate(obstacles):
        dist = robot.compute_dist(
            obj=obstacle, q=q, old_dist_struct=old_struct[i])
        struct.append(dist)
        for idx, a in enumerate(dist.dist_vect):
            if a < d:
                msg = 'link' + str(dist[idx].link_number) + '  Obj #:' + \
                    str(dist[idx].link_col_obj_number) + f' OBstacle: {i}'
                d = a

    return d, msg, struct


def heatmatp(robot, q, obstacles, i, j, res=30):
    q_mod = q.copy().astype(float)
    mat = np.zeros((res, res))
    msgs = []
    struct = [None,] * len(obstacles)

    for k in range(res):
        print(k)
        msgs.append([])
        q_mod[i, 0] = robot.joint_limit[i, 0] + \
            (robot.joint_limit[i, 1] - robot.joint_limit[i, 0]) * k / 29

        if (k == 5):
            q_mod[6, 0] = robot.joint_limit[6, 0] + \
                (robot.joint_limit[i, 1] - robot.joint_limit[i, 0]) * k / 29
            q_mod[7, 0] = robot.joint_limit[7, 0] + \
                (robot.joint_limit[i, 1] - robot.joint_limit[i, 0]) * k / 29

        for w in range(res):
            q_mod[j, 0] = robot.joint_limit[j, 0] + \
                (robot.joint_limit[j, 1] - robot.joint_limit[j, 0]) * w / 29

            if (w == 5):
                q_mod[6, 0] = robot.joint_limit[6, 0] + \
                    (robot.joint_limit[j, 1] -
                     robot.joint_limit[j, 0]) * w / 29
                q_mod[7, 0] = robot.joint_limit[7, 0] + \
                    (robot.joint_limit[j, 1] -
                     robot.joint_limit[j, 0]) * w / 29

            d, msg, struct = dist_computation_heatmap(
                robot, q_mod, obstacles, old_struct=struct)
            mat[k, w] = d
            msgs[k].append(msg)

    return mat, msgs


right_foot = Box(htm=Utils.trn([0.26, -0.26, 0.09]),
                 width=0.25, height=0.19, depth=1.4, opacity=0.5)
left_foot = Box(htm=Utils.trn([-0.46, -0.26, 0.09]),
                width=0.25, height=0.19, depth=1.4, opacity=0.5)
center_foot = Box(htm=Utils.trn(
    [-0.1037, -0.52, 0.21]), width=0.62, height=0.44, depth=0.8, opacity=0.5)
cable_rest = Box(htm=Utils.trn([-0.1037, -0.93, 0.4]),
                 width=0.35, height=0.82, depth=0.35, opacity=0.5)
tower = Box(htm=Utils.trn([-0.10, -0.275, 0.9]),
            width=0.2, height=1.7, depth=0.19, opacity=0.5)
stat_parts = [right_foot, left_foot, center_foot, cable_rest, tower]


def auto_collision_helper(robot_, q=None, stat_parts=stat_parts, old_struct=[None, None, None, None, None], max_dist=np.inf):
    """Computes autocollision between joints and stationary part (chest) of davinci"""
    dist_vect, gradD_list, struct = [], [], []
    n = len(robot_.links)

    for i, obj in enumerate(stat_parts):
        dist_struct = robot_.compute_dist(
            obj, q=q, old_dist_struct=old_struct[i], max_dist=max_dist)
        dist, grad = np.array(dist_struct.dist_vect), np.array(
            dist_struct.jac_dist_mat)
        idxs_aux = 1 - np.zeros_like(dist, dtype=bool).ravel()

        # Skip first joint distances
        for k, d in enumerate(dist_struct):
            if d.link_number == 0:
                idxs_aux[k] = False
            elif d.link_number == 1 and d.link_col_obj_number == 2:
                idxs_aux[k] = False

        dist = dist[idxs_aux.astype(bool)]
        grad = grad[idxs_aux.astype(bool)]
        if dist.size > 0:
            dist_vect.append(dist)
            gradD_list.append(grad)
        struct.append(dist_struct)

    dist_vect = np.array(dist_vect).reshape(-1, 1)
    gradD_list = np.array(gradD_list).reshape(-1, n)

    return dist_vect, gradD_list, struct


def _control_demo_davinci(arm=0):
    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)

    robot_arm1, robot_arm2, robot_arm3, robot_arm4, chest = rb.Robot.create_davinci().list_of_objects
    arms = [robot_arm1, robot_arm2, robot_arm3, robot_arm4]
    robot = arms[arm]

    texture_wall = Texture(
        url='https://raw.githubusercontent.com/viniciusmgn/uaibot_content/master/contents/Textures/rough_metal.jpg',
        wrap_s='RepeatWrapping', wrap_t='RepeatWrapping', repeat=[4, 4])

    material_wall = MeshMaterial(texture_map=texture_wall, clearcoat=0.3, roughness=0.8,
                                 metalness=0.7, opacity=0.9, color='silver', side="DoubleSide")

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
        error_ori = (180 / np.pi) * max(abs(np.arccos(1 - r[3:6])))
        ok1 = error_pos < tol_pos
        ok2 = error_ori < 5 if len(r.tolist()) > 3 else True

        return ok1 and ok2, error_pos, error_ori

    def dist_computation(q, old_struct, obstacles=[wall1, wall2, wall3, wall4, wall5], max_dist=np.inf):
        n = len(robot.links)
        dist_vect, gradD_list, struct = 5, [5,]*n, []

        for i, obstacle in enumerate(obstacles):
            dist = robot.compute_dist(
                obj=obstacle, q=q, old_dist_struct=old_struct[i], max_dist=max_dist)
            struct.append(dist)
            if dist.dist_vect.size > 0:
                if isinstance(dist_vect, np.ndarray):
                    dist_vect = np.vstack(
                        (dist_vect, np.array(dist.dist_vect).reshape(-1, 1)))
                    gradD_list = np.vstack(
                        (gradD_list, np.array(dist.jac_dist_mat).reshape(-1, n)))
                else:
                    dist_vect = np.array(dist.dist_vect).reshape(-1, 1)
                    gradD_list = np.array(dist.jac_dist_mat).reshape(-1, n)

        dist_vect = np.array(dist_vect).reshape(-1, 1)
        gradD_list = np.array(gradD_list).reshape(-1, n)

        return dist_vect, gradD_list, struct

    # Target pose definition
    pose_tg = []
    pose_tg.append(Utils.trn([0.3 - 0.8, 0.7, .7]) @ Utils.rotx(np.deg2rad(130))
                   @ Utils.rotz(np.deg2rad(-100)) @ Utils.roty(np.deg2rad(-12)))

    # Create simulation
    sim = Simulation.create_sim_grid(
        [robot, wall1, wall2, wall3, wall4, wall5, chest])

    for k in range(len(pose_tg)):
        sim.add(Frame(name="pose_tg_" + str(k), htm=pose_tg[k]))

    # Parameters
    dt = 0.03
    K = 2
    eta = 0.5
    sigma = 0.2
    dist_safe = 0.01
    max_dist = 0.5
    xi = 1
    q = robot.q.astype(float)
    n = len(robot.links)
    Ntest = 4
    qdot = np.zeros((n, 1)).astype(float)

    # Initializations
    obstacles = [wall1, wall2, wall3, wall4, wall5]
    struct = [None,] * len(obstacles)
    auto_struct = None
    auto_struct2 = [None,] * len(stat_parts)

    i = 1
    imax = round((47.4/2)/dt)

    hist_dist = []
    hist_time = []
    hist_r = np.matrix(np.zeros(shape=(6, 0)))
    hist_q = np.matrix(np.zeros(shape=(n, 0)))
    hist_qdot = np.matrix(np.zeros(shape=(n, 0)))
    build_ts, solve_ts = [], []
    size_restrictions = []

    error_qp = False
    iteration_end = False

    # Changing Initial configuration
    qaux = q.copy()
    qaux[3] = -np.deg2rad(179)
    qaux[4] = -np.pi/2
    qaux[5] = -np.deg2rad(120)
    qaux[6] = -np.deg2rad(110)
    qaux[7] = 0
    robot.add_ani_frame(0, qaux)
    q65_c = qaux[6] - qaux[5]
    q75_c = qaux[7] - qaux[5]
    q = robot.q.astype(float)

    # Create witness points
    #balls = create_ballpoints(Ntest)
    # for b in balls:
    #     sim.add(b[0])
    #     sim.add(b[1])

    # Main loop
    for k in range(len(pose_tg)):
        converged = False
        while not converged and not error_qp and not iteration_end:

            qdot_max = robot.joint_limit[:, 1] - q.reshape(-1, 1)
            qdot_min = robot.joint_limit[:, 0] - q.reshape(-1, 1)
            qdot_max[1:-1] *= 10
            qdot_min[1:-1] *= 10

            # This is just for showing progress when the simulation is run
            if i % 50 == 0 or i == imax - 1:
                sys.stdout.write('\r')
                sys.stdout.write(
                    "[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
                sys.stdout.flush()

            # Compute r
            [r, jac_r] = robot.task_function(pose_tg[k], q=q)

            # Compute dist_vect, dist_vect_dot and the distance Jacobian
            build_t0 = time.time()
            dist_vect, gradD_list, struct = dist_computation(
                q, struct, obstacles=obstacles, max_dist=max_dist)

            # Create the quadratic program parameters
            A = -gradD_list
            b = eta * (dist_vect - dist_safe)
            #idx = np.argsort(dist_vect.ravel())

            # AutoCollision
            auto_struct = robot.compute_dist_auto(max_dist=max_dist/2)
            dist_vect_auto, jac_dist_auto = auto_struct.dist_vect, auto_struct.jac_dist_mat
            dist_vect_auto, jac_dist_auto = np.array(
                dist_vect_auto), np.array(jac_dist_auto)
            idx_auto = np.argsort(dist_vect_auto.ravel())
            b_auto = eta * \
                (np.array(dist_vect_auto).reshape(-1, 1) - dist_safe)
            b_auto = b_auto[idx_auto]
            A_auto = -np.array(jac_dist_auto).reshape(-1, n)
            A_auto = A_auto[idx_auto, :]
            b_auto2, A_auto2, auto_struct2 = auto_collision_helper(
                robot, old_struct=auto_struct2, max_dist=max_dist)
            b_auto2 = eta * (b_auto2 - dist_safe)
            A_auto2 = -A_auto2

            H = 2 * (jac_r.T * jac_r) + 0.001 * np.identity(n)
            f = (2 * K * r.T @ jac_r).T
            q6_rest = np.array([0, 0, 0, 0, 0, -1, 1, 0, 0]).reshape(1, -1)
            q7_rest = np.array([0, 0, 0, 0, 0, -1, 0, 1, 0]).reshape(1, -1)
            A = np.block([[A], [A_auto], [A_auto2], [np.identity(
                n)], [-np.identity(n)], [q6_rest], [q7_rest], [-q6_rest], [-q7_rest]])
            b = np.block([[b], [b_auto], [b_auto2], [xi * qdot_max], [-xi * qdot_min], [-sigma * (q[6] - q[5] - q65_c)],
                         [-sigma * (q[7] - q[5] - q75_c)], [sigma * (q[6] - q[5] - q65_c)], [sigma * (q[7] - q[5] - q75_c)]])
            deltaT_build = time.time() - build_t0

            if dist_vect.size <= 0:
                dist_vect = np.array([[4]])
            size_restrictions.append(b.shape[0])

            # Solve the quadratic program
            solve_t0 = time.time()
            try:
                qdot = solvers.qp(matrix(H), matrix(
                    f), matrix(A), matrix(b))['x']
            except:
                qdot = np.matrix(np.zeros((n, 1)))
                error_qp = True

            deltaT_solve = time.time() - solve_t0
            qdot = np.array(qdot).reshape(n, 1)

            # First order explicit Euler simulation
            q += qdot * dt

            # Add animation to simulation
            robot.add_ani_frame(i * dt, q)
            #add_witnesspoints(balls=balls, closest_idxs=idx[:Ntest], time=i*dt, dist_struct=struct)

            # Store data for showing the graphs later
            hist_time.append(i * dt)
            hist_dist.append(np.amin(dist_vect))
            hist_r = np.block([hist_r, r])
            hist_q = np.block([hist_q, q])
            hist_qdot = np.block([hist_qdot, qdot])
            build_ts.append(deltaT_build)
            solve_ts.append(deltaT_solve)

            # Continue the loop, check if converged
            i += 1
            converged, error_pos, error_ori = evaluate_error(r, 5e-4)
            if converged:
                print('\nConverged in {i} iterations. Posisition error: {poser}, Orientation error: {orier}'.format(i=i, poser=float(error_pos), orier=float(error_ori)))

            iteration_end = i > imax
    if i >= imax:
        print('Max iterations reached')
    elif error_qp:
        print('QP error')
    # Show collision primitives
    # for i in robot.links:
    #     for j in i.col_objects:
    #         sim.add(j[0])
    # robot.update_col_object(0)

    # Run simulation
    sim.run()

    # Plot graphs
    Utils.plot(hist_time, hist_dist, "", "Time (s)",
               "True distance (m)", "dist")
    
    fig =  make_subplots(specs=[[{"secondary_y": True}]])
    for i in range(hist_q.shape[0]):
        fig.add_trace(go.Scatter(x=hist_time, y=np.array(hist_q[i, :]).ravel(), name=f'q<sub>{i+1}</sub>'), secondary_y=(i==0 or i==8))
    fig.update_yaxes(title_text="Rotative Joint configuration (rad)", secondary_y=False)
    fig.update_yaxes(title_text="Prismatic Joint configuration (m)", secondary_y=True)
    fig.update_xaxes(title_text='Time (s)')
    fig.update_layout(width=800, height=400)
    fig.show()

    fig =  make_subplots(specs=[[{"secondary_y": True}]])
    for i in range(hist_qdot.shape[0]):
        fig.add_trace(go.Scatter(x=hist_time, y=np.array(hist_qdot[i, :]).ravel(), name=f'qdot<sub>{i+1}</sub>'), secondary_y=(i==0 or i==8))
    fig.update_yaxes(title_text="Rotative Joint speed (rad/s)", secondary_y=False)
    fig.update_yaxes(title_text="Prismatic Joint speed (m/s)", secondary_y=True)
    fig.update_xaxes(title_text='Time (s)')
    fig.update_layout(width=800, height=400)
    fig.show()
   
    fig = Utils.plot(hist_time, hist_r, "", "Time (s)", "Task function",
                     ["posx", "posy", "posz", "orix", "oriy", "oriz"])
    Utils.plot(np.arange(len(build_ts)), np.block([build_ts, solve_ts]), 
               'Efficiency Comparison: Time Spent Solving vs. Building the Problem',
               'Iteration', 'Time [s]', ['Build Time', 'Solve Time'])
    Utils.plot(np.arange(len(size_restrictions)), size_restrictions, 
               'Number of Constraints x Iterations',
               'Iteration', 'Size', ['Size'])

    # return sim, (build_ts, solve_ts, size_restrictions, (hist_time, hist_q, hist_dist, hist_qdot, hist_r))
