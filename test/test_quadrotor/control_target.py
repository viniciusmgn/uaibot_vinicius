import uaibot as ub
import numpy as np
from cvxopt import matrix, solvers

def control_target(robot, radius_robot, obstacle, radius_obstacle, p_target):

    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)

    eta = 0.1
    kp = 0.4
    costheta = np.cos(np.pi/6)

    dist_min = 1000
    A = np.matrix(np.zeros((0, 6)))
    b = np.matrix(np.zeros((0, 1)))
    list_dist=[]

    p=robot.htm[0:3,-1]

    #Compute the distances
    for i in range(len(obstacle)):
        if np.linalg.norm(p - obstacle[i].htm[0:3, -1]) - (radius_robot + radius_obstacle[i]) < 0.4:
            p_robot, p_obs, dist = ub.Utils.compute_dist(robot, obstacle[i])

            a = np.block([(p_robot - p_obs).T, (p_robot - p_obs).T * ub.Utils.S(p - p_robot)])
            A = np.block([[A], [a]])
            b = np.block([[b], [-eta * (dist - 0.03)]])

            list_dist.append(dist)
            dist_min = min(dist_min, dist)

    #Create orientation constraint
    z = robot.htm[0:3,2]
    z_d = np.matrix([[0],[0],[1]])
    a = np.block([ np.matrix([0,0,0])  , -z_d.T * ub.Utils.S( z )  ])
    A = np.block([[A], [a]])
    b = np.block([[b], [-eta * (z_d.T * z - costheta)]])

    #Create and solve the optimization problem
    H = 2 * np.diag([1, 1, 1, 0.1, 0.1, 0.1])
    f = np.block([[2 * kp * (p - p_target)], [0], [0], [0]])

    try:
        u = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b))['x']
        u = np.reshape(u, (6, 1))
        v = np.matrix(u[0:3, 0]).reshape((3, 1))
        w = np.matrix(u[3:6, 0]).reshape((3, 1))
        error = False
    except:
        v = np.matrix(np.zeros((3, 1)))
        w = np.matrix(np.zeros((3, 1)))
        error = True

    return v, w, error, dist_min