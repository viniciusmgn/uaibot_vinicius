from utils import *
import numpy as np


# Function used for task function/task Jacobian
def _coop_task_function(robot_a, robot_b, htm_a_des, htm_a_b_des, q_a, q_b):
    if q_a is None:
        q_a = robot_a.q

    if q_b is None:
        q_b = robot_b.q

    # Error handling
    if not Utils.get_uaibot_type(robot_a) == 'uaibot.Robot':
        raise Exception("The parameter 'robot_a' should be a 'uaibot.Robot' object.")

    if not Utils.is_a_vector(q_a, len(robot_a.links)):
        raise Exception("The parameter 'q_a' should be a " + str(len(robot_a.links)) + " dimensional vector.")

    if not Utils.get_uaibot_type(robot_b) == 'uaibot.Robot':
        raise Exception("The parameter 'robot_b' should be a 'uaibot.Robot' object.")

    if not Utils.is_a_vector(q_a, len(robot_b.links)):
        raise Exception("The parameter 'q_b' should be a " + str(len(robot_b.links)) + " dimensional vector.")

    if not Utils.is_a_matrix(htm_a_des, 4, 4):
        raise Exception("The parameter 'htm_a_des' should be a 4x4 homogeneous transformation matrix.")

    if not Utils.is_a_matrix(htm_a_b_des, 4, 4):
        raise Exception("The parameter 'htm_a_b_des' should be a 4x4 homogeneous transformation matrix.")
    # end error handling

    jac_a, htm_a = robot_a.jac_geo(q_a)
    jac_b, htm_b = robot_b.jac_geo(q_b)
    rot_a = htm_a[0:3, 0:3]
    p_a = htm_a[0:3, 3]
    rot_b = htm_b[0:3, 0:3]
    p_b = htm_b[0:3, 3]

    inv_htm_a = Utils.inv_htm(htm_a)
    htm_a_b = inv_htm_a @ htm_b

    n_a = len(robot_a.links)
    n_b = len(robot_b.links)
    r = np.matrix(np.zeros((12, 1)))
    jac_r = np.matrix(np.zeros((12, n_a + n_b)))

    # First part of the cooperative task function (relative pose between A and B)

    x_rel_des = htm_a_b_des[0:3, 0]
    y_rel_des = htm_a_b_des[0:3, 1]
    z_rel_des = htm_a_b_des[0:3, 2]
    p_rel_des = htm_a_b_des[0:3, 3]

    x_rel = htm_a_b[0:3, 0]
    y_rel = htm_a_b[0:3, 1]
    z_rel = htm_a_b[0:3, 2]
    p_rel = htm_a_b[0:3, 3]

    jac_rel_v = rot_a.T * np.hstack((Utils.S(p_b - p_a) * jac_a[3:6, :] - jac_a[0:3, :], jac_b[0:3, :]))
    jac_rel_w = rot_a.T * np.hstack((-jac_a[3:6, :], jac_b[3:6, :]))

    r[0:3] = p_rel - p_rel_des
    r[3] = max(1 - x_rel_des.T * x_rel, 0)
    r[4] = max(1 - y_rel_des.T * y_rel, 0)
    r[5] = max(1 - z_rel_des.T * z_rel, 0)

    jac_r[0:3, :] = jac_rel_v
    jac_r[3, :] = x_rel_des.T * Utils.S(x_rel) * jac_rel_w
    jac_r[4, :] = y_rel_des.T * Utils.S(y_rel) * jac_rel_w
    jac_r[5, :] = z_rel_des.T * Utils.S(z_rel) * jac_rel_w

    # Second part of the cooperative task function (pose for A)
    x_a_des = htm_a_des[0:3, 0]
    y_a_des = htm_a_des[0:3, 1]
    z_a_des = htm_a_des[0:3, 2]
    p_a_des = htm_a_des[0:3, 3]

    x_a = htm_a[0:3, 0]
    y_a = htm_a[0:3, 1]
    z_a = htm_a[0:3, 2]
    p_a = htm_a[0:3, 3]

    r[6:9] = p_a - p_a_des
    r[9] =  max(1 - x_a_des.T * x_a, 0)
    r[10] = max(1 - y_a_des.T * y_a, 0)
    r[11] = max(1 - z_a_des.T * z_a, 0)

    jac_r[6:9, :] = np.hstack((jac_a[0:3, :], np.zeros((3, n_b))))
    jac_w_a = np.hstack((jac_a[3:6, :], np.zeros((3, n_b))))
    jac_r[9, :] =  x_a_des.T * Utils.S(x_a) * jac_w_a
    jac_r[10, :] = y_a_des.T * Utils.S(y_a) * jac_w_a
    jac_r[11, :] = z_a_des.T * Utils.S(z_a) * jac_w_a

    r = r.reshape((12,1))

    return r, jac_r
