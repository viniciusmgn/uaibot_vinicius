from utils import *
import numpy as np


# Function used for task function/task Jacobian
def _task_function(self, htm_des, q=None, htm=None):
    if q is None:
        q = self.q

    if htm is None:
        htm = self.htm

    # Error handling
    if not Utils.is_a_matrix(htm_des, 4, 4):
        raise Exception("The parameter 'htm_des' should be a 4x4 homogeneous transformation matrix.")

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    n = len(self.links)
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    # end error handling

    p_des = htm_des[0:3, 3]
    x_des = htm_des[0:3, 0]
    y_des = htm_des[0:3, 1]
    z_des = htm_des[0:3, 2]

    jac_eef, htm_eef = self.jac_geo(q, "eef", htm)
    p_eef = htm_eef[0:3, 3]
    x_eef = htm_eef[0:3, 0]
    y_eef = htm_eef[0:3, 1]
    z_eef = htm_eef[0:3, 2]

    r = np.matrix(np.zeros((6,1)))
    r[0:3,0] = p_eef - p_des
    r[3] = max(1 - x_des.T * x_eef, 0)
    r[4] = max(1 - y_des.T * y_eef, 0)
    r[5] = max(1 - z_des.T * z_eef, 0)

    n = len(self.links)
    jac_r = np.matrix(np.zeros((6, n)))
    jac_r[0:3, :] = jac_eef[0:3, :]
    jac_r[3, :] = x_des.T * Utils.S(x_eef) * jac_eef[3:6, :]
    jac_r[4, :] = y_des.T * Utils.S(y_eef) * jac_eef[3:6, :]
    jac_r[5, :] = z_des.T * Utils.S(z_eef) * jac_eef[3:6, :]



    return r, jac_r
