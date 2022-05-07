from utils import *
import numpy as np


# Geometric Jacobian
def _jac_geo(self, q=None, axis='eef', htm=None):
    if q is None:
        q = self.q
    if htm is None:
        htm = np.matrix(self.htm)

    n = len(self.links)

    # Error handling
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not (axis == "eef" or axis == "dh" or axis == "com"):
        raise Exception("The parameter 'axis' should be one of the following strings:\n" \
                        "'eef': End-effector \n" \
                        "'dh': All " + str(n) + " axis of Denavit-Hartenberg\n" \
                                                "'com': All " + str(
            n) + " axis centered at the center of mass of the objects.")

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")
    # end error handling

    if axis == 'dh' or axis == 'eef':
        htm_for_jac = self.fkm(q, 'dh', htm)
    if axis == 'com':
        htm_for_jac = self.fkm(q, 'com', htm)

    jac = [np.matrix(np.zeros((6,n))) for i in range(n)]

    htm_world_0 = htm * self.htm_base_0

    for i in range(n):
        p_i = htm_for_jac[i][0:3, 3]
        for j in range(i + 1):

            if j == 0:
                p_j_ant = htm_world_0[0:3, 3]
                z_j_ant = htm_world_0[0:3, 2]
            else:
                p_j_ant = htm_for_jac[j - 1][0:3, 3]
                z_j_ant = htm_for_jac[j - 1][0:3, 2]

            if self.links[j].joint_type == 0:
                jac[i][0:3, j] = Utils.S(z_j_ant) * (p_i - p_j_ant)
                jac[i][3:6, j] = z_j_ant

            if self.links[j].joint_type == 1:
                jac[i][0:3, j] = z_j_ant
                jac[i][3:6, j] = np.matrix(np.zeros((3,)))

    if axis == 'dh' or axis == 'com':
        return jac, htm_for_jac

    if axis == 'eef':
        return jac[-1][:, :], htm_for_jac[-1][:, :]
