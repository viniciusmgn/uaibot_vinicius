from utils import *
import numpy as np
from cvxopt import matrix, solvers

def _const_control(self, htm_des, q=None, htm=None, obstacles = [], eta_obs=0.5, eta_joint=0.5, dist_safe=0.01, kp = 0.5,
                   dict_old_dist_struct=None, eps=0.001, max_dist = np.inf):

    solvers.options['show_progress'] = False
    np.set_printoptions(precision=4, suppress=True, linewidth=150)


    if q is None:
        q = self.q
    if htm is None:
        htm = np.matrix(self.htm)

    n = len(self._links)

    # Error handling
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")
    # end error handling

    r, Jr = self.task_function(htm_des, q, htm)


    A = np.matrix(np.zeros(( 0, n )))
    b = np.matrix(np.zeros(( 0, 1 )))

    #Create constraint due to collision with obstacles
    dict_dist_struct = {}

    for obs in obstacles:
        old_dist_struct = None if dict_old_dist_struct is None else dict_old_dist_struct[obs]
        dist_struct = self.compute_dist(obs, q, htm, old_dist_struct, max_dist=max_dist)
        dict_dist_struct[obs] = dist_struct
        A = np.block([[A],[dist_struct.jac_dist_mat]])
        b = np.block([[b], [-eta_obs*(dist_struct.dist_vect-dist_safe)]])

    #Create joint limits constraints
    A = np.block([[A], [np.identity(n)]])
    b = np.block([[b], [-eta_joint * (q - self.joint_limit[:,0])]])
    A = np.block([[A], [-np.identity(n)]])
    b = np.block([[b], [-eta_joint * (self.joint_limit[:,1]-q)]])

    #Create objective function

    H = 2 * Jr.T * Jr + 2 * eps*np.identity(n)
    f = 2 * kp * Jr.T * r

    failure=False
    try:
        qdot = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b))['x']
    except:
        qdot = np.matrix(np.zeros( (n,1) ))
        failure=True

    return qdot, failure, r, dict_dist_struct