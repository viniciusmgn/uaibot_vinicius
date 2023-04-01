from utils import *
import numpy as np
from cvxopt import matrix, solvers
import math

def _const_control_mod(self, htm_des, q=None, htm=None, obstacles = [], dict_old_dist_struct=None,
                   eta_obs=0.5, eta_joint=0.5, eta_auto=0.5,
                   dist_safe_obs=0.01, dist_safe_auto=0.01,
                   max_dist_obs=np.inf, max_dist_auto = np.inf, task_rate_fun = 0.5, eps=0.001):


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

    if not str(type(obstacles)) == "<class 'list'>":
        raise Exception("The parameter 'list' should be a list of simple objects.")

    for obs in obstacles:
        if not Utils.is_a_simple_object(obs):
            raise Exception("The parameter 'list' should be a list of simple objects.")



    if not Utils.is_a_number(eta_joint) or eta_joint <= 0:
        raise Exception("The parameter 'eta_joint' should be a positive number.")

    if not Utils.is_a_number(eta_auto) or eta_auto <= 0:
        raise Exception("The parameter 'eta_auto' should be a positive number.")



    if not Utils.is_a_number(dist_safe_auto) or dist_safe_auto <= 0:
        raise Exception("The parameter 'dist_safe_auto' should be a positive number.")

    if not Utils.is_a_number(max_dist_obs) or max_dist_obs <= 0:
        raise Exception("The parameter 'max_dist_obs' should be a positive number.")

    if not Utils.is_a_number(max_dist_auto) or max_dist_auto <= 0:
        raise Exception("The parameter 'max_dist_auto' should be a positive number.")

    if not (Utils.is_a_number(task_rate_fun) and task_rate_fun > 0) and not str(type(task_rate_fun)) == "<class 'function'>":
        raise Exception("The parameter 'task_rate_fun' should be a positive number or a positive function.")

    if not Utils.is_a_number(eps) or eps <= 0:
        raise Exception("The parameter 'eps' should be a positive number.")

    # end error handling

    r, Jr = self.task_function(htm_des, q, htm)

    if str(type(task_rate_fun)) == "<class 'function'>":
        try:
            tg_dot_r = task_rate_fun(r)
            if Utils.is_a_matrix(tg_dot_r,6,1):
                Exception("The output of f should be a 6 dimensional column vector.")
        except:
            raise Exception("There is an error with function 'f'. Check its code.")
    else:
        tg_dot_r = task_rate_fun * r


    A = np.matrix(np.zeros(( 0, n )))
    b = np.matrix(np.zeros(( 0, 1 )))

    #Create constraint due to collision with obstacles
    if True:
        dict_dist_struct = {}

        for obs in obstacles:
            old_dist_struct = None if dict_old_dist_struct is None else dict_old_dist_struct[obs]
            dist_struct = self.compute_dist(obs, q, htm, old_dist_struct, max_dist=max_dist_obs)
            dict_dist_struct[obs] = dist_struct
            A = np.block([[A], [dist_struct.jac_dist_mat]])
            b = np.block([[b], [-eta_obs[obs] * (dist_struct.dist_vect - dist_safe_obs[obs])]])

    #Create joint limits constraints
    if not math.isinf(eta_joint):
        A = np.block([[A], [np.identity(n)]])
        b = np.block([[b], [-eta_joint * (q - self.joint_limit[:, 0])]])
        A = np.block([[A], [-np.identity(n)]])
        b = np.block([[b], [-eta_joint * (self.joint_limit[:, 1] - q)]])

    #Create auto collision constraints
    #TO DO

    #Create objective function

    H = 2 * Jr.T * Jr + 2 * eps*np.identity(n)
    f = 2 * Jr.T * tg_dot_r

    try:
        qdot = solvers.qp(matrix(H), matrix(f), matrix(-A), matrix(-b))['x']
        failure = False
    except:
        qdot = np.matrix(np.zeros( (n,1) ))
        failure=True

    return qdot, failure, r, dict_dist_struct, A, b