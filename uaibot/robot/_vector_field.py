import numpy as np
from utils import *

_INVHALFPI = 0.63660


def _vector_field(curve, alpha, const_vel):
    # Error handling
    if not Utils.is_a_matrix(curve):
        raise Exception("The parameter 'curve' should be a matrix of float numbers.")

    vector_size =  np.shape(curve)[0]

    if not Utils.is_a_number(alpha) or alpha <= 0:
        raise Exception("The parameter 'alpha' should be a positive float.")

    if not Utils.is_a_number(const_vel):
        raise Exception("The parameter 'const_vel' should be a float.")

    # end error handling

    return lambda p: _vector_field_vel(p, curve, alpha, const_vel, vector_size)


def _vector_field_vel(p, curve, alpha, const_vel, vector_size):
    # Error handling
    if not Utils.is_a_vector(p, vector_size):
        raise Exception("The vector field argument should be a " + str(vector_size) + " dimensional vector.")
    # end error handling

    vec_n, vec_t, min_dist = _compute_ntd(curve, p)
    fun_g = _INVHALFPI * atan(alpha * min_dist)
    fun_h = sqrt(max(1 - fun_g ** 2, 0))
    abs_const_vel = abs(const_vel)
    sgn = const_vel / (abs_const_vel + 0.00001)

    return abs_const_vel * (fun_g * vec_n + sgn * fun_h * vec_t)


def _compute_ntd(curve, p):
    min_dist = float('inf')
    ind_min = -1

    pr = np.matrix(p).reshape((3,1))

    for i in range(np.shape(curve)[1]):
        dist_temp = np.linalg.norm(pr - curve[:,i])
        if dist_temp < min_dist:
            min_dist = dist_temp
            ind_min = i

    vec_n = curve[:,ind_min] - pr
    vec_n = vec_n / (np.linalg.norm(vec_n) + 0.0001)

    if ind_min == np.shape(curve)[1] - 1:
        vec_t = curve[:,1] - curve[:,ind_min]
    else:
        vec_t = curve[:,ind_min + 1] - curve[:,ind_min]

    vec_t = vec_t / (np.linalg.norm(vec_t) + 0.0001)

    return vec_n, vec_t, min_dist
