from utils import *
import numpy as np
from ._dist_struct_robot_auto import DistStructRobotAuto


# Compute the distance from each non-sequential link to other link
def _compute_dist_auto(self, q=None, old_dist_struct=None, tol=0.0005, no_iter_max=20, max_dist = np.inf):
    n = len(self.links)

    if q is None:
        q = self.q

    # Error handling
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not Utils.is_a_number(tol) or tol <= 0:
        raise Exception("The parameter 'tol' should be a positive number.")

    if not Utils.is_a_number(max_dist) or tol <= 0:
        raise Exception("The parameter 'max_dist' should be a positive number, or np.inf.")

    if not Utils.is_a_natural_number(no_iter_max) or no_iter_max <= 0:
        raise Exception("The parameter 'no_iter_max' should be a positive natural number.")

    if not (old_dist_struct is None):
        try:
            if not id(old_dist_struct.robot) == id(self):
                Exception("The parameter 'old_dist_struct' is a 'DistStructRobotAuto' object, but it " \
                          "must have to be relative to the SAME robot object, and " \
                          "this is not the case.")
        except:
            raise Exception("The parameter 'old_dist_struct' must be a 'DistStructRobotAuto' object.")

    # end error handling

    dist_struct = DistStructRobotAuto(self)

    jac_dh, mth_dh = self.jac_geo(q, "dh")

    col_object_copy = []

    # Update all collision objects of all links
    for i in range(n):
        col_object_copy.append([])
        for j in range(len(self.links[i].col_objects)):
            temp_copy = self.links[i].col_objects[j][0].copy()
            htmd = self.links[i].col_objects[j][1]
            temp_copy.set_ani_frame(mth_dh[i][:, :] * htmd)
            col_object_copy[i].append(temp_copy)

    # Compute the distance structure
    for i in range(n):
        for j in range(i+2,n):
            for isub in range(len(self.links[i].col_objects)):
                for jsub in range(len(self.links[j].col_objects)):

                    est_dist = 0 if math.isinf(max_dist) else Utils.compute_aabbdist(col_object_copy[i][isub], col_object_copy[j][jsub])

                    if est_dist <= max_dist:

                        if old_dist_struct is None:
                            p_obj_0 = np.matrix(np.random.uniform(-100, 100, size=(3, 1)))
                        else:
                            p_obj_0 = old_dist_struct.get_item(i, isub, j, jsub).point_object

                        p_obj_i, p_obj_j, d = Utils.compute_dist(col_object_copy[i][isub], col_object_copy[j][jsub] \
                                                                 , p_obj_0, tol, no_iter_max)

                        jac_obj_i = jac_dh[i][0:3, :] - Utils.S(p_obj_i - mth_dh[i][0:3, 3]) * jac_dh[i][3:6, :]
                        jac_obj_j = jac_dh[j][0:3, :] - Utils.S(p_obj_j - mth_dh[j][0:3, 3]) * jac_dh[j][3:6, :]

                        jac_dist = (p_obj_i - p_obj_j).T * (jac_obj_i - jac_obj_j) / d
                        dist_struct._append(i, isub, j, jsub, d, p_obj_i, p_obj_j, jac_dist)



    return dist_struct
