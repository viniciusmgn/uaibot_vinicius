from utils import *
import numpy as np
from ._dist_struct_robot_obj import _DistStructRobotObj


# Compute the distance from each link to an object, for the current configuration
# of the robot
def _compute_dist(self, obj, h, g, q=None, htm=None, old_dist_struct=None, tol=0.0005, no_iter_max=20):
    n = len(self.links)

    if q is None:
        q = self.q

    if htm is None:
        htm = self.htm

    # Error handling
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

    if not Utils.is_a_simple_object(obj):
        raise Exception("The parameter 'obj' should be one of the following types: " + str(Utils.IS_SIMPLE) + ".")

    if not Utils.is_a_number(h) or h <= 0:
        raise Exception("The parameter 'h' should be a positive number.")

    if not Utils.is_a_number(g) or g <= 0:
        raise Exception("The parameter 'g' should be a positive number.")

    if not Utils.is_a_number(tol) or tol <= 0:
        raise Exception("The parameter 'tol' should be a positive number.")

    if not Utils.is_a_natural_number(no_iter_max) or no_iter_max <= 0:
        raise Exception("The parameter 'no_iter_max' should be a positive natural number.")

    if not (old_dist_struct is None):
        try:
            if not (id(old_dist_struct.obj) == id(obj) and id(old_dist_struct.robot) == id(self)):
                Exception("The parameter 'old_dist_struct' is a '_DistStructRobotObj' object, but it " \
                          "must have to be relative to the SAME robot object and SAME external object, and " \
                          "this is not the case.")
        except:
            raise Exception("The parameter 'old_dist_struct' must be a '_DistStructRobotObj' object.")

    # end error handling

    dist_struct = _DistStructRobotObj(obj, self)

    jac_dh, mth_dh = self.jac_geo(q, "dh", htm)

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
        for j in range(len(self.links[i].col_objects)):

            if old_dist_struct is None:
                p_obj_0 = np.matrix(np.random.uniform(-100, 100, size=(3,1)))
            else:
                p_obj_0 = old_dist_struct.get_item(i, j).point_object

            p_obj, p_obj_col, d = Utils.compute_dist(obj, col_object_copy[i][j], h, g, p_obj_0, tol, no_iter_max)

            jac_obj_col = jac_dh[i][0:3, :] - Utils.S(p_obj_col - mth_dh[i][0:3, 3]) * jac_dh[i][3:6, :]
            jac_dist = ((p_obj_col - p_obj).T * jac_obj_col) / d

            dist_struct._append(i, j, d, p_obj_col, p_obj, jac_dist)

    return dist_struct
