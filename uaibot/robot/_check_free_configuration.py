from utils import *
import numpy as np
from ._dist_struct_robot_auto import DistStructRobotAuto


# Compute the distance from each non-sequential link to other link
def _check_free_configuration(self, q=None, old_dist_struct=None, tol=0.0005, dist_tol=0.005, no_iter_max=20):
    n = len(self.links)

    if q is None:
        q = self.q


    # Error handling
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not Utils.is_a_number(tol) or tol <= 0:
        raise Exception("The parameter 'tol' should be a positive number.")

    if not Utils.is_a_number(dist_tol) or dist_tol <= 0:
        raise Exception("The parameter 'dist_tol' should be a positive number.")

    if not Utils.is_a_natural_number(no_iter_max) or no_iter_max <= 0:
        raise Exception("The parameter 'no_iter_max' should be a positive natural number.")

    if not (old_dist_struct is None):
        try:
            if not id(old_dist_struct.robot) == id(self):
                Exception("The parameter 'old_dist_struct' is a '_DistStructRobotAuto' object, but it " \
                          "must have to be relative to the SAME robot object, and " \
                          "this is not the case.")
        except:
            raise Exception("The parameter 'old_dist_struct' must be a '_DistStructRobotAuto' object.")

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


    #Create the list of all possible collision checks
    list_col_check = []
    for i in range(n):
        for j in range(i+2,n):
            for isub in range(len(self.links[i].col_objects)):
                for jsub in range(len(self.links[j].col_objects)):
                    list_col_check.append([i,isub,j,jsub])

    #Check if there is collision
    collided = False
    ended = False
    message = "Ok!"
    info = []

    while (not collided and not ended):
        #Check joint limits:
        k=0
        while (not collided and k < n):
            collided = self.q[k] < self.joint_limit[k,0]
            if collided:
                message="Joint number "+str(k)+" is below minimum limit."
                info = [0,k]

            k+=1

        k = 0
        while (not collided and k < n):
            collided = self.q[k] > self.joint_limit[k, 1]
            if collided:
                message = "Joint number " + str(k) + " is above maximum limit."
                info = [1,k]

            k += 1

        # Check auto collision:
        k = 0
        while (not collided and k < len(list_col_check)) :

            i = list_col_check[k][0]
            isub = list_col_check[k][1]
            j = list_col_check[k][2]
            jsub = list_col_check[k][3]

            if old_dist_struct is None:
                p_obj_0 = np.matrix(np.random.uniform(-100, 100, size=(3, 1)))
            else:
                p_obj_0 = old_dist_struct.get_item(i, isub, j, jsub).point_object

            p_obj_i, p_obj_j, d = Utils.compute_dist(col_object_copy[i][isub], col_object_copy[j][jsub] \
                                                     , p_obj_0, tol, no_iter_max)

            if d<dist_tol:
                collided = True
                message = "Collision between link "+str(i)+" (col object "+str(isub)+") and link "+str(j)+" " \
                            "(col object "+str(jsub)+")."
                info = [2,i,isub,j,jsub]

            k+=1
            ended = (k==len(list_col_check))

    return (not collided), message, info
