from utils import *
import numpy as np

def _add_ani_frame(self, time, q=None, htm=None, enforce_joint_limits=False):

    n = len(self.links)

    if q is None:
        q = self.q

    if htm is None:
        htm = self.htm

    # Error handling
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not Utils.is_a_number(time) or time < 0:
        raise Exception("The parameter 'time' should be a nonnegative float.")

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix")

    if not str(type(enforce_joint_limits)) == "<class 'bool'>":
        raise Exception("The parameter 'enforce_joint_limits' must be a boolean.")

    # end error handling
    self._q = np.matrix(q).reshape((n, 1))

    if enforce_joint_limits:
        for i in range(len(self.links)):
            self._q[i,0] = min(max(self._q[i,0],self.joint_limit[i,0]),self.joint_limit[i,1])

    f = [time, htm[0,0], htm[0,1], htm[0,2], htm[0,3],
         htm[1,0], htm[1,1], htm[1,2], htm[1,3],
         htm[2,0], htm[2,1], htm[2,2], htm[2,3],
         0, 0, 0, 1, np.array(q).reshape((n,)).tolist()]

    self._htm = htm
    self._frames.append(f)
    self._max_time = max(self._max_time, time)

    # Update attached objects:
    if len(self.attached_objects) > 0:
        htm = self.fkm(q, 'eef', htm)
        for obj in self.attached_objects:
            obj[0].add_ani_frame(time, htm @ obj[1])
