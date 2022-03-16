from utils import *
import numpy as np


# Add config to animation queue
def _add_ani_frame(self, time, q=None, htm=None):
    if q is None:
        q = self.q

    if htm is None:
        htm = self.htm

    n = len(self.links)

    # Error handling
    if not Utils.is_a_vector(q, n):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not Utils.is_a_number(time) or time < 0:
        raise Exception("The parameter 'time' should be a nonnegative float.")

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix")
    # end error handling

    self._q = np.array(q).reshape((n, 1))

    f = [time, htm[0][0], htm[0][1], htm[0][2], htm[0][3],
         htm[1][0], htm[1][1], htm[1][2], htm[1][3],
         htm[2][0], htm[2][1], htm[2][2], htm[2][3],
         0, 0, 0, 1, np.ndarray.tolist(np.array(q))]

    self._htm = htm
    self._frames.append(f)
    self._max_time = max(self._max_time, time)

    # Update attached objects:
    if len(self.attached_objects) > 0:
        htm = self.fkm(q, 'eef', htm)
        for obj in self.attached_objects:
            obj[0].add_ani_frame(htm @ obj[1])
