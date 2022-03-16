from utils import *


# Set config. Restart animation queue
def _set_ani_frame(self, q=None, htm=None):
    if q is None:
        q = self.q0

    if htm is None:
        htm = self.htm

    # Error handling
    if not Utils.is_a_vector(q, len(self.links)):
        raise Exception("The parameter 'q' should be a " + str(n) + " dimensional vector.")

    if not Utils.is_a_matrix(htm, 4, 4):
        raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")
    # end error handling

    self._frames = []
    self.code = ''
    self._max_time = 0
    self.add_ani_frame(0, q, htm)
