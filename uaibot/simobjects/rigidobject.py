from utils import *
import numpy as np
from graphics.model3d import *


class RigidObject:
    """
   A rigid object.

   Parameters
   ----------

   name : string
       The object's name.
       (default: 'genRigidObject').

   model_3d : 'Model3D' object
       The 3d model of the object.

   htm : 4x4 numpy array or 4x4 nested list
       The object's configuration.
       (default: the same as the current HTM).

   """

    #######################################
    # Attributes
    #######################################

    @property
    def name(self):
        """Name of the object."""
        return self._name

    @property
    def htm(self):
        """Object pose. A 4x4 homogeneous transformation matrix written is scenario coordinates."""
        return np.array(self._htm)

    @property
    def model_3d(self):
        """The 3d model of the object."""
        return self._model_3d

    #######################################
    # Constructor
    #######################################

    def __init__(self, model_3d, name="genRigidObject", htm=np.identity(4)):

        # Error handling
        if not (Utils.is_a_name(name)):
            raise Exception(
                "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. "
                "It should not begin with a number.")

        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not Utils.get_uaibot_type(model_3d) == "uaibot.Model3D":
            raise Exception("The parameter 'model_3d' should a 'uaibot.Model3D' object.")
        # end error handling

        self._htm = np.array(htm)
        self._name = name
        self._model_3d = model_3d
        self._max_time = 0

        self.set_ani_frame(self._htm)

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "Rigid Object with name '" + self.name + "': \n\n"
        string += " HTM: \n" + str(self.htm) + "\n"

        return string

    #######################################
    # Methods
    #######################################

    def add_ani_frame(self, time, htm=None):
        """
    Add a single configuration to the object's animation queue.

    Parameters
    ----------
    time: positive float
        The timestamp of the animation frame, in seconds.
    htm : 4x4 numpy array or 4x4 nested list
        The object's configuration
        (default: the same as the current HTM).

    Returns
    -------
    None
    """
        if htm is None:
            htm = self._htm

        # Error handling
        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not Utils.is_a_number(time) or time < 0:
            raise Exception("The parameter 'time' should be a positive float.")
        # end error handling

        f = [time, htm[0][0], htm[0][1], htm[0][2], htm[0][3],
             htm[1][0], htm[1][1], htm[1][2], htm[1][3],
             htm[2][0], htm[2][1], htm[2][2], htm[2][3],
             0, 0, 0, 1]

        self._htm = htm
        self._frames.append(f)
        self._max_time = max(self._max_time, time)

    # Set config. Restart animation queue
    def set_ani_frame(self, htm=None):
        """
    Reset object's animation queue and add a single configuration to the
    object's animation queue.

    Parameters
    ----------
    htm : 4x4 numpy array or 4x4 nested list
        The object's configuration
        (default: the same as the current HTM).

    Returns
    -------
    None
    """

        if htm is None:
            htm = self._htm

        # Error handling
        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        # end error handling

        self._frames = []
        self.add_ani_frame(0, htm)
        self._max_time = 0

    def gen_code(self):
        """Generate code for injection."""

        string = "\n"
        string += "//BEGIN DECLARATION OF THE RIGID OBJECT '" + self.name + "'\n\n"
        string += self.model_3d.gen_code(self.name) + "\n"
        string += "const var_" + self.name + " = new RigidObject(" + str(
            self._frames) + ", object3d_" + self.name + ");\n"
        string += "sceneElements.push(var_" + self.name + ");\n"
        string += "//USER INPUT GOES HERE"

        return string
