from utils import *
import numpy as np


class PointCloud:
    """
   A cloud of points to draw in the simulator.

   There is a fixed set of points ('points' attribute), and animation can be done by choosing, at each time,
   an interval [initial_ind, final_ind] that determines the range of points that will be displayed.

   Parameters
   ----------

   name : string
       The object's name.
       (default: 'genPointCloud').

   size : positive float
       The size of each point in the point cloud.

   color : string
       A HTML-compatible color.

   points : a 3xm numpy array or matrix
       A matrix with 3 rows. The first row is the x coordinates, the second row the y coordinates, and the third row
       the z coordinates.

   """

    #######################################
    # Attributes
    #######################################

    @property
    def name(self):
        """The object name."""
        return self._name

    @property
    def size(self):
        """The size of each point in the point cloud."""
        return self._size

    @property
    def color(self):
        """Color of the object, a HTML-compatible string."""
        return self._color

    @property
    def points(self):
        """The points that compose the point cloud."""
        return np.array(self._points)

    #######################################
    # Constructor
    #######################################

    def __init__(self, name="", points=[], size=0.1, color="blue"):

        # Error handling
        if not Utils.is_a_number(size) or size < 0:
            raise Exception("The parameter 'size' should be a positive float")

        if name=="":
            name="var_pointcloud_id_"+str(id(self))

        if not (Utils.is_a_name(name)):
            raise Exception(
                "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

        if not Utils.is_a_color(color):
            raise Exception("The parameter 'color' should be a color")

        if not Utils.is_a_matrix(points, 3):
            raise Exception("The parameter 'points' should be a matrix with 3 rows.")

        # end error handling

        self._name = name
        self._size = size
        self._points = np.matrix(points)
        self._color = color
        self._frames = []
        self._max_time = 0

        self.add_ani_frame(0, 0, np.shape(points)[1])

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "Point cloud '" + self._name + "' with " + str(len(self.points)) + " points: \n\n"
        string += " Size: " + str(self.size) + "\n"
        string += " Color: " + str(self._color) + "\n"

        return string

    #######################################
    # Methods
    #######################################

    def add_ani_frame(self, time, initial_ind, final_ind):

        if (not Utils.is_a_number(time)) or time < 0:
            raise Exception("The parameter 'time' should be a nonnegative float.")

        if (not str(type(initial_ind)) == "<class 'int'>") or initial_ind < 0:
            raise Exception("The parameter 'initial_ind' should be a nonnegative integer.")

        if (not str(type(final_ind)) == "<class 'int'>") or final_ind < 0:
            raise Exception("The parameter 'final_ind' should be a nonnegative integer.")

        if initial_ind > final_ind:
            raise Exception("The parameter 'initial_ind' should be greater or equal than the parameter 'final_ind'.")

        if final_ind > len(self.points[0]):
            raise Exception("The parameter 'final_ind' should be at most " + str(len(self.points[0])) + ".")

        # Error handling

        # end error handling

        self._frames.append([time, initial_ind, final_ind])
        self._max_time = max(self._max_time, time)

    def set_ani_frame(self, time, initial_ind, final_ind):

        if (not Utils.is_a_number(time)) or time < 0:
            raise Exception("The parameter 'time' should be a nonnegative float.")

        if (not str(type(initial_ind)) == "<class 'int'>") or initial_ind < 0:
            raise Exception("The parameter 'initial_ind' should be a nonnegative integer.")

        if (not str(type(final_ind)) == "<class 'int'>") or final_ind < 0:
            raise Exception("The parameter 'final_ind' should be a nonnegative integer.")

        if initial_ind > final_ind:
            raise Exception("The parameter 'initial_ind' should be greater or equal than the parameter 'final_ind'.")

        if final_ind > len(self.points[0]):
            raise Exception("The parameter 'final_ind' should be at most " + str(len(self.points[0])) + ".")

        # Error handling

        # end error handling

        self._frames = []
        self.add_ani_frame(0, initial_ind, final_ind)
        self._max_time = 0

    def gen_code(self):
        """Generate code for injection."""

        string = "\n"
        string += "//BEGIN DECLARATION OF THE POINT CLOUD '" + self.name + "'\n\n"
        string += "const var_" + self.name + " = new PointCloud(" + str(np.around(np.matrix(self.points),4).tolist()) + ", " + str(
            self._frames) + ", '" + self.color + "', " + str(
            self.size) + ");\n"
        string += "sceneElements.push(var_" + self.name + ");\n"
        string += "//USER INPUT GOES HERE"

        return string
