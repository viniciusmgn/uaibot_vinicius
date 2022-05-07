from utils import *
import numpy as np
from graphics.meshmaterial import *


class Box:
    """
  A box object.

  Parameters
  ----------
  htm : 4x4 numpy array or 4x4 nested list
      The object's configuration
      (default: the same as the current HTM).

  name : string
      The object's name.
      (default: 'genBox').

  width : positive float
      The object's width, in meters.
      (default: 1).    

  depth : positive float
      The object's depth, in meters.
      (default: 1).  

  height : positive float
      The object's height, in meters.
      (default: 1).  

  mass : positive float
      The object's mass, in kg.
      (default: 1).  

  color : string
      The object's color, a HTML - compatible string.
      (default: "red").

  opacity : float between 0 and 1
      The opacity. 1 = fully opaque, and 0 = transparent.

  mesh_material: 'MeshMaterial' object
      The object mesh material. If set to 'None', the default is used.
      (default: None).
  """

    #######################################
    # Attributes
    #######################################

    @property
    def width(self):
        """The box width, in meters."""
        return self._width

    @property
    def height(self):
        """The box height, in meters."""
        return self._height

    @property
    def depth(self):
        """The box depth, in meters."""
        return self._depth

    @property
    def name(self):
        """The object name."""
        return self._name

    @property
    def htm(self):
        """Object pose. A 4x4 homogeneous transformation matrix written is scenario coordinates."""
        return np.matrix(self._htm)

    @property
    def mass(self):
        """Mass of the object, in kg."""
        return self._mass

    @property
    def color(self):
        """Color of the object"""
        return self.mesh_material.color

    @property
    def mesh_material(self):
        """Mesh properties of the object"""
        return self._mesh_material

    @property
    def volume(self):
        """The volume of the object, in m³."""
        return self._volume

    #######################################
    # Constructor
    #######################################

    def __init__(self, htm=np.identity(4), name="genBox", width=1, height=1, depth=1, mass=1, color="red", opacity=1, \
                 mesh_material=None):

        # Error handling
        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not Utils.is_a_number(mass) or mass < 0:
            raise Exception("The parameter 'mass' should be a positive float.")

        if not Utils.is_a_number(width) or width < 0:
            raise Exception("The parameter 'width' should be a positive float.")

        if not Utils.is_a_number(height) or height < 0:
            raise Exception("The parameter 'height' should be a positive float.")

        if not Utils.is_a_number(depth) or depth < 0:
            raise Exception("The parameter 'depth' should be a positive float.")

        if not (Utils.is_a_name(name)):
            raise Exception(
                "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

        if not Utils.is_a_color(color):
            raise Exception("The parameter 'color' should be a HTML-compatible color.")

        if not ((mesh_material is None) or (Utils.get_uaibot_type(mesh_material) == "uaibot.MeshMaterial")):
            raise Exception(
                "The parameter 'mesh_material' should be either 'None' or a 'uaibot.MeshMaterial' object.")

        if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
            raise Exception("The parameter 'opacity' should be a float between 0 and 1")
            # end error handling

        self._width = width
        self._height = height
        self._depth = depth
        self._htm = np.matrix(htm)
        self._name = name
        self._mass = 1
        self._frames = []
        self._volume = self.width * self.depth * self.height
        self._max_time = 0

        if mesh_material is None:
            self._mesh_material = MeshMaterial(color=color, opacity=opacity)
        else:
            self._mesh_material = mesh_material

        # Set initial total configuration
        self.set_ani_frame(self._htm)

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "Box with name '" + self.name + "': \n\n"
        string += " Width (m): " + str(self.width) + "\n"
        string += " Depth (m): " + str(self.depth) + "\n"
        string += " Height (m): " + str(self.height) + "\n"
        string += " Color: " + str(self.color) + "\n"
        string += " Mass (kg): " + str(self.mass) + "\n"
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

        f = [time, np.around(htm[0,0],4), np.around(htm[0,2],4), np.around(-htm[0,1],4), np.around(htm[0,3],4),
             np.around(htm[1,0],4), np.around(htm[1,2],4), np.around(-htm[1,1],4), np.around(htm[1,3],4),
             np.around(htm[2,0],4), np.around(htm[2,2],4), np.around(-htm[2,1],4), np.around(htm[2,3],4),
             0, 0, 0, 1]

        self._htm = htm
        self._frames.append(f)
        self._max_time = max(self._max_time, time)

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
        string += "//BEGIN DECLARATION OF THE BOX '" + self.name + "'\n\n"
        string += self.mesh_material.gen_code(self.name) + "\n"
        string += "const var_" + self.name + " = new Box(" + str(self.width) + "," + str(
            self.height) + "," + str(self.depth) + "," + str(self._frames) + ", material_" + self.name + ");\n"
        string += "sceneElements.push(var_" + self.name + ");\n"
        string += "//USER INPUT GOES HERE"
        return string

    # Compute inertia matrix with respect to the inertia frame
    def inertia_matrix(self, htm=None):
        """
    The 3D inertia matrix of the object, written in the world frame.
    Assume that the transformation between the word frame and the object frame is 'htm'.

    Parameters
    ----------
    htm : 4x4 numpy array or 4x4 nested list
        The object's configuration for which the inertia matrix will be computed
        (default: the same as the current HTM).

    Returns
    -------
     inertia_matrix : 3x3 numpy array
        The 3D inertia matrix.
    """

        if htm is None:
            htm = self._htm

        # Error handling
        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The optional parameter 'htm' should be a 4x4 homogeneous transformation matrix.")
        # end error handling

        Ixx = (1 / 12) * self.mass * (self.height * self.height + self.depth * self.depth)
        Iyy = (1 / 12) * self.mass * (self.width * self.width + self.depth * self.depth)
        Izz = (1 / 12) * self.mass * (self.height * self.height + self.width * self.width)
        Q = htm[0:3, 0:3]
        S = Utils.S(htm[0:3, 3])

        return Q * np.diag([Ixx, Iyy, Izz]) * Q.T - self.mass * S * S

    def copy(self):
        """Return a deep copy of the object, without copying the animation frames."""
        return Box(self.htm, self.name + "_copy", self.width, self.height, self.depth, self.mass, self.color)

    # Compute the h projection of a point into an object
    def h_projection(self, point, h=0.001, htm=None):
        """
    The h projection of a point in the object, that is, the 
    h-closest point in the object to a point 'point'.

    Parameters
    ----------
    point : 3D vector
        The point for which the projection will be computed.

    h : positive float
        Smoothing parameter, in meters
        (defalt: 0.001 m)

    htm : 4x4 numpy array or 4x4 nested list
        The object's configuration
        (default: the same as the current HTM).            

    Returns
    -------
     proj_point : 3D vector
        The h-projection of the point 'point' in the object.

     d : positive float
        The h-distance between the object and 'point'.     
    """

        if htm is None:
            htm = self._htm

        # Error handling
        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The optional parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not Utils.is_a_vector(point, 3):
            raise Exception("The parameter 'point' should be a 3D vector.")

        if not Utils.is_a_number(h) or h <= 0:
            raise Exception("The optional parameter 'h' should be a positive number.")
        # end error handling
        tpoint = htm[0:3, 0:3].T * (point - htm[0:3, 3])

        delta = 0.001

        dxf = Utils.fun_Int(tpoint[0,0] + delta, h, 0.5 * self.width)
        dxb = Utils.fun_Int(tpoint[0,0] - delta, h, 0.5 * self.width)

        dyf = Utils.fun_Int(tpoint[1,0] + delta, h, 0.5 * self.depth)
        dyb = Utils.fun_Int(tpoint[1,0] - delta, h, 0.5 * self.depth)

        dzf = Utils.fun_Int(tpoint[2,0] + delta, h, 0.5 * self.height)
        dzb = Utils.fun_Int(tpoint[2,0] - delta, h, 0.5 * self.height)

        d = 0.5 * (dxf + dxb + dyf + dyb + dzf + dzb)
        x = tpoint[0,0] - (dxf - dxb) / (2 * delta)
        y = tpoint[1,0] - (dyf - dyb) / (2 * delta)
        z = tpoint[2,0] - (dzf - dzb) / (2 * delta)

        return htm[0:3, 0:3] * np.matrix([[x], [y], [z]]) + htm[0:3, 3], d
