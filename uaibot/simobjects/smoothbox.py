from utils import *
import numpy as np
from graphics.meshmaterial import *


class SmoothBox:
    """
  A smooth box object. Smooth boxes satisfy the parametric equation (in their own aligned axis)

  (x/width)^4+(y/depth)^4+(z/height)^4 = 1

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

    def __init__(self, htm=np.identity(4), name="", width=1, height=1, depth=1, mass=1, color="red", opacity=1, \
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

        if name=="":
            name="var_smoothbox_id_"+str(id(self))

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

        string = "SmoothBox with name '" + self.name + "': \n\n"
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
        string += "//BEGIN DECLARATION OF THE SMOOTH BOX '" + self.name + "'\n\n"
        string += self.mesh_material.gen_code(self.name) + "\n"
        string += "const var_" + self.name + " = new SmoothBox(" + str(self.width) + "," + str(
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

        Ixx = 0.05486 * self.mass * (self.height * self.height + self.depth * self.depth)
        Iyy = 0.05486 * self.mass * (self.width * self.width + self.depth * self.depth)
        Izz = 0.05486 * self.mass * (self.height * self.height + self.width * self.width)
        Q = htm[0:3, 0:3]
        S = Utils.S(htm[0:3, 3])

        return Q * np.diag([Ixx, Iyy, Izz]) * Q.T - self.mass * S * S

    def copy(self):
        """Return a deep copy of the object, without copying the animation frames."""
        return SmoothBox(self.htm, self.name + "_copy", self.width, self.height, self.depth, self.mass, self.color)

    def aabb(self):
        """
    Compute the width, depth and height of an axis aligned bounding box (aabb) that
    covers the object. It also considers the current orientation.

    Returns
    -------
     width : positive float
        The width of the box, in meters.

     depth : positive float
        The depth of the box, in meters.

     height : positive float
        The depth of the box, in meters.
    """

        p1 = self.width * self.htm[:,0] + self.depth * self.htm[:,1] + self.height * self.htm[:,2]
        p2 = -self.width * self.htm[:, 0] + self.depth * self.htm[:, 1] + self.height * self.htm[:, 2]
        p3 = self.width * self.htm[:, 0] - self.depth * self.htm[:, 1] + self.height * self.htm[:, 2]
        p4 = self.width * self.htm[:, 0] + self.depth * self.htm[:, 1] - self.height * self.htm[:, 2]

        w = np.max([abs(p1[0, 0]), abs(p2[0, 0]), abs(p3[0, 0]), abs(p4[0, 0])])
        d = np.max([abs(p1[1, 0]), abs(p2[1, 0]), abs(p3[1, 0]), abs(p4[1, 0])])
        h = np.max([abs(p1[2, 0]), abs(p2[2, 0]), abs(p3[2, 0]), abs(p4[2, 0])])

        return w, d, h

    # Compute the projection of a point into an object
    def projection(self, point, htm=None):
        """
    The projection of a point in the object, that is, the
    closest point in the object to a point 'point'.

    Parameters
    ----------
    point : 3D vector
        The point for which the projection will be computed.

    htm : 4x4 numpy array or 4x4 nested list
        The object's configuration
        (default: the same as the current HTM).

    Returns
    -------
     proj_point : 3D vector
        The projection of the point 'point' in the object.

     d : positive float
        The distance between the object and 'point'.
    """

        if htm is None:
            htm = self._htm

        # Error handling
        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The optional parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not Utils.is_a_vector(point, 3):
            raise Exception("The parameter 'point' should be a 3D vector.")

        # end error handling
        tpoint = htm[0:3, 0:3].T * (point - htm[0:3, 3])

        A = self.width/2
        B = self.depth/2
        C = self.height/2

        d4 = (tpoint[0, 0]/A)**4 + (tpoint[1, 0]/B)**4 + (tpoint[2, 0]/C)**4

        if d4 <= 1:
            return point, 0
        else:

            fun = lambda lam: (Utils.cubicsolve(tpoint[0, 0], lam / (A ** 4)) / A) ** 4 + \
                              (Utils.cubicsolve(tpoint[1, 0], lam / (B ** 4)) / B) ** 4 + \
                              (Utils.cubicsolve(tpoint[2, 0], lam / (C ** 4)) / C) ** 4 - 1

            maxR = max(A, B, C)
            minR = min(A, B, C)
            dp = np.linalg.norm(tpoint)

            lam0 = 0.00001
            lamf = maxR*(dp-minR)/0.75

            lam_star = Utils.bissection(fun,lam0,lamf,0.0001)

            x = Utils.cubicsolve(tpoint[0, 0], lam_star / (A ** 4))
            y = Utils.cubicsolve(tpoint[1, 0], lam_star / (B ** 4))
            z = Utils.cubicsolve(tpoint[2, 0], lam_star / (C ** 4))

            d = sqrt( (x-tpoint[0,0])**2 + (y-tpoint[1,0])**2 + (z-tpoint[2,0])**2 )

            return htm[0:3, 0:3] * np.matrix([[x], [y], [z]]) + htm[0:3, 3], d
