from utils import *
import numpy as np
from graphics.meshmaterial import *


class Ball:
    """
  A ball object.

  Parameters
  ----------
  htm : 4x4 numpy array or 4x4 nested list
      The object's configuration.
      (default: the same as the current HTM).

  name : string
      The object's name.
      (default: 'genBall').

  radius : positive float
      The object's radius, in meters.
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
    def radius(self):
        """The ball radius, in meters."""
        return self._radius

    @property
    def name(self):
        """Name of the object."""
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
        """Mesh properties of the object."""
        return self._mesh_material

    @property
    def volume(self):
        """The volume of the object, in m³."""
        return self._volume

    #######################################
    # Constructor
    #######################################

    def __init__(self, htm=np.identity(4), name="", radius=1, mass=1, color="red", opacity=1,
                 mesh_material=None):

        # Error handling
        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not Utils.is_a_number(mass) or mass < 0:
            raise Exception("The parameter 'mass' should be a positive float.")

        if not Utils.is_a_number(radius) or radius < 0:
            raise Exception("The parameter 'radius' should be a positive float.")

        if name=="":
            name="var_ball_id_"+str(id(self))

        if not (Utils.is_a_name(name)):
            raise Exception(
                "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

        if not Utils.is_a_color(color):
            raise Exception("The parameter 'color' should be a HTML-compatible color.")

        if not ((mesh_material is None) or (Utils.get_uaibot_type(mesh_material) == "uaibot.MeshMaterial")):
            raise Exception(
                "The parameter 'mesh_material' should be either 'None' or a 'uaibot.MeshMaterial' object.")

        if (not Utils.is_a_number(opacity)) or opacity < 0 or opacity > 1:
            raise Exception("The parameter 'opacity' should be a float between 0 and 1.")
        # end error handling

        self._radius = radius
        self._htm = np.matrix(htm)
        self._name = name
        self._mass = 1
        self._frames = []
        self._volume = (4 / 3) * np.pi * self.radius * self.radius * self.radius
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

        string = "Ball with name '" + self.name + "': \n\n"
        string += " Radius (m): " + str(self.radius) + "\n"
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

        f = [time, np.around(htm[0,0],4), np.around(htm[0,1],4), np.around(htm[0,2],4), np.around(htm[0,3],4),
             np.around(htm[1,0],4), np.around(htm[1,1],4), np.around(htm[1,2],4), np.around(htm[1,3],4),
             np.around(htm[2,0],4), np.around(htm[2,1],4), np.around(htm[2,2],4), np.around(htm[2,3],4),
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
        string += "//BEGIN DECLARATION OF THE BALL '" + self.name + "'\n\n"
        string += self.mesh_material.gen_code(self.name) + "\n"
        string += "const var_" + self.name + " = new Ball(" + str(self.radius) + "," + str(
            self._frames) + ", material_" + self.name + ");\n"
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

        S = Utils.S(htm[0:3, 3])
        I = (2 / 5) * self.mass * (self._radius * self._radius)

        return I * np.identity(3) - self.mass * S * S

    def copy(self):
        """Return a deep copy of the object, without copying the animation frames."""
        return Ball(self.htm, self.name + "_copy", self.radius, self.mass, self.color)

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
        return 2 * self.radius, 2 * self.radius, 2 * self.radius

    #Generate samples
    def generate_samples(self, delta=0.025):

        N = round(self.radius / delta)+1
        P = np.matrix(np.zeros((3, 0)))
        for i in range(N):
            phi =  np.pi*(i/N)
            M = round(N*np.sin(phi))+1
            for j in range(M):
                theta = 2 * np.pi *(j/M)
                x = self.radius * np.sin(phi) * np.cos(theta)
                y = self.radius * np.sin(phi) * np.sin(theta)
                z = self.radius * np.cos(phi)
                P = np.block([P, np.matrix([x, y,  z]).transpose()])



        for i in range(np.shape(P)[1]):
            P[:,i] = self.htm[0:3,0:3]*P[:,i]+self.htm[0:3,-1]

        return P



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

        dd = np.linalg.norm(point - htm[0:3, 3])
        d = max(dd-self.radius,0)

        if d == 0:
            return np.matrix(point).reshape((3,1)), d
        else:
            cp = htm[0:3, 3] + self.radius * (np.matrix(point).reshape((3,1)) - htm[0:3, 3]) / dd
            return cp, d


