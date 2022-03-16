import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import numpy as np
from utils import *
from graphics.meshmaterial import *

from ._set_ani_frame import _set_ani_frame
from ._add_ani_frame import _add_ani_frame

from ._ikm import _ikm
from ._fkm import _fkm
from ._jac_geo import _jac_geo
from ._jac_jac_geo import _jac_jac_geo

from ._dyn_model import _dyn_model

from ._vector_field import _vector_field
from ._task_function import _task_function
from ._coop_task_function import _coop_task_function

from ._gen_code import _gen_code
from ._update_col_object import _update_col_object
from ._add_col_object import _add_col_object
from ._attach_object import _attach_object
from ._detach_object import _detach_object

from ._compute_dist import _compute_dist

from ._create_kuka_kr5 import _create_kuka_kr5
from ._create_epson_t6 import _create_epson_t6
from ._create_staubli_tx60 import _create_staubli_tx60
from ._create_kuka_lbr_iiwa import _create_kuka_lbr_iiwa

class Robot:
    """
  A class that contains a robot object in jupyterbot.

  Parameters
  ----------

  name : string
      The robot name.
      (default: 'genRobot').

  htm_base_0 : 4x4 numpy array or 4x4 nested list
      The robot base's configuration.
      (default: 4x4 identity matrix).

  list_base_3d_obj : list of 'jupyterbot.Model3D' objects
      The list of 3d models of the base of the robot.
      If set to None, there is no base 3d object.
      (default: None).

  links : A list of 'jupyterbot.Link' objects
      The list of link objects.

  q0 : nd numpy vector or array
      The robot initial configuration.
      (default: zero vector).
  """

    #######################################
    # Attributes
    #######################################

    @property
    def q(self):
        """The current joint configuration."""
        return np.array(self._q)

    @property
    def q0(self):
        """The default joint configuration."""
        return np.array(self._q0)

    @property
    def htm(self):
        """
        The current base configuration in scenario coordinates.
        A 4x4 homogeneous matrix written is scenario coordinates.
        """
        return np.array(self._htm)

    @property
    def htm_base_0(self):
        """
        The constant homogeneous transformation between the HTM of the base and
        the HTM of the first Denavit-Hartenberg frame.
        """
        return np.array(self._htm_base_0)

    @property
    def links(self):
        """Data structures containing the links of the robot."""
        return self._links

    @property
    def attached_objects(self):
        """Data structures containing the objects attached into the robot."""
        return self._attached_objects

    @property
    def name(self):
        """Name of the object."""
        return self._name

    @property
    def list_object_3d_base(self):
        """The list of 3d objects that form the base."""
        return self._list_object_3d_base

    #######################################
    # Constructor
    #######################################

    def __init__(self, name, links, list_base_3d_obj=None, htm=np.identity(4), htm_base_0=np.identity(4), q0=None):
        # Error handling

        if not (Utils.is_a_name(name)):
            raise Exception(
                "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. It should not begin with a number.")

        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not (str(type(links)) == "<class 'list'>"):
            raise Exception("The parameter 'links' should be a list of 'jupyterbot.Link' objects.")
        else:
            for link in links:
                if not Utils.get_jupyterbot_type(link) == "jupyterbot.Link":
                    raise Exception("The parameter 'links' should be a list of 'jupyterbot.Link' objects.")

        n = len(links)

        if not (str(type(list_base_3d_obj)) == "<class 'list'>" or (list_base_3d_obj is None)):
            raise Exception("The parameter 'list_base_3d_obj' should be a list of 'jupyterbot.Model3D' objects.")
        else:
            for i in range(len(list_base_3d_obj)):
                if not (Utils.get_jupyterbot_type(list_base_3d_obj[i]) == "jupyterbot.Model3D"):
                    raise Exception(
                        "The parameter 'list_base_3d_obj' should be a list of 'jupyterbot.Model3D' objects.")

        if not Utils.is_a_vector(q0, n):
            raise Exception("The parameter 'q0' should be a " + str(n) + " dimensional vector.")

        # end error handling

        self._frames = []
        self._list_object_3d_base = list_base_3d_obj
        self._htm = np.array(htm)
        self._name = name
        self._attached_objects = []
        self._links = links
        self._htm_base_0 = htm_base_0
        self._max_time = 0

        if not (q0 is None):
            self._q0 = q0
        else:
            self._q0 = np.zeros((n,))

        # Set initial total configuration
        self.set_ani_frame(self._q0, self._htm)

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):
        n = len(self.links)

        string = "Robot with name '" + self.name + "': \n\n"
        string += " Number of joints: " + str(n) + "\n"
        string += " Joint types: "

        for i in range(n):
            string += "R" if self._links[i].joint_type == 0 else "P"

        string += "\n"
        string += " Current configuration: " + str([round(num, 3) for num in np.ndarray.tolist(self.q)]) + "\n"
        string += " Current base HTM: \n" + str(self.htm) + "\n"
        string += " Current end-effector HTM: \n" + str(self.fkm())
        return string

    #######################################
    # Methods for configuration changing
    #######################################

    def add_ani_frame(self, time, q=None, htm=None):
        """
    Add a single configuration to the object's animation queue.

    Parameters
    ----------
    time: positive float
        The timestamp of the animation frame, in seconds.
    q : nd numpy vector or array
        The manipulator's joint configuration.
    htm : 4x4 numpy array or 4x4 nested list
        The robot base's configuration.
        (default: the same as the current HTM).

    Returns
    -------
    None
    """
        return _add_ani_frame(self, time, q, htm)

    def set_ani_frame(self, q=None, htm=None):
        """
    Reset object's animation queue and add a single configuration to the 
    object's animation queue.

    Parameters
    ----------
    q : nd numpy vector or array
        The manipulator's joint configuration .
        (default: the default joint configuration for the manipulator, q0).
    htm : 4x4 numpy array or 4x4 nested list
        The robot base's configuration.
        (default: the same as the current HTM).

    Returns
    -------
    None
    """
        return _set_ani_frame(self, q, htm)

    #######################################
    # Methods for kinematics model
    #######################################

    def fkm(self, q=None, axis='eef', htm=None):
        """
    Compute the forward kinematics for an axis at a given joint and base
    configuration. Everything is written in the scenario coordinates.

    Parameters
    ----------
    q : nd numpy vector or array
        The manipulator's joint configuration.
        (default: the default joint configuration for the manipulator).
    axis : string
        For which axis you want to compute the FK:
        'eef': for the end-effector;
        'dh': for all Denavit-Hartenberg axis;
        'com': for all center-of-mass axis.
        (default: 'eef').    
    htm : 4x4 numpy array or 4x4 nested list
        The robot base's configuration.
        (default: the same as the current HTM).

    Returns
    -------
    htm_fk : 4x4 or nx4x4 numpy matrix
        For axis='eef', returns a single htm. For the other cases, return
        n htms as a nx4x4 numpy matrix.
    """
        return _fkm(self, q, axis, htm)

    def ikm(self, htm_target, q0=None, p_tol=0.005, a_tol=5, no_iter_max=2000, ignore_orientation=False):
        """
    Try to solve the inverse kinematic problem for the end-effector, given a
    desired homogeneous transformation matrix. It returns the manipulator
    configuration.

    Important: it disregards the current htm of the base of the robot. That is,
    it assumes that robot.htm = np.identity(4). You can easily consider other
    cases by transforming htm_target as Utils.inv_htm(robot.htm) @ htm_target.

    Uses an iterative algorithm.

    The algorithm can fail, throwing an Exception when it happens.

    Parameters
    ----------
    htm_target : 4x4 numpy array or 4x4 nested list
        The target end-effector HTM, written in scenario coordinates.
    q0 : n-dimensional numpy vector or array
        Initial guess for the algorithm for the joint configuration.
        (default: a random joint configuration).
    p_tol : positive float
        The accepted error for the end-effector position, in meters.
        (default: 0.005 m).    
    a_tol : positive float
        The accepted error for the end-effector orientation, in degrees.
        (default: 5 degrees). 
    no_iter_max : positive int
        The maximum number of iterations for the algorithm.
        (default: 2000 iterations).
    ignore_orientation: boolean
        If True, the orientation part of the HTM is ignored. Task is position-only.
        (default: False)

    Returns
    -------
    q : n-dimensional numpy column vector
        The configuration that solves the IK problem.
    """
        return _ikm(self, htm_target, q0, p_tol, a_tol, no_iter_max, ignore_orientation)

    def jac_geo(self, q=None, axis='eef', htm=None):
        """
    Compute the geometric Jacobian for an axis at a given joint and base
    configuration. Also returns the forward kinematics as a by-product.
    Everything is written in the scenario coordinates.

    Parameters
    ----------
    q : nd numpy vector or array
        The manipulator's joint configuration 
        (default: the default joint configuration for the manipulator).
    axis : string
        For which axis you want to compute the FK:
        'eef': for the end-effector;
        'dh': for all Denavit-Hartenberg axis;
        'com': for all center-of-mass axis.
        (default: 'eef').    
    htm : 4x4 numpy array or 4x4 nested list
        The robot base's configuration.
        (default: the same as the current htm).

    Returns
    -------
    jac_geo : 6 x n or n x 6 x n numpy matrix
        For axis='eef', returns a single 6xn Jacobian. For the other cases, 
        return n Jacobians as a nx6xn numpy matrix.

    htm_out : 4 x 4 or n x 4 x 4 numpy matrix
        For axis='eef', returns a single htm. For the other cases, return
        n htms as a n x 4 x 4 numpy matrix.
    """
        return _jac_geo(self, q, axis, htm)

    def jac_jac_geo(self, q=None, axis='eef', htm=None):
        """
    Compute the Jacobians of the columns of the geometric Jacobian in the joint variable 'q'.
    This can be either to the end-effector frames (axis='eef'), to the Denavit-Hartenberg (DH) frames
    (axis='dh') or to the center-of-mass (COM) frames (axis='com').

    If axis ='dh' or 'com':

    If the robot has n links, jj_geo = robot.jac_jac_geo(q, htm) is a list of lists, in which
    jj_geo[i][j] is a 6 x n matrix that represents the Jacobian of the j-th column of the geometric
    Jacobian matrix of the i-th DH or COM frame.

    jj_geo[i][j] for j>i is not computed, because the j-th column of the geometric Jacobian matrix of the
    i-th DH or COM frame is always the 6 x n zero matrix, regardless of the 'q' and 'htm' chosen.

    jj_geo[i][j] could be alternatively computed numerically. For example, for axis='dh', by defining the function of q
    f = lambda q_var: robot.jac_geo(q = q_var, htm = htm, axis = 'dh')[0][i][j].tolist()
    and then computing the numerical Jacobian as Utils.jac(f,q).
    However, this function  is faster and has greater numerical accuracy, since it is computed analytically
    instead of numerically.

    If axis='eef', this is the same as computing axis='dh' but throwing away all but the last list away.


    Parameters
    ----------
    q : n-dimensional numpy vector or array
        The manipulator's joint configuration
        (default: the default joint configuration for the manipulator).

    axis : string
        For which axis you want to compute the FK:
        'eef': for the end-effector;
        'dh': for all Denavit-Hartenberg axis;
        'com': for all center-of-mass axis.
        (default: 'eef').

    htm : 4x4 numpy array or 4x4 nested list
        The robot base's configuration.
        (default: the same as the current htm).

    Returns
    -------
    jj_geo : list of lists of 6 x n numpy arrays (if axis='dh' or 'com') or a list of 6 x n numpy arrays (if axis=='eef')
        The Jacobian of the j-th column of the geometric Jacobian matrix of the i-th Denavit-Hartenberg frame.

    """
        return _jac_jac_geo(self, q, axis, htm)

    #######################################
    # Methods for dynamics model
    #######################################

    def dyn_model(self, q, qdot):
        """
    Compute the three robot's dynamic model terms, in a given joint configuration 'q'
    and joint speed 'qdot'.

    Parameters
    ----------
    q : n-dimensional numpy vector or array
        The manipulator's joint configuration.

    qdot : n-dimensional numpy vector or array
        The manipulator's joint configuration speed.

    Returns
    -------
    dyn_m : n x n numpy array
        The generalized inertia matrix at the joint configuration q.

    dyn_c : n-dimensional numpy column vector
        The generalized Coriolis-Centrifugal torques at the joint
        configuration q and joint configuration speed qdot.

    dyn_g : n-dimensional numpy column vector
        The generalized gravity torques at the joint configuration q.

    """
        return _dyn_model(self, q, qdot)

    #######################################
    # Methods for control
    #######################################
    @staticmethod
    def vector_field(curve, alpha=1, const_vel=1):
        """
    Computes a handle to a vector field function fun(p). Uses the vector field
    presented in 
    
    "Adriano M. C. Rezende; Vinicius M. Goncalves; Luciano C. A. Pimenta: 
    Constructive Time-Varying Vector Fields for Robot Navigation 
    IEEE Transactions on Robotics (2021)". 
    
    The vector field has constant velocity and use the function 
    G(u) = (2/pi)*atan(alpha*u).


    Parameters
    ----------
    curve : nxm numpy array or nxm nested list
        Curve, described as sampled points. Each one of the n rows should 
        contain a m-dimensional float vector that is the n-th m-dimensional
        sampled point of the curve. 
 
    alpha : positive float
        Controls the vector field behaviour. Greater alpha's imply more 
        robustness to the vector field, but increased velocity and acceleration
        behaviours. Used in G(u) = (2/pi)*atan(alpha*u)
        (default: 1).

    const_vel : positive float
        The constant velocity of the vector field. The signal of this number 
        controls the direction of rotation 
        (default: 1).

    Returns
    -------
    fun: a function handle that you can call as f(p), returning a numpy column vector. 'p' should be a
         m-dimensional vector.
    """

        return _vector_field(curve, alpha, const_vel)

    def task_function(self, htm_des, q=None, htm=None):
        """
    Computes the 6-dimensional task function for end-effector pose control,  
    given a joint configuration, a base configuration and the desired pose 
    'htm_des'.

    The first three entries are position error, and the three last entries are
    orientation error.

    Everything is written in scenario coordinates. 

    Also returns the Jacobian of this function.

    Parameters
    ----------
    htm_des : 4x4 numpy array or 4x4 nested list
        The desired end-effector pose. 
 
    q : nd numpy vector or array
        The manipulator's joint configuration.
        (default: the default joint configuration for the manipulator).

    htm : 4x4 numpy array or 4x4 nested list
        The robot base's configuration.
        (default: the same as the current htm).

    Returns
    -------
    r : 6-dimensional numpy column vector
        The task function.

    jac_r : 6 x n numpy matrix
        The respective task Jacobian.
    """
        return _task_function(self, htm_des, q, htm)

    @staticmethod
    def coop_task_function(robot_a, robot_b, htm_a_des, htm_a_b_des, q_a=None, q_b=None):
        """
    Computes the 12-dimensional task function for end-effector pose control
    of two robots 'robot_a', 'robot_b' given their respective configurations
    q_a and q_b.

    
    The first three components are relative position error.  

    The second three components are relative orientation error.  

    The third three components are position error for 'robot_a'.

    The third three components are orientation error for 'robot_a'.
    
    Everything is written in scenario coordinates. 
    
    Also returns the Jacobian of this function.

    Parameters
    ----------
    robot_a :robot object
        The first robot.

    robot_b :robot object
        The second robot.

    htm_a_des :4x4 numpy array or 4x4 nested list
        The desired pose for the end-effector of 'robot_a'.

    htm_a_b_des :4x4 numpy array or 4x4 nested list
        The desired relative pose between the end-effector of 'robot_a' and
        'robot_b'. That is, inv(htmA) * htmB.

    q_a : nd numpy vector or array
        'robot_a'' joint configuration
        (default: the default joint configuration for 'robot_a').

    q_b : md numpy vector or array
        'robot_b'' joint configuration
        (default: the default joint configuration for 'robot_b').

    Returns
    -------
    r : 12-dimensional numpy column vector
        The task function.

    jac_r : 12 x (n+m) numpy matrix
        The respective task Jacobian.
    """
        return _coop_task_function(robot_a, robot_b, htm_a_des, htm_a_b_des, q_a, q_b)

    #######################################
    # Methods for simulation
    #######################################

    def gen_code(self):
        """Generate code for injection."""
        return _gen_code(self)

    def update_col_object(self, time):
        """
        Update internally the objects that compose the collision model to the
        current configuration of the robot.
        """
        _update_col_object(self, time)

    def add_col_object(self, sim):
        """
        Add the objects that compose the collision model to a simulation.

        Parameters
        ----------
        sim : 'Simulation' object
            'Simulation' object.
    """
        _add_col_object(self, sim)

    def attach_object(self, obj):
        """
        Attach an object to the end-effector.
        The list of the type of objects that can be grouped can be seen in 'Utils.IS_GROUPABLE'.

        Parameters
        ----------
        obj : Object that is one of the types in 'Utils.IS_GROUPABLE'
            Object to be attached.
    """
        _attach_object(self, obj)

    def detach_object(self, obj):
        """
        Detach an object (ball, box or cylinder) to the end-effector.

        Parameters
        ----------
        obj : object
            Object to be detached.
    """
        _detach_object(self, obj)

    #######################################
    # Robot constructors
    #######################################

    @staticmethod
    def create_kuka_kr5(htm=np.identity(4), name='kukakr5', color="#df6c25", opacity=1):
        """
    Create a Kuka KR5, a six-degree of freedom manipulator.
    Thanks Sugi-Tjiu for the 3d model (see https://grabcad.com/library/kuka-kr-5-r850).

    Parameters
    ----------
    htm : 4x4 numpy array or 4x4 nested list
        The initial base configuration for the robot.
        (default: np.identity(4))
 
    name : string
        The robot name.
        (default: 'kukakr5').

    htm : color
        A HTML-compatible string representing the object color.
        (default: '#df6c25')'.

    opacity : positive float between 0 and 1
        The opacity of the robot. 1 = fully opaque and 0 = transparent.
        (default: 1)

    Returns
    -------
    robot : Robot object
        The robot.

    """
        base_3d_obj, links, htm_base_0, q0 = _create_kuka_kr5(htm, name, color, opacity)
        return Robot(name, links, base_3d_obj, htm, htm_base_0, q0)

    @staticmethod
    def create_epson_t6(htm=np.identity(4), name='epsont6', color="white", opacity=1):
        """
    Create an Epson T6, a SCARA manipulator.
    Thanks KUA for the 3d model (see https://grabcad.com/library/epson-t6-scara-robot-1).

    Parameters
    ----------
    htm : 4x4 numpy array or 4x4 nested list
        The initial base configuration for the robot.
        (default: np.identity(4))

    name : string
        The robot name.
        (default: 'epsont6').

    htm : color
        A HTML-compatible string representing the object color.
        (default: 'white')'.

    opacity : positive float between 0 and 1
        The opacity of the robot. 1 = fully opaque and 0 = transparent.
        (default: 1)

    Returns
    -------
    robot : Robot object
        The robot.

    """
        base_3d_obj, links, htm_base_0, q0 = _create_epson_t6(htm, name, color, opacity)
        return Robot(name, links, base_3d_obj, htm, htm_base_0, q0)

    @staticmethod
    def create_staubli_tx60(htm=np.identity(4), name='staublitx60', color="#ff9b00", opacity=1):
        """
    Create a Staubli TX60.
    Model taken from the ROS github repository (https://github.com/ros-industrial/staubli).

    Parameters
    ----------
    htm : 4x4 numpy array or 4x4 nested list
        The initial base configuration for the robot.
        (default: np.identity(4))

    name : string
        The robot name.
        (default: 'staublitx60').

    htm : color
        A HTML-compatible string representing the object color.
        (default: '#ff9b00')'.

    opacity : positive float between 0 and 1
        The opacity of the robot. 1 = fully opaque and 0 = transparent.
        (default: 1)

    Returns
    -------
    robot : Robot object
        The robot.

    """
        base_3d_obj, links, htm_base_0, q0 = _create_staubli_tx60(htm, name, color, opacity)
        return Robot(name, links, base_3d_obj, htm, htm_base_0, q0)

    @staticmethod
    def create_kuka_lbr_iiwa(htm=np.identity(4), name='kukalbriiwa', color="silver", opacity=1):
        """
    Create a Kuka LBR IIWA 14 R820.
    Model taken from the ROS github repository (https://github.com/ros-industrial/kuka_experimental).

    Parameters
    ----------
    htm : 4x4 numpy array or 4x4 nested list
        The initial base configuration for the robot.
        (default: np.identity(4))

    name : string
        The robot name.
        (default: 'kukalbriiwa').

    htm : color
        A HTML-compatible string representing the object color.
        (default: 'silver')'.

    opacity : positive float between 0 and 1
        The opacity of the robot. 1 = fully opaque and 0 = transparent.
        (default: 1)

    Returns
    -------
    robot : Robot object
        The robot.

    """
        base_3d_obj, links, htm_base_0, q0 = _create_kuka_lbr_iiwa(htm, name, color, opacity)
        return Robot(name, links, base_3d_obj, htm, htm_base_0, q0)

    #######################################
    # Advanced methods
    #######################################

    def compute_dist(self, obj, h=0.00001, g=0.00001, q=None, htm=None, old_dist_struct=None, tol=0.0005,
                     no_iter_max=20):
        """
    Compute (h,g) distance structure from each one of the robot's link to a 
    'simple' external object (ball, box or cylinder), given a joint and base 
    configuration. If h and g are very small, this is simply the traditional
    distance.

    Use an iterative algorithm, based on h-projections and a modification of
    Von Neumann's cyclic projection algorithm.

    Parameters
    ----------
    obj : a simple object (ball, box or cylinder)
        The external object for which the distance structure is going to be 
        computed, for each robot link.
    h : positive float
        Smoothing parameter for the robot's links, in meters.
        (default: 0.0001 m).
    g : positive float
        Smoothing parameter for the external object.
        (default: 0.0001 m).
    q : nd numpy vector or array
        The manipulator's joint configuration.
        (default: the default joint configuration for the manipulator).
    htm : 4x4 numpy array or 4x4 nested list
        The robot base's configuration.
        (default: the same as the current htm).
    old_dist_struct : 'DistStruct' object
        'DistStruct' obtained previously for the same robot and external object.
        Can be used to enhance the algorith speed using the previous closest 
        point as an initial guess.
        (default: None).
    tol : positive float
        Tolerance for convergence in the iterative algorithm, in meters.
        (default: 0.0005 m).        
    no_iter_max : positive int
        The maximum number of iterations for the algorithm.
        (default: 20 iterations). 

    Returns
    -------
    dist_struct : '_DistStructRobotObj' object
        Distance struct for each one of the m objects that compose the robot's
        collision model. Contains a list of m '_DisStructLinkObj' objects.
    """

        return _compute_dist(self, obj, h, g, q, htm, old_dist_struct, tol, no_iter_max)
