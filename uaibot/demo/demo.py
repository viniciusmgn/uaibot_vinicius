import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from ._control_demo_1 import _control_demo_1
from ._control_demo_2 import _control_demo_2
from ._control_demo_3 import _control_demo_3


from ._lesson_demo_3 import _lesson_demo_3

class Demo:
    """
  A class that contains demonstrations in robotics.
  """

    #######################################
    # Control demos
    #######################################

    @staticmethod
    def control_demo_1():
        """
    Show a robotic manipulator drawing a circle in a drawboard.
    The task demands that the manipulator keep a constant pose while
    drawing.
    """
        return _control_demo_1()

    @staticmethod
    def control_demo_2():
        """
    Show two robotic manipulators cooperating to transport an object between
    two plates. Note that the relative pose between the two end-effectors must
    be kept.

    The control uses task priority framework using the null space of the task
    Jacobian. The task of keeping the relative pose is prioritized over the
    task of moving the plate.
    """
        return _control_demo_2()

    @staticmethod
    def control_demo_3():
        """
    Show a robot manipulator that must achieve a pose inside a hole. Collision must be avoided.

    Perform second order kinematic control with constraints that enforce
    collision avoidance.
    """
        return _control_demo_3()

    @staticmethod
    def lesson_demo_3(robot_creator=None, width=800, height=600):
        """
    Show a robot manipulator that must achieve a pose inside a hole. Collision must be avoided.

    Perform second order kinematic control with constraints that enforce
    collision avoidance.
    """
        return _lesson_demo_3(robot_creator, width, height)