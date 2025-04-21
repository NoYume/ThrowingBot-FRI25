import rospy
import numpy as np
import tf.transformations
import intera_interface
from intera_interface import CHECK_VERSION
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions

# Import functions from our kinematics module
from kinematics import (
    calculate_robot_arm_parameters,
    inverse_kinematics_for_throwing,
    calculate_joint_velocities
)

class CartesianControl:
    """
    Class for controlling the Sawyer Robot in Cartesian space using kinematics calculations for trajectory planning
    """