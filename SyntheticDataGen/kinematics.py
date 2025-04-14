import sys
import numpy as np

EPSILON = np.finfo(float).eps
GRAVITY = 9.807
# Testing Tolerance, error allowed
TOLERANCE = 1e-6
# Change Average Distance Once Determined (Gravity * Distance)
DEFAULT_VELOCITY = np.sqrt(GRAVITY * 5)
# The Robot Arm Base Location For X
ROBOT_BASE_X = 10
# The Robot Arm Base Location For Y
ROBOT_BASE_Y = 10


def calculate_throw_parameters(delta_x, delta_y):
    """""Calculate the optimal throwing angle for a projectile to reach a target position"""
    # Calculate the optimal throwing angles using equation
    if abs(delta_y) < EPSILON:
        if (delta_x > 0):
            # Default angle for horizontal targets to the right
            return np.pi / 4
        else:
            # Default angle for horizontal targets to the left
            return 3 * np.pi / 4
        
    # Calculate theta using the formula
    theta = (np.arctan2(-delta_x, delta_y) + np.pi) / 2
    
    # Verify with constraint equation
    constraint = delta_x * np.cos(2 * theta) + delta_y * np.sin(2 * theta)
    
    if abs(constraint) > TOLERANCE:
        theta = np.arctan2(delta_y, -delta_x) / 2
    
    return theta
        

def calculate_required_velocity(delta_x, delta_y, theta):
    """Calculate the required velocity for the throw based on the constraint equation"""
    # Calculate denominator from constraint equation
    denominator = 2 * delta_x * np.sin(theta) * np.cos(theta) - 2 * delta_y * np.cos(theta)**2
        
    if abs(denominator) < EPSILON:
        throwDistance = np.sqrt(delta_x * 2 + delta_y * 2)
        velocity = np.sqrt(GRAVITY * throwDistance)
        return velocity
    
    # Calculate velocity using the equation
    numerator = delta_x * delta_x * delta_y
    velocity = numerator / denominator
    
    # Velocity magnitude
    return abs(velocity)


def verify_constraint_equation(delta_x, delta_y, theta):
    """Verify that the calculated theta satisfies the constraint equation"""
    # Verify the constraint equation
    leftSide = delta_x * np.cos(2 * theta) + delta_y * np.sin(2 * theta)
    return abs(leftSide) < TOLERANCE


def calculate_robot_arm_parameters(target_x, target_y):
    """Calculate all parameters needed for the robot arm throw."""
     # Convert target coordinates to delta values (relative to robot base)
    delta_x = target_x - ROBOT_BASE_X
    delta_y = target_y - ROBOT_BASE_Y
    
    # Calculate throwing angle
    theta = calculate_throw_parameters(delta_x, delta_y)
    
    # Verify that angle checks constraint equation
    if not verify_constraint_equation(delta_x, delta_y, theta):
        return "Constraint equation not satisfied with calculated angle"
    
    # Calculate required throwing velocity
    velocity = calculate_required_velocity(delta_x, delta_y, theta)
    
    # Calculate robot arm joint angles
    jointAngles  = inverse_kinematics_for_throwing(theta)
    
    # Calculate joint velocities needed to achieve throwing velocity
    jointVelocities = calculate_joint_velocities(velocity)
    
    return{
        "throwingAngle": theta,
        "throwingVelocity": velocity,
        "jointAngles": jointAngles,
        "jointVelocities": jointVelocities
    }


def inverse_kinematics_for_throwing(theta):
    """Convert throwing angel to robot orientation"""
    roll = 0.0
    pitch = theta
    yaw = 0.0
    
    return [roll, pitch, yaw]

def calculate_joint_velocities(velocity):
    """Convert throwing velocity to robot speed ratio"""
    MAX_VELOCITY = 2.0
    speed_ratio = min(velocity / MAX_VELOCITY, 1.0)
    
    return speed_ratio