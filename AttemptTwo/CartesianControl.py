import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import RobotState
from Intera_SDK import intera_interface, CHECK_VERSION


class SawyerCartesianControl:
    def __init__(self):
        # Initilize ROS node
        rospy.init_node('sawyer_cartesian_controller')
        
        # Initilize Sawyer's limb
        self.limb = intera_interface.limb('right')
        self.gripper = intera_interface.Griiper('right')
        
        # Enable the robot
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        rs.enable()
        
        # Initialize MoveIt commander
        self.robot = RobotCommander()
        self.group = MoveGroupCommander("right_arm")
        
        # Set planning parameters
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_planning_time(5.0)
        
        # Intitilize data collection array
        self.trajectory_data = []
        
    def move_to_cartesian_pose(self, position, orientation):
        """
        Move to specificed position and orientation in Cartesian Space
        """
        
        pose_target = PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = position[0]
        pose_target.pose.position.y = position[1]
        pose_target.pose.position.z = position[2]
        pose_target.pose.orientation.x = orientation[0]
        pose_target.pose.orientation.y = orientation[1]
        pose_target.pose.orientation.z = orientation[2]
        pose_target.pose.orientation.w = orientation[3]
        
        
        # Set the target pose
        self.group.set_pose_target(pose_target)
        
        # Plan and excute
        plan = self.group.place()
        success = self.group.execute(plan, wait=True)
        
        return success, plan
    
    def open_gripper(self):
        """
        Open the gripper to release the ball
        """
        self.gripper.open()
        rospy.sleep(0.5)
        
    def close_gripper(self):
        """
        Close the gripper to grab the ball
        """
        self.gripper.close()
        self.sleep(0.5)
    
    def collect_trajectory_data(self, plan, target_position):
        """
        Collect tajectory data for imitation learning
        """
        trajectory = plan.joint_trajectory
        
        for point in trajectory.points:
            # Get joint positions
            joint_positions = point.positions
            
            # Calculate forward kinematics to get end-effector position
            # (In a real implementation, you would use the robot's FK service)
            # For simplicity, we'll use the current pose from MoveIt
            current_pose = self.group.get_current_pose().pose
            
            # Store data point
            data_point = {
                'joint_position': joint_positions,
                'end_effector_position': [
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z
                ],
                'target_position': target_position
            }
            
            self.trajectory_data.append(data_point)
            
    def save_trajectory_data(self, filename):
        """
        Save collected trajectory data to a numpy file
        """
        np.save(filename, self.trajectory_data)
        print(f"Saved trajectory data to {filename}")
        
    def execute_ball_drop(self, pickup_position, target_position):
        """
        Execute a ball drop sequence:
        1. Move to pickup position
        2. Close gripper to grab ball
        3. Move to a position above the target
        4. Move down to the target position
        5. Open gripper to drop ball
        6. Return to home position
        """
        # Define orinentation (downward-facing gripper)
        orientation = [0.0, 1.0, 0.0, 0.0]
        
        # Step 1: Move to pickup position
        approach_position = [pickup_position[0], pickup_position[1], pickup_position[2] + 0.1]
        success, _ = self.move_to_cartesian_pose(approach_position, orientation)
        if not success:
            return False, "Failed to move to approach position"
        
        # Move down to pickup position
        success, _ = self.move_to_cartesian_pose(pickup_position, orientation)
        if not success:
            return False, "Failed to move to pickup position"
        
        # Step 2: Close gripper to grab ball
        self.close_gripper()
        
        # Step 3: Move to a position above the target
        above_target = [target_position[0], target_position[1], target_position[2] + 0.1]
        success, plan = self.move_to_cartesian_pose(above_target, orientation)
        if not success:
            return False, "Failed to move above target"
        
        # Collect data for this trajectory
        self.collect_trajectory_data(plan, target_position)
        
        # Step 4: Move down to the target position
        success, plan = self.move_to_cartesian_pose(target_position, orientation)
        if not success:
            return False, "Failed to move to target position"
        
        # Collect data for this trajecotry
        self.collect_trajectory_data(plan, target_position)
        
        # Step 5: Open gripper to drop ball
        self.open_gripper()
        
        # Step 6: Return to home position
        self.group.set_named_target("right_neutral")
        success, _ = self.group.plan()
        if not success:
            return False, "Failed to plan movement to home position"
        
        success = self.group.execute(plan, wait=True)
        if not success:
            return False, "Failed to move to home position"
        
        return True, "Successfully executed ball drop"
     
    def main():
        try:
            controller = SawyerCartesianControl()
            
            #Define table dimensions and height
            table_height = 0.0 # Z-coords of table
            table_min_x, table_max_x = 0.3, 0.8 # X-coord bounds
            table_min_y, table_max_y = -0.4, 0.4 # Y-coord bounds
            
            pickup_position = [0.5, 0.0, table_height + 0.02]
            
            # Generate multiple target positions for data collection
            num_target = 1000
            target_positions = []
            
            for _ in range(num_target):
                #Random position on the table
                x = np.random.uniform(table_min_x, table_max_x)
                y = np.random.uniform(table_min_y, table_max_y)
                z = table_height + 0.02
                
                target_positions.append([x, y, z])
                
            # Excute ball drops for each target
            for i, target_positions in enumerate(target_positions):
                print(f"Excuting ball drop {i + 1}/{num_target}")
                success, message = controller.execute_ball_drop(pickup_position, target_position)
                print(message)
                
                if not success:
                    print(f"Skipping to next target")
                    continue
            
            # Save collected data
            controller.save_trajectory_data("sawyer_ball_drop_data.npy")
            
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return
        
    if __name__ == '__main__':
        main()