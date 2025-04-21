import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState, DisplayTrajectory

class SawyerSimulationController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('sawyer_sim_controller')
        
        # Initialize MoveIt! components
        self.robot = RobotCommander()
        self.group = MoveGroupCommander("right_arm")
        self.scene = PlanningSceneInterface()
        
        # Configure virtual table
        self.table_height = 0.0  # Adjust based on your needs
        self.table_dimensions = [0.8, 1.2, 0.01]  # x, y, thickness
        
        # Add virtual table to planning scene
        self.add_virtual_table()
        
        # Set planning parameters
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_planning_time(5.0)
        
        # Initialize data collection
        self.trajectory_data = []

    def add_virtual_table(self):
        """Add a virtual table to the planning scene"""
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base"
        table_pose.pose.position.z = self.table_height - 0.005
        self.scene.add_box("table", table_pose, self.table_dimensions)
        rospy.sleep(1)  # Wait for scene update

    def generate_trajectory_data(self, start_pose, target_pose):
        """Generate and store trajectory data between two poses"""
        # Set start state
        self.group.set_start_state_to_current_state()
        
        # Plan to target pose
        self.group.set_pose_target(target_pose)
        plan = self.group.plan()
        
        if plan.joint_trajectory.points:
            # Store trajectory data
            for point in plan.joint_trajectory.points:
                data_point = {
                    'joint_positions': point.positions,
                    'start_pose': [
                        start_pose.position.x,
                        start_pose.position.y,
                        start_pose.position.z
                    ],
                    'target_pose': [
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z
                    ]
                }
                self.trajectory_data.append(data_point)
            return True
        return False

    def save_data(self, filename):
        """Save collected data to numpy file"""
        np.save(filename, self.trajectory_data)
        print(f"Saved {len(self.trajectory_data)} data points to {filename}")

    def generate_dataset(self, num_samples=100):
        """Generate dataset with random target positions"""
        for _ in range(num_samples):
            # Generate random target within table bounds
            target_pose = Pose()
            target_pose.position.x = np.random.uniform(0.3, 0.8)
            target_pose.position.y = np.random.uniform(-0.4, 0.4)
            target_pose.position.z = self.table_height + 0.02
            
            # Set orientation (downward-facing gripper)
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 1.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.0
            
            # Get current pose as start state
            start_pose = self.group.get_current_pose().pose
            
            # Generate trajectory data
            if self.generate_trajectory_data(start_pose, target_pose):
                # Update virtual robot state
                self.group.set_joint_value_target(plan.joint_trajectory.points[-1].positions)
                self.group._g.update_robot_state(True)

def main():
    try:
        controller = SawyerSimulationController()
        
        print("Generating simulation data...")
        controller.generate_dataset(num_samples=100)
        
        controller.save_data("simulated_trajectories.npy")
        print("Data generation complete!")

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
