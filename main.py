from AttemptOne.SyntheticDataGen.kinematics import calculate_robot_arm_parameters

def main():
    # Set up random (x,y) targets to test
    targets = [(15, 15), (10, 5), (5, 20)]
    
    for i, (target_x, target_y) in enumerate(targets):
        print(f"\nTarget {i+1}: ({target_x}, {target_y})")
        result = calculate_robot_arm_parameters(target_x, target_y)

        print(f"Throwing Angle: {result['throwingAngle']:.4f} radians")
        print(f"Throwing Velocity: {result['throwingVelocity']:.4f} m/s")
        print(f"Joint Angles: {result['jointAngles']}")
        print(f"Joint Velocities: {result['jointVelocities']}")


if __name__ == "__main__":
    main()
