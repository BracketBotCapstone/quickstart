import time

import matplotlib.pyplot as plt
import numpy as np


class LookaheadController:
    """
    This class is a simple lookahead controller that computes a trajectory to follow a path given by a list of poses.
    """

    def __init__(self, lookahead_distance, max_linear_velocity, max_angular_velocity, acceleration):
        """
        Initialize the LookaheadController with given parameters.

        :param lookahead_distance: The distance to look ahead on the path.
        :param max_linear_velocity: The maximum linear velocity.
        :param max_angular_velocity: The maximum angular velocity.
        :param acceleration: The acceleration rate.
        """
        self.lookahead_distance = lookahead_distance
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.acceleration = acceleration

    def find_goal_pose(self, path_pose_list, current_pose):
        """
        Find the goal pose within the lookahead distance.

        :param path_pose_list: List of poses representing the path.
        :param current_pose: The current pose of the robot.
        :return: The goal pose within the lookahead distance.
        """
        current_x, current_y = current_pose
        for pose in path_pose_list:
            distance = np.sqrt((pose[0] - current_x) ** 2 + (pose[1] - current_y) ** 2)
            if distance >= self.lookahead_distance:
                return pose
        return path_pose_list[-1]  # Return the last pose if no pose is found within the lookahead distance

    def calculate_linear_error(self, current_pose, goal_pose):
        """
        Calculate the linear error between the current pose and the goal pose.

        :param current_pose: The current pose of the robot.
        :param goal_pose: The goal pose to reach.
        :return: The linear error.
        """
        return np.sqrt((current_pose[0] - goal_pose[0])**2 +
                       (current_pose[1] - goal_pose[1])**2)

    def calculate_angular_error(self, current_pose, goal_pose):
        """
        Calculate the angular error between the current pose and the goal pose.

        :param current_pose: The current pose of the robot.
        :param goal_pose: The goal pose to reach.
        :return: The angular error.
        """
        error_angular = np.arctan2(goal_pose[1] - current_pose[1],
                                   goal_pose[0] - current_pose[0]) - current_pose[2]
        
        # Normalize the angular error to be within [-pi, pi]
        if error_angular <= -np.pi:
            error_angular += 2 * np.pi
        elif error_angular >= np.pi:
            error_angular -= 2 * np.pi
        
        return error_angular

    def compute_trajectory(self, current_pose, goal_pose):
        """
        Compute the trajectory to follow the path.

        :param current_pose: The current pose of the robot.
        :param goal_pose: The goal pose to reach.
        :return: A list of tuples representing the trajectory with timestamps.
        """
        # Calculate initial angular error
        error_angular = self.calculate_angular_error(current_pose, goal_pose)
        
        # Create a timestamped trajectory
        trajectory = []
        current_time = time.time()
        dt = 0.01  # Time step

        # Step 1: Rotate to face the goal
        while abs(error_angular) > 0.05:
            angular_velocity = self.max_angular_velocity * np.sign(error_angular)
            current_pose = (
                current_pose[0],
                current_pose[1],
                current_pose[2] + angular_velocity * dt
            )
            error_angular = self.calculate_angular_error(current_pose, goal_pose)
            trajectory.append((current_time, 0.0, angular_velocity))
            current_time += dt

        # Correct final angular position
        error_angular = self.calculate_angular_error(current_pose, goal_pose)
        current_pose = (
            current_pose[0],
            current_pose[1],
            current_pose[2] + error_angular
        )

        # Step 2: Move towards the goal with constant linear velocity
        error_linear = self.calculate_linear_error(current_pose, goal_pose)
        while error_linear > 0.05:
            linear_velocity = self.max_linear_velocity
            current_pose = (
                current_pose[0] + linear_velocity * dt * np.cos(current_pose[2]),
                current_pose[1] + linear_velocity * dt * np.sin(current_pose[2]),
                current_pose[2]
            )
            error_linear = self.calculate_linear_error(current_pose, goal_pose)
            trajectory.append((current_time, linear_velocity, 0.0))
            current_time += dt

        print("Trajectory computed")
        return trajectory

if __name__ == "__main__":
    path_pose_list = [(0, 0), (1, 1), (2, 2), (3, 3)]
    current_pose = (0, 0, 0)  # (x, y, yaw)
    controller = LookaheadController(lookahead_distance=1.0, max_linear_velocity=0.5, max_angular_velocity=0.3, acceleration=0.2)
    goal_pose = controller.find_goal_pose(path_pose_list, (0, 0))
    print(f"Goal pose: {goal_pose}")
    trajectory = controller.compute_trajectory(current_pose, goal_pose)

    # Plotting the trajectory
    times = [timestamp for timestamp, _, _ in trajectory]
    linear_velocities = [linear_vel for _, linear_vel, _ in trajectory]
    angular_velocities = [angular_vel for _, _, angular_vel in trajectory]

    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(times, linear_velocities, label='Linear Velocity')
    plt.xlabel('Time')
    plt.ylabel('Linear Velocity')
    plt.title('Linear Velocity over Time')
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, angular_velocities, label='Angular Velocity', color='orange')
    plt.xlabel('Time')
    plt.ylabel('Angular Velocity')
    plt.title('Angular Velocity over Time')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig('trajectory.png')

    # for timestamp, linear_vel, angular_vel in trajectory:
    #     print(f"Time: {timestamp}, Linear Velocity: {linear_vel}, Angular Velocity: {angular_vel}")
