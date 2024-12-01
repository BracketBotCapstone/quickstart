import numpy as np
import time
import matplotlib.pyplot as plt

class LookaheadController:
    def __init__(self, lookahead_distance, max_linear_velocity, max_angular_velocity, acceleration):
        self.lookahead_distance = lookahead_distance
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.acceleration = acceleration

    def find_goal_pose(self, path_pose_list, current_pose):
        print("Inside find_goal_pose")
        current_x, current_y = current_pose
        for pose in path_pose_list:
            distance = np.sqrt((pose[0] - current_x) ** 2 + (pose[1] - current_y) ** 2)
            if distance >= self.lookahead_distance:
                print("Found goal pose", pose)
                return pose
        print("Found goal pose", path_pose_list[-1])
        return path_pose_list[-1]  # Return the last pose if no pose is found within the lookahead distance

    def calculate_linear_error(self, current_pose, goal_pose):
        
        return np.sqrt( (current_pose[0] - goal_pose[0])**2 +
                    (current_pose[1] - goal_pose[1])**2 )

    def calculate_angular_error(self,current_pose, goal_pose):

        error_angular= np.arctan2(goal_pose[1]-current_pose[1],
                            goal_pose[0]-current_pose[0]) - current_pose[2]

        if error_angular <= -np.pi:
            error_angular += 2*np.pi

        elif error_angular >= np.pi:
            error_angular -= 2*np.pi
        
        return error_angular

    def compute_trajectory(self, current_pose, goal_pose):
        print("Inside compute_trajectory")

        error_linear = self.calculate_linear_error(current_pose, goal_pose)
        error_angular = self.calculate_angular_error(current_pose, goal_pose)

        print(f"error_linear: {error_linear}, error_angular: {error_angular}")

        # Create a timestamped trajectory
        trajectory = []
        current_time = time.time()
        linear_velocity = 0.0
        angular_velocity = 0.0

        iteration = 0

        while error_linear > 0.1 or abs(error_angular) > 0.1:  # Continue until close to the goal and orientation is corrected
            # iteration += 1
            # if iteration > 100:
            #     print("Exiting loop")
            #     break
            dt = 0.01

            if abs(error_angular) > 0.05:
                # Turn in place to correct angular error
                linear_velocity = 0.0
                angular_velocity = self.max_angular_velocity * np.sign(error_angular)
            else:
                # Proceed with normal velocity settings
                linear_velocity = self.max_linear_velocity
                if abs(error_angular) > 0.1:
                    angular_velocity = self.max_angular_velocity * np.sign(error_angular)
                else:
                    angular_velocity = 0.0

            # Update pose based on velocities
            current_pose = (
                current_pose[0] + linear_velocity * dt * np.cos(current_pose[2]),
                current_pose[1] + linear_velocity * dt * np.sin(current_pose[2]),
                current_pose[2] + angular_velocity * dt
            )

            # print("Current pose: ", current_pose)
            # print("Goal pose: ", goal_pose)

            # Recalculate errors based on new pose
            error_linear = self.calculate_linear_error(current_pose, goal_pose)
            # error_angular = self.calculate_angular_error(current_pose, goal_pose)
            error_angular -= angular_velocity * dt
            # print(f"linear error: {error_linear}")
            # print(f"error_angular: {error_angular}")

            trajectory.append((current_time, linear_velocity, angular_velocity))
            current_time += dt

        print("Trajectory computed")
        return trajectory

if __name__ == "__main__":
    path_pose_list = [(0, 0), (1, 1), (2, 2), (3, 3)]
    current_pose = (0, 0, 0)  # (x, y, yaw)

    controller = LookaheadController(lookahead_distance=1.0, max_linear_velocity=0.5, max_angular_velocity=0.3, acceleration=0.2)
    goal_pose = controller.find_goal_pose(path_pose_list, (0,0))
    trajectory = controller.compute_trajectory(current_pose, goal_pose)

    # get the size of the trajectory in storage
    import sys
    trajectory_size = sys.getsizeof(trajectory)
    print(f"Trajectory size: {trajectory_size} bytes")

    # # Plotting the trajectory
    # times = [timestamp for timestamp, _, _ in trajectory]
    # linear_velocities = [linear_vel for _, linear_vel, _ in trajectory]
    # angular_velocities = [angular_vel for _, _, angular_vel in trajectory]

    # plt.figure(figsize=(10, 5))
    # plt.subplot(2, 1, 1)
    # plt.plot(times, linear_velocities, label='Linear Velocity')
    # plt.xlabel('Time')
    # plt.ylabel('Linear Velocity')
    # plt.title('Linear Velocity over Time')
    # plt.grid(True)
    # plt.legend()

    # plt.subplot(2, 1, 2)
    # plt.plot(times, angular_velocities, label='Angular Velocity', color='orange')
    # plt.xlabel('Time')
    # plt.ylabel('Angular Velocity')
    # plt.title('Angular Velocity over Time')
    # plt.grid(True)
    # plt.legend()

    # plt.tight_layout()
    # plt.savefig('trajectory.png')

    # for timestamp, linear_vel, angular_vel in trajectory:
    #     print(f"Time: {timestamp}, Linear Velocity: {linear_vel}, Angular Velocity: {angular_vel}")
