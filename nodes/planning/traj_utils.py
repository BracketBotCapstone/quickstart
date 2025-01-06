import numpy as np
from scipy import interpolate


def _min_jerk_spaces(N: int, T: float):
    """
    Generates a 1-dim minimum jerk trajectory from 0 to 1 in N steps & T seconds.
    Assumes zero velocity & acceleration at start & goal.
    The resulting trajectories can be scaled for different start & goals.
    Args:
        N: Length of resulting trajectory in steps
        T: Duration of resulting trajectory in seconds
    Returns:
        p_traj: Position trajectory of shape (N,)
        pd_traj: Velocity trajectory of shape (N,)
        pdd_traj: Acceleration trajectory of shape (N,)
    """
    assert N > 1, "Number of planning steps must be larger than 1."

    t_traj = np.linspace(0, 1, N)
    p_traj = 10 * t_traj**3 - 15 * t_traj**4 + 6 * t_traj**5
    pd_traj = (30 * t_traj**2 - 60 * t_traj**3 + 30 * t_traj**4) / T
    pdd_traj = (60 * t_traj - 180 * t_traj**2 + 120 * t_traj**3) / (T**2)

    return p_traj, pd_traj, pdd_traj


def generate_joint_space_min_jerk(start, goal, time_to_go: float, dt: float):
    """
    Primitive joint space minimum jerk trajectory planner.
    Assumes zero velocity & acceleration at start & goal.
    Args:
        start: Start joint position of shape (N,)
        goal: Goal joint position of shape (N,)
        time_to_go: Trajectory duration in seconds
        hz: Frequency of output trajectory
    Returns:
        waypoints: List of waypoints
    """
    steps = int(time_to_go / dt)

    p_traj, pd_traj, pdd_traj = _min_jerk_spaces(steps, time_to_go)

    D = goal - start
    q_traj = start[None, :] + D[None, :] * p_traj[:, None]
    qd_traj = D[None, :] * pd_traj[:, None]
    qdd_traj = D[None, :] * pdd_traj[:, None]

    waypoints = [{
        "time_from_start": i * dt,
        "position": q_traj[i, :],
        "velocity": qd_traj[i, :],
        "acceleration": qdd_traj[i, :],
    } for i in range(steps)]

    return waypoints


def retime_trajectory(trajectory, original_hz, target_hz):
    """
    Resample a trajectory from original_hz to target_hz.

    Args:
    trajectory: numpy array of shape (n_samples, n_dimensions)
    original_hz: original sampling frequency
    target_hz: desired sampling frequency

    Returns
    -------
    resampled_trajectory: numpy array of shape (new_n_samples, n_dimensions)
    """
    # Calculate the original and new time points
    original_duration = trajectory.shape[0] / original_hz
    original_times = np.linspace(0, original_duration, trajectory.shape[0])
    new_times = np.linspace(0, original_duration, int(original_duration * target_hz))

    # Create an interpolation function for each dimension
    interpolators = [
        interpolate.interp1d(original_times, trajectory[:, i], kind='cubic')
        for i in range(trajectory.shape[1])
    ]

    # Apply the interpolation to get the new trajectory
    resampled_trajectory = np.column_stack([interp(new_times) for interp in interpolators])

    return resampled_trajectory