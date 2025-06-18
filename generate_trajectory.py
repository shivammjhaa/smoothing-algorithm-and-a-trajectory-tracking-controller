import numpy as np
from smooth_path import smooth_path

def generate_trajectory(waypoints, velocity=0.2):
    """
    Generates a time-parameterized trajectory from smoothed waypoints.

    Args:
        waypoints (list of tuple): List of 2D waypoints [(x1, y1), (x2, y2), ...].
        velocity (float): Constant velocity (m/s) used to time-parameterize the path.

    Returns:
        list of tuple: Trajectory as [(x, y, t), ...] where t is cumulative time in seconds.
    """
    smooth_pts = smooth_path(waypoints)
    trajectory = []
    total_time = 0.0

    for i in range(len(smooth_pts) - 1):
        x0, y0 = smooth_pts[i]
        x1, y1 = smooth_pts[i + 1]
        distance = np.linalg.norm([x1 - x0, y1 - y0])
        dt = distance / velocity
        total_time += dt
        trajectory.append((x1, y1, total_time))

    return trajectory

if __name__ == "__main__":
    # Example usage with test waypoints
    waypoints = [(0, 0), (2, 3), (5, 4), (7, 1)]
    trajectory = generate_trajectory(waypoints)

    print("Generated trajectory:")
    for x, y, t in trajectory:
        print(f"x: {x:.2f}, y: {y:.2f}, time: {t:.2f}")

    print("Trajectory generated successfully.")
