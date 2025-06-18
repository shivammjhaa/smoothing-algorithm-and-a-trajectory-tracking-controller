import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

def smooth_path(waypoints, num_points=100):
    """
    Smooths a list of 2D waypoints using B-spline interpolation.

    Args:
        waypoints (list of tuple): Discrete waypoints [(x1, y1), (x2, y2), ...].
        num_points (int): Number of interpolated points to generate along the smooth path.

    Returns:
        list of tuple: Smoothed path as a list of (x, y) points.
    """
    # Extract x and y coordinates
    x, y = zip(*waypoints)

    # Generate spline representation of the path
    tck, _ = splprep([x, y], s=2.0)

    # Generate equally spaced points along the spline
    u = np.linspace(0, 1, num_points)
    x_smooth, y_smooth = splev(u, tck)

    return list(zip(x_smooth, y_smooth))


if __name__ == "__main__":
    # Test example
    waypoints = [(0, 0), (2, 3), (5, 4), (7, 1)]
    smooth = smooth_path(waypoints)

    # Plot original and smoothed path
    x, y = zip(*waypoints)
    sx, sy = zip(*smooth)

    plt.plot(x, y, 'ro--', label='Original')
    plt.plot(sx, sy, 'b-', label='Smoothed')
    plt.legend()
    plt.title("Path Smoothing with B-spline")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.show()

    print("Smoothed Path:", smooth)
    print("Path smoothing completed successfully.")