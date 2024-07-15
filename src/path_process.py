import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial import KDTree

def interpolate_path(points, interval):
    """
    Interpolates a path with given points at specified intervals.
    
    Args:
    points (list of tuple): A list of (x, y) tuples representing the path.
    interval (float): The desired interval between interpolated points.
    
    Returns:
    np.ndarray: Interpolated path as an array of (x, y) points.
    """
    points = np.array(points)
    x = points[:, 0]
    y = points[:, 1]
    
    # Calculate the cumulative distance between points
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
    
    # Create interpolation functions
    fx = interp1d(cumulative_distances, x, kind='linear')
    fy = interp1d(cumulative_distances, y, kind='linear')
    
    # Generate new distances at the specified interval
    new_distances = np.arange(0, cumulative_distances[-1], interval)
    
    # Generate new interpolated points
    new_x = fx(new_distances)
    new_y = fy(new_distances)
    
    return np.column_stack((new_x, new_y))

def plot_paths(original_points, interpolated_points):
    """
    Plots the original and interpolated paths.
    
    Args:
    original_points (list of tuple): The original path points.
    interpolated_points (np.ndarray): The interpolated path points.
    """
    original_points = np.array(original_points)
    
    plt.figure(figsize=(10, 6))
    plt.plot(original_points[:, 0], original_points[:, 1], 'o-', label='Original Path')
    plt.plot(interpolated_points[:, 0], interpolated_points[:, 1], 'x-', label='Interpolated Path', markersize=5)
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Original Path vs Interpolated Path')
    plt.grid(True)
    plt.show()

def find_nearest_point_idx(path, point):
    """
    Finds the nearest point on the path to the given point using KDTree.
    
    Args:
    path (np.ndarray): The path as an array of (x, y) points.
    point (tuple): The point (x, y) to find the nearest point on the path.
    
    Returns:
    tuple: The nearest point (x, y) on the path.
    """
    path = np.array(path)
    point = np.array(point)
    
    # Create a KDTree from the path points
    tree = KDTree(path)
    
    # Query the tree for the nearest point
    distance, nearest_idx = tree.query(point)
    
    return nearest_idx

# # Example usage
# original_path = [(6, 0), (1, 2), (4, 5), (7, 8), (10, 10)]
# interval = 0.2

# interpolated_path = interpolate_path(original_path, interval)
# plot_paths(original_path, interpolated_path)

# # Find the nearest point on the interpolated path to a given point
# test_point = (3, 1)
# nearest_point = find_nearest_point(interpolated_path, test_point)
# print(f"The nearest point on the path to {test_point} is {nearest_point}")
