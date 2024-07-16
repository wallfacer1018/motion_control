import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev

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
# path = [(3, 0), (1, 2), (4, 5), (7, 8), (10, 10)]
# interval = 0.2

# interpolated_path = interpolate_path(path, interval)
# plot_paths(path, interpolated_path)


def smooth_corner(p1, p2, p3, radius):
    """
    Smooth the corner formed by three points with a circular arc of given radius.
    
    Parameters:
    p1, p2, p3 (tuple): Points forming the corner.
    radius (float): Radius of the circular arc.

    Returns:
    list of tuples: Points forming the circular arc.
    """
    # Convert points to numpy arrays
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    
    # Calculate direction vectors
    v1 = p1 - p2
    v2 = p3 - p2
    
    # Normalize direction vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    
    # Calculate angle between vectors
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    
    # Calculate tangent length
    tangent_length = radius / np.tan(angle / 2)
    
    # Check if tangent length exceeds distance to neighboring points
    if tangent_length > np.linalg.norm(p1 - p2) / 2 or tangent_length > np.linalg.norm(p3 - p2) / 2:
        return [p2]  # Do not smooth if the tangent length is too long
    
    # Calculate arc start and end points
    arc_start = p2 + v1 * tangent_length
    arc_end = p2 + v2 * tangent_length
    
    # Calculate arc center
    bisector = (v1 + v2) / 2
    bisector = bisector / np.linalg.norm(bisector)
    arc_center = p2 + bisector * radius / np.sin(angle / 2)
    
    # Generate arc points
    start_angle = np.arctan2(*(arc_start - arc_center)[::-1])
    end_angle = np.arctan2(*(arc_end - arc_center)[::-1])
    
    if np.cross(v1, v2) < 0:
        if start_angle > end_angle:
            start_angle -= 2 * np.pi
    else:
        if start_angle < end_angle:
            end_angle -= 2 * np.pi
    
    angles = np.linspace(start_angle, end_angle, num=20)
    
    arc_points = [tuple(arc_center + np.array([radius * np.cos(angle), radius * np.sin(angle)])) for angle in angles]
    
    return arc_points

def smooth_path_with_corners(path, radius=0.5):
    """
    Smooth a given path, preserving straight segments and smoothing corners with circular arcs.
    
    Parameters:
    path (list of tuples): A list of (x, y) points representing the path.
    radius (float): Radius of the circular arcs for smoothing corners.
    
    Returns:
    list of tuples: Smoothed path.
    """
    smoothed_path = [path[0]]
    
    for i in range(1, len(path) - 1):
        p1 = path[i - 1]
        p2 = path[i]
        p3 = path[i + 1]
        
        # Check if the points form a corner (non-collinear)
        if np.cross(np.array(p2) - np.array(p1), np.array(p3) - np.array(p2)) != 0:
            arc_points = smooth_corner(p1, p2, p3, radius)
            smoothed_path.extend(arc_points)
        else:
            smoothed_path.append(p2)
    
    smoothed_path.append(path[-1])
    return smoothed_path

# 通过加一个path上最后两个点射线上的点，点延伸path
def append_point(path):
    vector = np.subtract(path[-1], path[-2])
    new_point = tuple(path[-1] + 0.5*vector)
    path.append(new_point)
    return path

# # Example usage
# path = [(200, 375), (350,375), (350,25), (135,25)]
# smoothed_path = smooth_path_with_corners(path, radius=50)
# interpolated_path = interpolate_path(smoothed_path, 2)

# # Plot the original and smoothed path for visualization
# plt.plot(*zip(*path), 'bo-', label='Original Path')
# plt.plot(*zip(*interpolated_path), 'r-', label='Smoothed Path')
# plt.legend()
# plt.show()




