import numpy as np

def find_intersection(p1, p2, center, radius):
    """ Find intersection points of a line segment with a circle. """
    p1 = np.array(p1, dtype=np.float64)
    p2 = np.array(p2, dtype=np.float64)
    center = np.array(center, dtype=np.float64)

    d = p2 - p1
    f = p1 - center

    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - radius**2

    discriminant = b**2 - 4*a*c

    if discriminant < 0:
        return []  # No intersection

    discriminant = np.sqrt(discriminant)
    t1 = (-b - discriminant) / (2*a)
    t2 = (-b + discriminant) / (2*a)

    intersections = []
    if 0 <= t1 <= 1:
        intersections.append(p1 + t1 * d)
    if 0 <= t2 <= 1:
        intersections.append(p1 + t2 * d)

    return intersections

def find_target(path, current_pos, d):
    path = [tuple(p) for p in path]
    current_pos = tuple(current_pos)
    

    found_points = []
    
    for i in range(len(path) - 1, 0, -1):
        intersections = find_intersection(path[i-1], path[i], current_pos, d)
        if intersections:
            found_points.extend(intersections)
    
    if found_points:
        # Sort by distance to the end of the path, prefer points on later segments
        found_points.sort(key=lambda point: (np.linalg.norm(np.array(point) - np.array(path[-1])), 
                                                np.linalg.norm(np.array(point) - np.array(current_pos))))
        return found_points[0]

    return find_target(path, current_pos, 1.5*d) # 如果没有找到，扩大d再找

# 通过加一个path上最后两个点射线上的点，点延伸path
def append_point(path):
    vector = np.subtract(path[-1], path[-2])
    new_point = tuple(path[-1] + vector)
    path.append(new_point)
    return path

# Example usage
path = [(135, 25), (40, 170), (40, 375), (125, 352)]
d = 10
path=append_point(path)
# print(path)

current_pos = (40, 240)
target_pos = find_target(path, current_pos, d)
print(target_pos)

current_pos = (40, 370)
target_pos = find_target(path, current_pos, d)
print(target_pos)

current_pos = (135, 25)
target_pos = find_target(path, current_pos, d)
print(target_pos)

current_pos = (125, 360)
target_pos = find_target(path, current_pos, d)
print(target_pos)

current_pos = (40, 0)
target_pos = find_target(path, current_pos, d)
print(target_pos)
