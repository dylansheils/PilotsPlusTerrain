import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

def generate_smooth_path(points, z_coords, initial_heading, final_heading, turning_radius, theta, resolution=10000):
    # Sort the points based on z-coordinates in descending order
    sorted_indices = np.argsort(z_coords)[::-1]
    points = points[sorted_indices]
    z_coords = z_coords[sorted_indices]

    # Add initial and final points to the path based on headings
    start_point = points[0] + turning_radius * np.array([np.cos(initial_heading), np.sin(initial_heading)])
    end_point = points[-1] + turning_radius * np.array([np.cos(final_heading), np.sin(final_heading)])
    points = np.vstack((start_point, points, end_point))

    # Perform spline interpolation
    tck, u = splprep(points.T, s=0, per=False)
    u_new = np.linspace(u.min(), u.max(), resolution)
    x_new, y_new = splev(u_new, tck)

    # Extend the path to respect the turning radius
    extended_x, extended_y = [], []
    for i in range(len(x_new) - 1):
        x1, y1 = x_new[i], y_new[i]
        x2, y2 = x_new[i + 1], y_new[i + 1]
        # Calculate the distance between the points
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if dist > 2 * turning_radius:
            # Calculate the number of intermediate points needed
            num_points = int(dist / (2 * turning_radius)) + 1
            # Generate intermediate points
            for j in range(num_points):
                t = j / (num_points - 1)
                x = (1 - t) * x1 + t * x2
                y = (1 - t) * y1 + t * y2
                extended_x.append(x)
                extended_y.append(y)
        else:
            extended_x.append(x1)
            extended_y.append(y1)

    extended_x.append(x_new[-1])
    extended_y.append(y_new[-1])

    # Calculate the descent rate based on the descent angle
    descent_angle_rad = np.deg2rad(theta)
    descent_rate = 9.8 * np.sin(descent_angle_rad)  # Assuming elevation is in feet

    z_path = np.linspace(z_coords[0], z_coords[-1], len(extended_x))

    # Reinterpolate the path after adding points on the helix
    tck, u = splprep([extended_x, extended_y, z_path], s=0, per=False)
    u_new = np.linspace(u.min(), u.max(), resolution)
    x_path, y_path, z_path = splev(u_new, tck)

    # Calculate headings along the reinterpolated path
    dx = np.gradient(x_path)
    dy = np.gradient(y_path)
    headings = np.arctan2(dy, dx)

    # Find the indices of the original points in the reinterpolated path
    point_indices = []
    for point in points:
        distances = np.sqrt((x_path - point[0])**2 + (y_path - point[1])**2)
        point_indices.append(np.argmin(distances))
    point_indices.append(len(x_path) - 1)  # Add the index of the last point

    return x_path, y_path, z_path, headings, descent_rate, point_indices

# Generate random 2D points
num_points = 5
points = np.random.rand(num_points, 2) * 10
z_coords = np.sort(np.random.rand(num_points) * 2000)[::-1]  # Random z-coordinates in descending order

# Set initial and final headings (in radians)
initial_heading = np.random.uniform(0, 2 * np.pi)
final_heading = np.random.uniform(0, 2 * np.pi)

# Set the turning radius, smoothness factor, resolution, and theta
turning_radius = 2
smoothness = 1
resolution = 1000
theta = 15  # Descent angle in degrees

# Generate the smooth path
x_path, y_path, z_path, headings, descent_rate, point_indices = generate_smooth_path(points, z_coords, initial_heading, final_heading, turning_radius, theta, resolution)

fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')

# Plot the results
ax.plot3D(x_path, y_path, z_path, 'green')
ax.scatter(x_path[point_indices], y_path[point_indices], z_path[point_indices], c='red', marker='o', s=50)

print(f"Descent rate: {descent_rate:.2f} ft/s")

#plt.savefig("3d_path_cubic_interpolation.png", dpi=500)
plt.show() 