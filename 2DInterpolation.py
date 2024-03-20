import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

def generate_smooth_path(points, initial_heading, final_heading, turning_radius, smoothness=0.1, resolution=100):
    num_points = len(points)
   
    # Add initial and final points to the path based on headings
    extended_points = np.vstack(([points[0][0] + turning_radius * np.cos(initial_heading),
                                  points[0][1] + turning_radius * np.sin(initial_heading)], points,
                                 [points[-1][0] + turning_radius * np.cos(final_heading),
                                  points[-1][1] + turning_radius * np.sin(final_heading)]))
   
    # Initialize interpolation parameters
    k = 3
   
    # Iteratively adjust interpolation to achieve desired smoothness and turning radius
    for _ in range(100):
        tck, _ = splprep([extended_points[:, 0], extended_points[:, 1]], s=smoothness, k=5)
       
        # Generate points along the path
        u = np.linspace(0, 1, resolution)
        x_path, y_path = splev(u, tck)
       
        # Calculate curvature along the path
        dx = np.gradient(x_path)
        ddx = np.gradient(dx)
        dy = np.gradient(y_path)
        ddy = np.gradient(dy)
        curvature = np.abs(dx * ddy - ddx * dy) / (dx ** 2 + dy ** 2) ** 1.5
       
        # Check if curvature exceeds the turning radius constraint
        if np.max(curvature) <= 1 / turning_radius:
            print("Curvature Extended")
            break
       
        # Adjust interpolation parameters for smoother path
        k += 1
   
    # Calculate headings along the path
    headings = np.arctan2(dy, dx)
   
    return x_path, y_path, headings

# Generate random 2D points
num_points = 6
points = np.random.rand(num_points, 2) * 10

# Set initial and final headings (in radians)
initial_heading = np.random.uniform(0, 2 * np.pi)
final_heading = np.random.uniform(0, 2 * np.pi)

# Set the turning radius, smoothness factor, and resolution
turning_radius = 0.5
smoothness = 0.1
resolution = 1000

# Generate the smooth path
x_path, y_path, headings = generate_smooth_path(points, initial_heading, final_heading, turning_radius, smoothness, resolution)

# Plot the results
plt.figure(figsize=(8, 6))
plt.plot(x_path, y_path, 'b-', label='Smooth Path')
plt.plot(points[:, 0], points[:, 1], 'ko', label='Original Points')
plt.plot(x_path[0], y_path[0], 'go', label='Start Point')
plt.plot(x_path[-1], y_path[-1], 'ro', label='End Point')

# Plot arrows to indicate initial and final headings
plt.arrow(points[0, 0], points[0, 1], turning_radius * np.cos(initial_heading), turning_radius * np.sin(initial_heading),
          head_width=0.3, head_length=0.5, fc='k', ec='k')
plt.arrow(points[-1, 0], points[-1, 1], turning_radius * np.cos(final_heading), turning_radius * np.sin(final_heading),
          head_width=0.3, head_length=0.5, fc='k', ec='k')

plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Smooth Path with Custom Interpolation')
plt.grid(True)
plt.axis('equal')
plt.show()