import sys
sys.path.append('/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion')
sys.path.append('/mnt/c/Users/Dylan/Desktop/NewWindAware/source/path_generation')
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import imageio
import nmslib

# New global variable to store the points
paths_history = []

import os

def generate_random_vectors(num_points, vector_length):
    # Define the minimum and maximum values for each dimension
    min_vals = np.array([-73, 43, 80])
    max_vals = np.array([-74, 45, 8000])

    # Generate random numbers between 0 and 1, then scale and shift them
    random_vectors = np.random.rand(num_points, vector_length)
    scaled_vectors = min_vals + (max_vals - min_vals) * random_vectors

    return scaled_vectors

num_points = 128  # Adjust as needed
vector_length = 3

# Generate random data for q0s and q1s
q0s = generate_random_vectors(num_points, vector_length)
q1s = generate_random_vectors(num_points, vector_length)
q0s = np.concatenate((q0s, np.array([[-74,43,2000]])))
q1s = np.concatenate((q1s, np.array([[-73,42.5,200]])))

from node import Node
from montecarlo import MonteCarlo

def linear_path(start, end, num_points=5*20):
    linear_space = np.linspace(start, end, num=num_points)
    zero_columns = np.zeros((num_points, 2))
    path_with_zeros = np.hstack((linear_space, zero_columns))
    return path_with_zeros

def norm(pointA, pointB):
	return ((pointA[0] - pointB[0])**2 + (pointA[1] - pointB[1])**2 + (pointB[2] - pointA[2])**2)**(1/2)

import rasterio
import numpy as np
from scipy.ndimage import minimum_filter, uniform_filter
from urllib.request import urlretrieve

trial = 0
def read_geotiff(filename):
    with rasterio.open(filename) as dataset:
        band = dataset.read(1)
        ds = dataset.transform
    return band, ds

files = ["USGS_1_n44w073.tif"]#, "USGS_1_n44w074.tif"]

index = None
def load_downsampled_data(filename="USGS_1_n43w074.tif"):
    band, transform = read_geotiff(filename)

    # Get the dimensions of the band
    nrows, ncols = band.shape

    # Generate row and column indices
    col_indices, row_indices = np.meshgrid(np.arange(ncols), np.arange(nrows))

    # Apply the affine transformation to get geographic coordinates
    X, Y = rasterio.transform.xy(transform, row_indices, col_indices)

    return np.array(X), np.array(Y), band

import random
grid_coordinates_final = None

def create_nmslib_index(grid_coordinates, elevation_data, sampling_rate=0.05):
    # Initialize nmslib index
    index = nmslib.init(method='hnsw', space='l2')

    # Combine coordinates and elevation into a single array
    data_points = np.column_stack((grid_coordinates, elevation_data.flatten()))

    # Sample the data points
    sample_size = int(len(data_points) * sampling_rate)
    sampled_data_points = data_points[np.random.choice(data_points.shape[0], sample_size, replace=False)]
    global grid_coordinates_final
    grid_coordinates_final = sampled_data_points
    global elevation_data_final
    elevation_data_final = elevation_data.flatten()
    # Add sampled data points to the index
    index.addDataPointBatch(sampled_data_points)

    # Create the index
    index.createIndex(print_progress=True)

    return index

indicies = []
def path_collision_detection(path):
    global indicies
    if(indicies == []):
    	global files
    	for file in files:
        	# Load elevation data
        	X, Y, elevation_data = load_downsampled_data(file)

        	# Flatten and combine X, Y for easier processing
        	grid_coordinates = np.column_stack((X.flatten(), Y.flatten()))

        	index = create_nmslib_index(grid_coordinates, elevation_data)
        	indicies.append(index)

    # Define a threshold for collision
    elevation_threshold = 50  # Adjust as needed

    # Initialize an empty list for intersection points
    intersection_points = []
    # Check each point in the path for collision
    for point in path:
    	for index in indicies:
        	# Query the index for the nearest neighbor
        	nearest_neighbor = index.knnQuery(point[:2], k=1)
        	closest_idx = nearest_neighbor[0][0]; global grid_coordinates_final; global elevation_data_final
        	closest_point = grid_coordinates_final[closest_idx]
        	closest_elevation = elevation_data_final[closest_idx]

        	# Check for collision
        	if abs(closest_elevation - point[2]) < elevation_threshold:
        		intersection_points.append(point)

    # Format the intersection points
    if intersection_points:
        intersections_array = np.array(intersection_points)
       	zero_columns = np.zeros((intersections_array.shape[0], 2))
        intersections_with_zeros = np.hstack((intersections_array, zero_columns))
        return intersections_with_zeros
    else:
        return None

best_path = None
best_yet = np.inf
class PointSet:
	def __init__(self, q0, q1):
		self.q0s = q0
		self.q1s = q1
		self.end = q1
		self.length = None

	def give_options(self):
		samples = 32
		angles = [k * 2 * np.pi / samples for k in range(1, 2 * samples - 1)]
		candidate_points = []

		for angle in angles:
			candidate_point = self.intersection + (np.random.rand() + 0.1) * 0.5 * np.array([np.sin(angle), np.cos(angle), 0])
			candidate_points.append(candidate_point)

		distances = [np.linalg.norm(point - self.end)**2 for point in candidate_points]
		sorted_points = [point for _, point in sorted(zip(distances, candidate_points))]

		return sorted_points

	def path_length(self, path):
		length = np.sum(np.linalg.norm(np.diff(path[:, :3], axis=0), axis=1))
		if(self.length == None):
			self.length = length
		return length

	def judge_path(self, path, desired_point):
		length = 1
		intersections = path_collision_detection(path)
		if(type(intersections) == type(None)):
			global best_path
			best_path = path
			return 0, 0
		else:
			length = len(intersections)
			intersections = np.array(intersections)
		#print("Number of Intersections: ", length)
		point_final = np.array(intersections[random.randint(0,length-1)][0:3])
		self.intersection = point_final
		length = self.path_length(path)
		return ((intersections.shape[0] + 1)*abs(length), length)

	def get_sample_points(self):
		point_array = []
		for a, b in zip(self.q0s, self.q1s):
			point_array.append([a, b])
		return point_array

	def merge_paths(self, example_paths):
		end_path = []
		for element in example_paths:
			end_path.extend(element[1:-1])
		return end_path

	def evaluate(self):
		global best_yet
		total_path = []
		for q0, q1 in zip(self.q0s, self.q1s):
			path_segment = linear_path(q0, q1)
			total_path.append(path_segment)
		#end_path = total_path
	#def evaluate(self):
	#	global best_yet
	#	total_path = memory_trick(self.q0s, self.q1s)
		#total_path = total_path[:len(self.q0s)]
		#print(total_path[0])
		end_path = np.array(self.merge_paths(total_path))
		value, length = self.judge_path(end_path, self.end)
		length = abs(length)
		#print("Value: ", value)
		#print("Q0s: ", self.q0s)
		#print("Q1s: ", self.q1s)
		#print("Best yet: ", best_yet)
		#print("Path Length: ", length)
		changed = False
		global best_path
		if(value <= best_yet):
			best_yet = value
			best_path = end_path
			print(self.get_sample_points())
			plot_best_path()
			#else:
			#	best_yet = value
			#	changed = True
			#	best_path = end_path
		global paths_history
		global trial
		if(trial > 100):
			create_gif(paths_history)
			trial = 0
		trial += 1
		return value

	def add_intermediate(self, choice):
		# q0s - sources
		# q1s - destinations
		# A -> C -> B
		# q0 = [A]
		# q1 = [B]

		# q0 = [A, C]
		# q1 = [C, B]

		# A -> C -> D -> B
		# q0 = [A, C, D]
		# q1 = [C, D, B]

		temp = self.q1s[-1] # B
		self.q1s = self.q1s[:-1] # []
		self.q0s = np.concatenate((self.q0s, np.array([choice])), axis=0)
		self.q1s = np.concatenate((self.q1s, np.array([choice]), np.array([temp])), axis=0)

def child_finder(node, montecarlo):
    global paths_history
    for move in node.state.give_options():
        child = Node(deepcopy(node.state))
        child.state.add_intermediate(move)
        node.add_child(child)

        # Constructing the path with correct order without looping back
        child_path = [child.state.q0s[0]]  # Start with the first q0
        for q0, q1 in zip(child.state.q0s[1:], child.state.q1s):  # Start from the second q0 as we already added the first
            child_path.extend([q1, q0])  # q1 -> q0 for each pair
        child_path.extend([child.state.q1s[-1]])
        paths_history.append(np.array(child_path))

from mpl_toolkits.mplot3d import Axes3D

import imageio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def merge_files():
	allX, allY, elevation_datas = [], [], []
	global files
	for file in files:
		X, Y, elevation_data = load_downsampled_data(file)
	allX.append(np.array(X))
	allY.append(np.array(Y))
	elevation_datas.append(np.array(elevation_data))
	return (np.concatenate(allX, axis=1), 
            np.concatenate(allY, axis=1), 
            np.concatenate(elevation_datas, axis=1))

import numpy as np
import matplotlib.pyplot as plt
import imageio
from mpl_toolkits.mplot3d import Axes3D

def plot_best_path(filename='best_path.jpg'):
    print("Plotting best path...")
    # Load elevation data
    global best_path
    X, Y, elevation_data = merge_files()

    ## Initialize plot for best path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X, Y, elevation_data, alpha=0.5, cmap='terrain')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Elevation')
    print("BEST PATH: ", best_path)

    # Plot the best path
    ax.plot(best_path[:, 0], best_path[:, 1], best_path[:, 2], marker='o', color='red', linewidth=2)

    # Save the plot as a JPG file
    plt.savefig(filename)
    plt.close(fig)

def create_gif(paths_history, filename='path_animation.gif', sample_rate=0.2):
    print("Making gif...")
    return
    # Load elevation data
    plot_best_path()
    print("Plotted Best Path")
    X, Y, elevation_data = merge_files()

    # Calculate the sample rate
    total_paths = len(paths_history)
    desired_images = 100
    sample_rate = max(1, total_paths // desired_images)

    # Sample the paths
    sampled_paths = paths_history[::sample_rate]

    # Initialize plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X, Y, elevation_data, alpha=0.5, cmap='terrain')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Elevation')

    # Prepare a single plot line object for updating
    line, = ax.plot([], [], [], marker='o', color='blue')

    # Create the GIF writer
    with imageio.get_writer(filename, mode='I', duration=0.1, loop=0) as writer:
        for i, path in enumerate(sampled_paths):
            # Update only the data of the line object
            line.set_data(path[:, :2].T)
            line.set_3d_properties(path[:, 2])
            ax.set_title(f'Step {i + 1}')

            # Save frame to buffer
            fig.canvas.draw()
            image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            writer.append_data(image)

    plt.close(fig)



def node_evaluator(node, montecarlo):
	value = node.state.evaluate()
	return value

#--------------------------------------------------

print("Making inital path...")
inital_path = PointSet(np.array([[-72.5, 44, 750]]), np.array([[-73.9648, 44.2664, 0]]))
print("Loading...")
inital_path.evaluate()

print("Setting up Monte Carlo...")
montecarlo = MonteCarlo(Node(inital_path))

montecarlo.child_finder = child_finder
montecarlo.node_evaluator = node_evaluator
montecarlo.simulate(1000)