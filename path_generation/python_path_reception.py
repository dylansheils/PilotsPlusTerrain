import numpy as np
import os
import numpy as np
import subprocess
import time
from scipy.interpolate import interp1d

def write_binary_file(data, filename):
    data.tofile(filename)

def read_paths(filename):
    paths = []
    with open(filename, "rb") as file:
        while True:
            # Read the length of the next array
            len_bytes = file.read(4)  # Assuming 'int' is 4 bytes
            if not len_bytes:
                break  # End of file
            length = np.frombuffer(len_bytes, dtype=np.int32)[0]

            # Read the corresponding data points
            data = np.fromfile(file, dtype=np.double, count=length * 5)
            reshaped_data = data.reshape(-1, 5)  # Reshape to (len, 5)

            paths.append(reshaped_data)

    return paths
    
def interpolate_paths(paths, num_points=50000):
    interpolated_paths = []

    for path in paths:
        if len(path) > 1:
            # Calculate the cumulative distance along the path
            distances = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
            distances = np.insert(distances, 0, 0)  # Starting with a distance of 0

            # Generate an evenly spaced set of distances
            new_distances = np.linspace(0, distances[-1], num_points)

            # Create interpolation functions for each dimension
            interpolators = [interp1d(distances, path[:,i], kind='linear') for i in range(path.shape[1])]

            # Existing code to generate the new set of points
            new_path = np.vstack([interpolator(new_distances) for interpolator in interpolators]).T

            # Create a boolean mask where True represents rows with the 4th value >= 80
            mask = new_path[:, 3] >= 80  # Assuming the 4th value is at index 3

            # Apply the mask to filter out rows
            filtered_path = new_path[mask]
            if(len(filtered_path) > 0):
                interpolated_paths.append(filtered_path)

    return interpolated_paths

def get_paths(q0s, q1s):
	if(os.path.exists("./path_generation/paths.bin")):
		os.remove("path_generation/paths.bin")

	# Write to binary files
	write_binary_file(q0s, "./path_generation/q0s.bin")
	write_binary_file(q1s, "./path_generation/q1s.bin")
	subprocess.run(["./path_generation/output_program.exe"]) 
	time.sleep(0.1)
	while(not os.path.exists("./path_generation/paths.bin")):
		print("Trying again...")
		subprocess.run(["./path_generation/output_program.exe"]) 
		time.sleep(1)	
	return read_paths("./path_generation/paths.bin")

def memory_trick(q0s, q1s):
	# Example Usage
	total_paths = get_paths(q0s, q1s); return total_paths