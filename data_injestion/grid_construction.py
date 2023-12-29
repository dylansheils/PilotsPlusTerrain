import numpy as np
import math
from scipy.interpolate import RegularGridInterpolator
import os
import random
import matplotlib.pyplot as plt
import nmslib
from concurrent.futures import ThreadPoolExecutor

epsilon = 1e-2


# TO DOs:
# - Parallelize region search
# - Do basic tree search O(log(region)*region) instead of O(region^2) operations
# - Allow terrain removal based on reindexing/query trade-off
# - Adapt granularity of collision detection based on stage of flight (when higher check more
#       general paths)
# - Adapt search to be more selective as one gets lower


def load_downsampled_data(filename):
    """
    Load downsampled data from a binary file.
    """
    data = np.fromfile(filename, dtype=np.int16)
    return data.reshape((int(np.sqrt(len(data))), int(np.sqrt(len(data)))))

def load_sample_points(filename):
    """
    Load sample points from a binary file.
    """
    points = np.fromfile(filename, dtype=np.float64)
    return points.reshape(-1, 2)

def perform_bilinear_interpolation(sample_points):
    """
    Perform bilinear interpolation for latitude and longitude.
    """
    # Assuming a 32x32 grid
    grid_size = int(np.sqrt(len(sample_points)))
    grid_x, grid_y = np.mgrid[0:grid_size, 0:grid_size] 

    # Reshape latitude and longitude for interpolation
    lats = sample_points[:, 0].reshape((grid_size, grid_size))
    lons = sample_points[:, 1].reshape((grid_size, grid_size))

    # Make sure 1.0 is not out of bounds
    grid_size = grid_size - 1
    # Create interpolation functions
    lat_interp = RegularGridInterpolator((grid_x[:, 0] / grid_size, grid_y[0, :] / grid_size), lats)
    lon_interp = RegularGridInterpolator((grid_x[:, 0] / grid_size, grid_y[0, :] / grid_size), lons)

    return (lat_interp, lon_interp)

def compute_grid_size(filename):
    # Size of each element in bytes (np.int16 => 2 bytes)
    element_size_bytes = 2

    # Get the file size in bytes
    file_size_bytes = os.path.getsize(filename)

    # Compute the total number of elements in the matrix
    total_elements = file_size_bytes // element_size_bytes

    # Compute the grid size (width/height of the square matrix)
    grid_size = int(total_elements ** 0.5)  # Square root of total elements

    return grid_size

def interpolate(interp, target_position):
    """
    Interpolate for a given target position.
    """
    interp_lat, interp_lot = interp
    return (interp_lat(target_position), interp_lot(target_position))

class Region:
    region = None
    active_resolutions = None
    resolution_interpolated = None
    resolution_distance = None
    upper_left = None
    lower_right = None
    center = None
    best_resolution = None

    def __init__(self, region_name):
        self.region = region_name
        self.active_resolutions = [1,2,4,8]
        self.resolution_interpolated = {
            2: None,
            4: None,
            8: None,
            16: None
        }
        self.resolution_distance = {
            1: 1,
            2: 10,
            4: 100,
            8: 1000,
        }
        self.nmslib_index = nmslib.init(method='hnsw', space='l2')
        self.index_built = False
        self.best_resolution = 17
        self.loaded_areas = {-1: {(1,2)}}
        self.kdtree_data = np.array([]).reshape(0, 3)  # Store points for KD-tree in a NumPy array
        self.upper_left = np.array(self.interpolate_resolution(16, (0,0,0)))
        self.lower_right = np.array(self.interpolate_resolution(16, (1,1,0)))
        self.center = np.array(self.upper_left + self.lower_right)/2.0
        self.hashed_sizes = dict()
        sample_points = self.load_full(16,True)
        self.build_index()

    def build_index(self):
        if len(self.kdtree_data) > 0 and not self.index_built:
            self.nmslib_index.addDataPointBatch(self.kdtree_data)
            self.nmslib_index.createIndex(print_progress=True)
            self.index_built = True

    def ensure_interpolation_initialized(self, resolution):
        """Ensure that interpolation functions are initialized for the given resolution."""
        if self.resolution_interpolated[resolution] is None:
            points_filename = "/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_samplepoints.bin".format(self.region, resolution)
            sample_points = load_sample_points(points_filename)
            interp_lat, interp_lon = perform_bilinear_interpolation(sample_points)
            self.resolution_interpolated[resolution] = (interp_lat, interp_lon)

    def interpolate_resolution(self, resolution, point):
        """Interpolate latitude and longitude for a given resolution and point."""
        self.ensure_interpolation_initialized(resolution)
        interp_lat, interp_lon = self.resolution_interpolated[resolution]
        return interpolate((interp_lat, interp_lon), point[0:2])

    def load_full(self, resolution, skipCheck=False):
        if(skipCheck == False):
            if(resolution >= self.best_resolution):
                return False

        points_filename = "/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_samplepoints.bin".format(self.region, resolution)
        sample_points = load_sample_points(points_filename)
        interp_lat, interp_lot = perform_bilinear_interpolation(sample_points)
        interp = (interp_lat, interp_lot)
        self.resolution_interpolated[resolution] = interp
        self.best_resolution = resolution
        grid_filename = "/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_elevation.bin".format(self.region, resolution)
        self.load_into_mesh(load_downsampled_data(grid_filename), resolution)
        return True

    def within_region(self, point_actual):
        point_actual = np.array(point_actual[0:2])
        upper_left = np.array(self.upper_left)
        lower_right = np.array(self.lower_right)

        # Calculate Euclidean distance to upper left and lower right corners
        distance_to_upper_left = np.linalg.norm(point_actual - upper_left)
        distance_to_lower_right = np.linalg.norm(point_actual - lower_right)

        # Check if the point is within the region defined by upper left and lower right corners
        return distance_to_upper_left <= np.sqrt(2) and distance_to_lower_right <= np.sqrt(2)

    # Compute distance of center
    def distance_to_center(self, point_actual):
        point_actual = np.array(point_actual[0:2])
        return np.linalg.norm(point_actual - self.center)

    # Load in ONLY the points from their apporpriate files at the particular resolution
    def load_region(self, new_location):
        if(self.best_resolution == 1):
            return False
        
        global epsilon

        change = False
        for resolution in self.active_resolutions:
            grid_size = -1
            if("/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_elevation.bin".format(self.region, resolution) in self.hashed_sizes):
                grid_size = self.hashed_sizes["/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_elevation.bin".format(self.region, resolution)]
            else:
                grid_size = compute_grid_size("/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_elevation.bin".format(self.region, resolution))
                self.hashed_sizes["/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_elevation.bin".format(self.region, resolution)] = grid_size

            points_filename = "/mnt/c/Users/Dylan/Desktop/NewWindAware/source/data_injestion/{}_{}_samplepoints.bin".format(self.region, resolution)
            if(grid_size < 100000 or np.linalg.norm(new_location[0:2], self.center) <= np.sqrt(2)):
                if(self.load_full(resolution)):
                    change = True
        
        if(change):
            self.build_index()

    def load_into_mesh(self, sample_points, resolution):
        global epsilon
        global kdtree
        # Assuming the grid is square
        grid_size = len(sample_points)
        grid_x, grid_y = np.mgrid[0:grid_size, 0:grid_size]
        # Normalize grid coordinates and flatten the arrays
        norm_x = grid_x.flatten() / (grid_size - 1)
        norm_y = grid_y.flatten() / (grid_size - 1)
        # Interpolate latitudes and longitudes using vectorized operations
        lats, lons = interpolate(self.resolution_interpolated[resolution], (norm_x, norm_y))
        # Flatten the elevation data
        elevations = sample_points.flatten()
        # Batch insert the points into the octree
        # Extract upper left and lower right bounds
        upper_left_lat, upper_left_lon = self.upper_left[:2]
        lower_right_lat, lower_right_lon = self.lower_right[:2]

        # Update R-tree index
        num_points = len(lats)  # Number of points to update
        new_points = np.column_stack((lats, lons, elevations))  # Create an array of new points

        # Expand self.kdtree_data to accommodate new points
        self.kdtree_data = np.vstack((self.kdtree_data, np.zeros((num_points, 3))))

        # Assign the new points to the appropriate positions in self.kdtree_data
        self.kdtree_data[-num_points:, :] = new_points

    def unload_full(self, resolution):
        pass

    def knearestneighbor(self, points):
        return self.nmslib_index.knnQueryBatch(points, k=5)

    def detect_intersection(self, points):
        neighbours = self.knearestneighbor(points)
        all_distances = np.array([dist for _, dist in neighbours])
        intersection_points = np.array(points)[all_distances < (50**2)]
        return intersection_points

class Grid:
    regions = []

    def add_region(self, region_name):
        new_region = Region(region_name)
        self.regions.append(new_region)

    def remove_region(self, region_name):
        self.regions.remove(region_name)

    def update_regions(self, point):
        for region in self.regions:
            region.load_region(point)

    def subsample_path_random(self, path, max_length=10000):
        if len(path) <= max_length:
            return path
        return random.sample(path, max_length)
    
    def path_collision(self, path):
        self.load_data(path)
        intersection_points = []
        for region in self.regions:
            region_intersections = region.detect_intersection(path)
            if region_intersections.size > 0:
                intersection_points.extend(region_intersections)
        return intersection_points

    def load_data(self, path):
        selection_points = np.linspace(0, len(path)-1, 1000)
        for region in self.regions:    
            for idx in selection_points:
                region.load_region(path[int(idx)])