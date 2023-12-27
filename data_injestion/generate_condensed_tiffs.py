import rasterio
import numpy as np
from scipy.ndimage import minimum_filter, uniform_filter
from urllib.request import urlretrieve

def read_geotiff(filename):
    with rasterio.open(filename) as dataset:
        band = dataset.read(1)
        ds = dataset.transform
    return band, ds

def transform_coordinates(ds, original_shape, downsampled_shape):
    # Determine the scaling factor
    scale_x = original_shape[1] / downsampled_shape[1]
    scale_y = original_shape[0] / downsampled_shape[0]

    # Number of points along each dimension (16x16 grid)
    num_points = int(np.sqrt(1024))

    # Generate grid points for the downsampled image
    x_coords = np.linspace(0, downsampled_shape[1] - 1, num_points) * scale_x
    y_coords = np.linspace(0, downsampled_shape[0] - 1, num_points) * scale_y

    # Compute the geographic coordinates for each grid point
    points = np.array([ds * (x, y) for y in y_coords for x in x_coords])
    return points

def save_sample_points(points, file_name):
    with open(file_name, 'wb') as file:
        points.tofile(file)

def downsample(array, window_size):
    """
    Downsample the array by averaging over blocks of size window_size x window_size.
    """
    # Ensure the array dimensions are divisible by window_size
    new_height = array.shape[0] // window_size * window_size
    new_width = array.shape[1] // window_size * window_size
    array = array[:new_height, :new_width]

    # Reshape and take the mean over each block
    downsampled_array = (array.reshape(new_height // window_size, window_size, 
                                      new_width // window_size, window_size)
                             .min(axis=(1, 3)))
    return downsampled_array


def try_different_window_sizes(image_data, region, ds):
    window_sizes = [1, 2, 4, 8, 16]  # Adjust as needed
    original_shape = image_data.shape
    for window_size in window_sizes:
        print(f"Applying convolution with window size {window_size}...")
        downsampled_result = downsample(image_data, window_size)

        modified_filename = f"{region}_{str(window_size)}_elevation.bin"
        print("Saving file...")
        save_array_to_file(downsampled_result, modified_filename)
        
        # Transform coordinates based on the original and downsampled result's shape
        sample_points = transform_coordinates(ds, original_shape, downsampled_result.shape)
        sample_points_file = f"{region}_{str(window_size)}_samplepoints.bin"
        save_sample_points(sample_points, sample_points_file)

def download_geotiff(region):    
    domain = f"https://rockyweb.usgs.gov/vdelivery/Datasets/Staged/Elevation/1/TIFF/current/{region}/USGS_1_{region}.tif"
    filename = f"USGS_1_{region}.tif"
    print("Starting download for ", region)
    urlretrieve(domain, filename)
    print("Downloaded ", region)
    return filename

def save_array_to_file(array, filename):
    # Ensure the array is in the correct data type (int16)
    array = array.astype(np.int16)

    # Save the array to a binary file
    array.tofile(filename)
    
def run_iteration(region):
    print("Starting region: ", region)
    filename = f"USGS_1_{region}.tif"

    image_data, ds = read_geotiff(filename)
    try_different_window_sizes(image_data, region, ds)