#include <stdio.h>
#include <stdlib.h>
#include "data_injestor.h"
#include <string.h>
#include <math.h>
#include "interpolation.h"

// Use ALGLIB for interpolation
using namespace alglib;

typedef struct {
    spline2dinterpolant spline;
    size_t width;
    size_t height;
} DataInjestionData2D;

void read_sample_points(const char *file_name, double (*points)[2], int *rows, int *columns) {
    FILE *file = fopen(file_name, "rb");
    if (file == NULL) {
        perror("Error opening file");
        exit(1);
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    // Calculate number of points
    int num_points = file_size / (2 * sizeof(double));
    *rows = *columns = sqrt(num_points);  // Assuming a square grid

    if (fread(points, sizeof(double), num_points * 2, file) != num_points * 2) {
        perror("Error reading sample points");
        fclose(file);
        exit(1);
    }

    fclose(file);
}

void init_data_injestion_2d(DataInjestionData2D *data, double (*points)[2], size_t rows, size_t cols) {
    real_1d_array x, y;
    real_2d_array f;

    x.setlength(cols); // Assuming uniform grid in x
    y.setlength(rows); // Assuming uniform grid in y
    f.setlength(rows, cols);

    // Populate x, y, and f with your data
    for (size_t i = 0; i < rows; ++i) {
        y[i] = i; // Replace with actual y coordinates if they are non-uniform
        for (size_t j = 0; j < cols; ++j) {
            x[j] = j; // Replace with actual x coordinates if they are non-uniform
            f[i][j] = points[i][j]; // Assuming points contain height data
        }
    }

    // Initialize the spline
    spline2dbuildbicubic(x, y, f, (ae_int_t)rows, (ae_int_t )cols, data->spline);
    printf("Made it here...\n");

    data->width = cols;
    data->height = rows;
}

Point2D interpolate_coordinates_2d(DataInjestionData2D *data, double x, double y) {
    if (x < 0 || x >= data->width || y < 0 || y >= data->height) {
        fprintf(stderr, "Coordinates out of range\n");
        return (Point2D){-1, -1};
    }
    double value = spline2dcalc(data->spline, x, y);
    return (Point2D){value, 0};  // Adjust according to your requirements
}

void approximate_jacobian(DataInjestionData2D *data, double x, double y, double delta, double jacobian[2][2]) {
    // Approximate partial derivatives
    // dLat/dx and dLon/dx
    // Example values for width and height
    
    Point2D resultA = interpolate_coordinates_2d(data, x + delta, y);
    Point2D resultB = interpolate_coordinates_2d(data, x - delta, y);
    double lat1 = resultA.x;
    double lat2 = resultB.x;
    double lon1 = resultA.y;
    double lon2 = resultB.y;

    jacobian[0][0] = (lat1 - lat2) / (2 * delta);
    jacobian[1][0] = (lon1 - lon2) / (2 * delta);

    // dLat/dy and dLon/dy
    resultA = interpolate_coordinates_2d(data, x, y + delta);
    resultB = interpolate_coordinates_2d(data, x, y - delta);
    lat1 = resultA.x;
    lat2 = resultB.x;
    lon1 = resultA.y;
    lon2 = resultB.y;

    jacobian[0][1] = (lat1 - lat2) / (2 * delta);
    jacobian[1][1] = (lon1 - lon2) / (2 * delta);
}

Point2D inverse_interpolate_coordinates(DataInjestionData2D *data, double target_lat, double target_lon) {
    double tolerance = 1e-13;
    int max_iterations = 1000;

    // Initial guesses
    double x0 = 0.0, y0 = 0.0;
    double x1 = 1.0, y1 = 1.0;

    for (int i = 0; i < max_iterations; ++i) {
        Point2D interp0 = interpolate_coordinates_2d(data, x0, y0);
        Point2D interp1 = interpolate_coordinates_2d(data, x1, y1);

        double fx0 = interp0.x - target_lat;
        double fy0 = interp0.y - target_lon;
        double fx1 = interp1.x - target_lat;
        double fy1 = interp1.y - target_lon;

        // Check if we are close enough
        if (fabs(fx1) < tolerance && fabs(fy1) < tolerance) {
            return (Point2D){x1, y1};
        }

        double dx = x1 - x0;
        double dy = y1 - y0;
        double dfx = fx1 - fx0;
        double dfy = fy1 - fy0;

        if (fabs(dfx) < 1e-12 || fabs(dfy) < 1e-12) {
            fprintf(stderr, "Secant method failed to converge (division by zero)\n");
            return (Point2D){-1, -1};
        }

        // Secant method update
        double newX = x1 - fx1 * dx / dfx;
        double newY = y1 - fy1 * dy / dfy;

        // Update for next iteration
        x0 = x1;
        y0 = y1;
        x1 = newX;
        y1 = newY;
    }

    fprintf(stderr, "Secant method failed to converge\n");
    return (Point2D){-1, -1};
}


// The initialization function
DataInjestionData2D initialization(char* region) {
    DataInjestionData2D data;
    memset(&data, 0, sizeof(DataInjestionData2D));  // Initialize the data to zero

    // Construct the filename for sample points
    char filename[256];
    snprintf(filename, sizeof(filename), "%s_samplepoints.bin", region);

    // Read sample points from the file
    double points[1024][2]; // Array to hold 9 sample points (latitude, longitude)
    double num_points = 1024;
    int rows = 0;
    int cols = 0;
    read_sample_points(filename, points, &rows, &cols);
    data.width = rows;
    data.height = cols;
    // Initialize data injestion with the read sample points
    init_data_injestion_2d(&data, points, rows, cols);

    return data;
}

int main() {
    char user_input[] = "n43w074_25";
    DataInjestionData2D data = initialization(user_input);

    // Example usage of the initialized data
    double x = 3612-1, y = 0; // Example coordinates for interpolation
    // Example values for width and height
    Point2D interpolated = interpolate_coordinates_2d(&data, x, y);
    printf("Interpolated Latitude: %f, Longitude: %f\n", interpolated.x, interpolated.y);

    // Example usage of inverse interpolation
    Point2D inverse = inverse_interpolate_coordinates(&data, interpolated.x, interpolated.y);
    if (inverse.x != -1 && inverse.y != -1) {
        printf("Inverse Interpolated x: %f, y: %f\n", inverse.x, inverse.y);
    }

    return 0;
}