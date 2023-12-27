#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

struct Point2D {
    double x, y;
};

class DataInjestionData2D {
public:
    Eigen::MatrixXd latMatrix, lonMatrix;
    size_t width, height;

    DataInjestionData2D(size_t rows, size_t cols)
        : width(cols), height(rows), latMatrix(rows, cols), lonMatrix(rows, cols) {}

    void setDataAt(size_t row, size_t col, double lat, double lon) {
        if (row < height && col < width) {
            latMatrix(row, col) = lat;
            lonMatrix(row, col) = lon;
        }
    }

    Point2D interpolate(double xFraction, double yFraction) {
        double x = xFraction * (width - 1);
        double y = yFraction * (height - 1);

        size_t x0 = std::min(static_cast<size_t>(x), width - 1);
        size_t y0 = std::min(static_cast<size_t>(y), height - 1);
        size_t x1 = std::min(x0 + 1, width - 1);
        size_t y1 = std::min(y0 + 1, height - 1);

        double dx = x - x0;
        double dy = y - y0;

        double lat = (1 - dx) * (1 - dy) * latMatrix(y0, x0) + dx * (1 - dy) * latMatrix(y0, x1) +
                     (1 - dx) * dy * latMatrix(y1, x0) + dx * dy * latMatrix(y1, x1);
        double lon = (1 - dx) * (1 - dy) * lonMatrix(y0, x0) + dx * (1 - dy) * lonMatrix(y0, x1) +
                     (1 - dx) * dy * lonMatrix(y1, x0) + dx * dy * lonMatrix(y1, x1);

        return Point2D{lat, lon};
    }
};

void readSamplePoints(const std::string& fileName, std::vector<Point2D>& points, int& rows, int& cols) {
    std::ifstream file(fileName, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        exit(1);
    }

    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    int numPoints = fileSize / (2 * sizeof(double));
    rows = cols = std::sqrt(numPoints);
    points.resize(numPoints);

    for (auto& point : points) {
        file.read(reinterpret_cast<char*>(&point.x), sizeof(double));
        file.read(reinterpret_cast<char*>(&point.y), sizeof(double));
    }
}

bool read2DArrayFromFile(const std::string& fileName, std::vector<std::vector<int16_t>>& array) {
    std::ifstream file(fileName, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return false;
    }

    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    int size = static_cast<int>(std::sqrt(fileSize / sizeof(int16_t)));
    array.resize(size, std::vector<int16_t>(size));

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (!file.read(reinterpret_cast<char*>(&array[i][j]), sizeof(int16_t))) {
                std::cerr << "Error reading file: " << fileName << std::endl;
                return size;
            }
        }
    }

    return size;
}


int main() {
    std::string fileName = "n43w074_25_samplepoints.bin";
    std::vector<Point2D> points;
    int rows, cols;
    readSamplePoints(fileName, points, rows, cols);

    fileName = "n43w074_25_elevation.bin";
    std::vector<std::vector<int16_t>> array;
    int temp;
    double size = (double)read2DArrayFromFile(fileName, array);

    DataInjestionData2D data(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            data.setDataAt(i/rows, j/cols, (double)points[i * cols + j].x, (double)points[i * cols + j].y);
        }
    }

    // Adjust the input coordinates for interpolation
    double inputX = 3000;  // This is now considered as an absolute position
    double inputY = 0;
    printf("%f", size);

    Point2D interpolated = data.interpolate(inputX/size, inputY/size);
    std::cout << "Interpolated Latitude: " << interpolated.x 
              << ", Longitude: " << interpolated.y << std::endl;


    return 0;
}
