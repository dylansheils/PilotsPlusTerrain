#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>
#include "helper.h" 
#include "wind_path_generator.h"

#define MAX_POINTS 2048  // Maximum number of points
#define POINT_DIM 3     // Dimension of each point

int main() {
    int number = MAX_POINTS;  // The number of points to read

    // Open the binary files
    FILE *file_q0 = fopen("path_generation/q0s.bin", "rb");
    FILE *file_q1 = fopen("path_generation/q1s.bin", "rb");
    if (!file_q0 || !file_q1) {
        perror("Failed to open file");
        exit(EXIT_FAILURE);
    }

    // Allocate memory on the stack for 'number' of double pointers
    double q0_array[MAX_POINTS][POINT_DIM];
    double q1_array[MAX_POINTS][POINT_DIM];
    double* q0_ptrs[MAX_POINTS];
    double* q1_ptrs[MAX_POINTS];

    for (int i = 0; i < number; ++i) {
        q0_ptrs[i] = q0_array[i];
        q1_ptrs[i] = q1_array[i];

        // Read points from the binary files
        fread(q0_ptrs[i], sizeof(double), POINT_DIM, file_q0);
        fread(q1_ptrs[i], sizeof(double), POINT_DIM, file_q1);
    }

    // Close the files
    fclose(file_q0);
    fclose(file_q1);

    printf("Running paths...\n");

    Result* results = run_paths(q0_ptrs, q1_ptrs, number);
    printf("Computed Paths..\n");

    FILE *file = fopen("path_generation/paths.bin", "wb");
    if (!file) {
        perror("Failed to open output file");
        exit(EXIT_FAILURE);
    }

    printf("Saving....\n");
    for (int i = 0; i < number; ++i) {
        if(results[i].len < 0) {
            printf("Impossible length...\n");
        }
        if (fwrite(&(results[i].len), sizeof(results[i].len), 1, file) != 1) {
            perror("Error writing length to file");
            fclose(file);
            exit(EXIT_FAILURE);
        }

        for (int j = 0; j < results[i].len; ++j) {
            if (fwrite(results[i].arr[j], sizeof(results[i].arr[0]), 1, file) != 1) {
                perror("Error writing array to file");
                fclose(file);
                exit(EXIT_FAILURE);
            }
        }
    }

    free(results);

    fclose(file);
    return 0;
}