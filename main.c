#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <math.h>
#include "helper.h" 
#include "wind_path_generator.h"

int main() {
    int number = 10000;

    // Allocate memory for 'number' of double pointers
    double** q0_ptrs = (double**)malloc(number * sizeof(double*));
    double** q1_ptrs = (double**)malloc(number * sizeof(double*));

    if (!q0_ptrs || !q1_ptrs) {
        perror("Failed to allocate memory");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < number; ++i) {
        q0_ptrs[i] = (double*)malloc(3 * sizeof(double));
        q1_ptrs[i] = (double*)malloc(3 * sizeof(double));

        if (!q0_ptrs[i] || !q1_ptrs[i]) {
            perror("Failed to allocate memory");
            exit(EXIT_FAILURE);
        }

        q0_ptrs[i][0] = -73.8767;
        q0_ptrs[i][1] = 40.8513;
        q0_ptrs[i][2] = 1.5586;

        q1_ptrs[i][0] = -73.8571;
        q1_ptrs[i][1] = 40.7721;
        q1_ptrs[i][2] = 2.3736;
    }

    double*** path_results = run_paths(q0_ptrs, q1_ptrs, number);
    printf("Computed Paths..\n");

    print_path(path_results[0]);
    // Add here any necessary code to deal with 'path_results' if needed

    // Free allocated memory
    for (int i = 0; i < number; ++i) {
        free(q0_ptrs[i]);
        free(q1_ptrs[i]);
    }
    free(q0_ptrs);
    free(q1_ptrs);

    // After processing path_results
    for (int i = 0; i < number - 1; ++i) {
        if (path_results[i]) {
            for (int j = 0; path_results[i][j] != NULL; j++) {
                free(path_results[i][j]);
            }
            free(path_results[i]);
        }
    }
    free(path_results);
    return 0;
}
