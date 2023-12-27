#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "helper.h"  // Assuming helper.c contains the run_wind_aware function and necessary data structures
#include "demo.h"
#include "dubins.h"
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "test.h"
#include <omp.h>

#define MAX_CONFIG_SIZE 100
#define MAX_POINTS 1000  // Adjust as necessary

double*** run_paths(double** q0s, double** q1s, int number_points) {
    FILE *file = fopen("config.txt", "r");
    if (file == NULL) {
        perror("Error opening file");
        return NULL;
    }

    double configArray[15];  // Declare a global copy of configArray
    int configSize = 0;
    while (fscanf(file, "%lf,", &configArray[configSize]) == 1) {
        configSize++;
    }
    fclose(file);
    double rho = 1.0;

    // Allocate memory for keep_paths
    double*** keep_paths = (double***)malloc(number_points * sizeof(double**));
    if (keep_paths == NULL) {
        perror("Memory allocation failed for keep_paths");
        return NULL;
    }

    // Initialize keep_paths to NULL
    for (int i = 0; i < number_points; ++i) {
        keep_paths[i] = NULL;
    }

    // Get the number of available CPU cores
    int num_cores = omp_get_num_procs();
    printf("Number of CPU cores: %d\n", num_cores);

    // Set the number of threads
    omp_set_num_threads(num_cores);

    // Enable parallel execution
    #pragma omp parallel
    {
        Seg2 local_seg2;
        double** local_merged_array;
        double** local_final_array;

        // Distribute the workload among the threads
        #pragma omp for
        for (int k = 0; k < number_points; k++) {
            double* q0 = q0s[k];
            double* q1 = q1s[k];
            local_seg2 = calculate_trajectory(q0, q1, configArray);
            local_merged_array = segment_to_4D_array(&local_seg2);
            local_final_array = linearInterpolation(local_merged_array);
            // Store only the last path
            if (k == 0) {
                #pragma omp critical
                {
                    keep_paths[k] = local_final_array;
                }
            } else {
                // Free intermediate paths to avoid memory leaks
                for (int j = 1; local_final_array[j] != NULL; j++) {
                    free(local_final_array[j]);
                }
                // Free intermediate paths to avoid memory leaks
                for (int j = 1; local_merged_array[j] != NULL; j++) {
                    free(local_merged_array[j]);
                }
                free(local_final_array);
                free(local_merged_array);
            }
        }
    }

    // Placeholder
    return keep_paths;
}
