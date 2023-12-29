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

Result* run_paths(double** q0s, double** q1s, int number_points) {
    FILE *file = fopen("path_generation/config.txt", "r");
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

    double EMERGENCY_ALTITUDE = configArray[6] / 364173.0; // in lat/long units
    double BEST_GLIDE_RATIO = configArray[7];
    double DIRTY_CONFIG_GLIDE_RATIO = configArray[8];
    double BEST_GLIDING_AIRSPEED = configArray[9]; // in knots
    double BANK_ANGLE = configArray[10];
    double WIND_HEADING = configArray[11];
    double WIND_SPEED = configArray[12];
    double INTERVAL = configArray[13]; // use 0.001 for A320 and 0.0001 for c172
    double TURN_RADIUS = (BEST_GLIDING_AIRSPEED*BEST_GLIDING_AIRSPEED)/(11.29* tan(30*PI/180))/364173.0; 

    // Allocate memory for keep_paths
    Result* keep_paths = (Result*)malloc(number_points * sizeof(Result));
    if (keep_paths == NULL) {
        perror("Memory allocation failed for keep_paths");
        return NULL;
    }

    // Get the number of available CPU cores
    int num_cores = omp_get_num_procs();
    printf("Number of CPU cores: %d\n", num_cores);

    // Set the number of threads
    omp_set_num_threads(num_cores);

    clock_t start, end;
    start = clock();

    #pragma omp parallel
    {
        Result local_result;

        #pragma omp for
        for (int k = 0; k < number_points; k++) {
            double* q0 = q0s[k];
            double* q1 = q1s[k];
            
            local_result = demo(q0, q1, TURN_RADIUS, 0.001);
            #pragma omp critical
            {
                keep_paths[k] = local_result;
            }
        }
    }
    end = clock();

    double cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC / 12;
    printf("Total time elapsed: %f seconds\n", cpu_time_used);

    // Placeholder
    return keep_paths;
}
