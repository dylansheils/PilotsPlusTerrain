#include "demo.h"
#include "helper.h"
#include <stdio.h>
#include <dirent.h> 
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include "test.h"

void print_packet(const Packet* dat) {
    if (dat == NULL) {
        printf("Packet is NULL.\n");
        return;
    }

    printf("Interval: %f\n", dat->interval);
    printf("Start Altitude: %f\n", dat->start_altitude);
    printf("Windspeed: %f\n", dat->windspeed);
    printf("Wind Heading: %f\n", dat->wind_heading);
    printf("Airspeed: %f\n", dat->airspeed);
    printf("Baseline G: %f\n", dat->baseline_g);
    printf("Dirty G: %f\n", dat->dirty_g);
    printf("Angle: %d\n", dat->angle);
    printf("Minimum Radius: %f\n", dat->min_rad);
    printf("File Name: %d\n", dat->file_name);
    printf("Alphabet: %c\n", dat->alphabet);

    printf("p1: [");
    for (int i = 0; i < 3; i++) {
        printf("%f", dat->p1[i]);
        if (i < 2) printf(", ");
    }
    printf("]\n");

    printf("p2: [");
    for (int i = 0; i < 3; i++) {
        printf("%f", dat->p2[i]);
        if (i < 2) printf(", ");
    }
    printf("]\n");

    printf("Runway: [");
    for (int i = 0; i < 3; i++) {
        printf("%f", dat->runway[i]);
        if (i < 2) printf(", ");
    }
    printf("]\n");
}

// Main entry function for calculating the basic path
Seg basic_path(Packet* data) {
	if (data == NULL) {
    	fprintf(stderr, "Error: Packet data is null.\n");
    	exit(EXIT_FAILURE);
	}

	double p1[3];
	double p2[3];
	for(int i = 0; i < 3; i++) {
		p1[i] = data->p1[i];
		p2[i] = data->p2[i];
	}
    // Generate Dubins path based on provided data
	Result dubins = demo(p1, p2, data->min_rad, data->interval); // Directly passing q1 and q2

    // Split Dubins path into segments
    Seg dubin_parts = split(dubins);

    // Assign altitude to each segment
    dubin_parts = assign_altitude(dubin_parts, data->start_altitude, p1[0], p2[1], data->angle, data->baseline_g);

    // Generate possible spiral segment with altitude
    Seg path_with_spiral = generate_spiral(dubin_parts, data->min_rad, data->angle, data->baseline_g);

    // Find extended runway
    path_with_spiral = find_extended_runway(path_with_spiral, p2[0], p2[1], p2[2], p1[0], p1[1], p1[2], data->start_altitude, data->angle, data->min_rad, data->interval, data->baseline_g, data->dirty_g);

    // Print the trajectory and save it to a file
    //print_trajectory(path_with_spiral, data->angle, p2[0], p2[1], p2[2]);

    // Return the final path with spiral
    return path_with_spiral;
}

// Function for modeling the wind effect on the path
Seg2 model_wind(Seg path_with_spiral, Packet* data) {
    // Unpacking the packet
    double q1[3], q2[3];
    for (int i = 0; i < 3; i++) {
        q1[i] = data->p1[i];  // Accessing elements of p1
        q2[i] = data->p2[i];  // Accessing elements of p2
    }

    Seg2 wind_path;
    wind_path.spiral = false;
    wind_path.extended = false;
    wind_path.end_alt = 0.0;

    Curve augmented_curve_A = wind_curveA(path_with_spiral, data->wind_heading, data->windspeed, OMEGA_30_DEGREE_BANK, data->min_rad, data->start_altitude, q1[0], q1[1], data->angle, data->baseline_g, data->airspeed);
    wind_path.aug_C1 = augmented_curve_A;

    Curve augmented_SLS = wind_SLS(path_with_spiral, data->wind_heading, data->windspeed, augmented_curve_A, data->baseline_g, data->airspeed, data->dirty_g);
    wind_path.aug_SLS = augmented_SLS;

    Curve augmented_curve_B = wind_curveB(path_with_spiral, data->wind_heading, data->windspeed, OMEGA_30_DEGREE_BANK, data->min_rad, augmented_SLS, data->angle, data->baseline_g, data->airspeed, q2[2]);
    wind_path.aug_C2 = augmented_curve_B;

    // Declare augmented_spiral and augmented_extended outside of the if blocks
    Curve augmented_spiral;
    augmented_spiral.spiral = false;
    Curve augmented_extended;
    augmented_extended.extended = false;

    if (path_with_spiral.lenspiral > 0) {
        augmented_spiral = wind_spiral(path_with_spiral, data->wind_heading, data->windspeed, OMEGA_30_DEGREE_BANK, data->min_rad, augmented_curve_B, data->angle, data->baseline_g, data->airspeed, q2[2]);
        wind_path.aug_SPIRAL = augmented_spiral;
        wind_path.spiral = true;
    }

    if (path_with_spiral.extended) {
        augmented_extended = wind_extended(path_with_spiral, data->wind_heading, data->windspeed, augmented_spiral, augmented_curve_B, q2[0], q2[1], q2[2], data->baseline_g, data->airspeed, data->dirty_g);
        wind_path.aug_EXTENDED = augmented_extended;
        wind_path.extended = true;
    }

    if (wind_path.extended) {
        wind_path.total_shift = horizontal(data->runway[0], data->runway[1], wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve - 1][0], wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve - 1][1]);
        wind_path.end_alt = wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve - 1][4];
    } else if (wind_path.spiral) {
        wind_path.total_shift = horizontal(data->runway[0], data->runway[1], wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve - 1][0], wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve - 1][1]);
        wind_path.end_alt = wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve - 1][4];
    } else {
        wind_path.total_shift = horizontal(data->runway[0], data->runway[1], wind_path.aug_C2.points[wind_path.aug_C2.len_curve - 1][0], wind_path.aug_C2.points[wind_path.aug_C2.len_curve - 1][1]);
        wind_path.end_alt = wind_path.aug_C2.points[wind_path.aug_C2.len_curve - 1][4];
    }

    return wind_path;
}

Seg2 calculate_trajectory(double q1[3], double q2[3], double configArray[15]) {
    // Validation for input arrays
    for (int i = 0; i < 3; i++) {
        if (q1[i] == 0 || q2[i] == 0) {
            fprintf(stderr, "Error: Invalid data in q1 or q2.\n");
            fflush(stderr);
            exit(EXIT_FAILURE);
        }
    }

    double EMERGENCY_ALTITUDE = configArray[6] / 364173.0; // in lat/long units
    double BEST_GLIDE_RATIO = configArray[7];
    double DIRTY_CONFIG_GLIDE_RATIO = configArray[8];
    double BEST_GLIDING_AIRSPEED = configArray[9]; // in knots
    double BANK_ANGLE = configArray[10];
    double WIND_HEADING = configArray[11];
    double WIND_SPEED = configArray[12];
    double INTERVAL = configArray[13]; // use 0.001 for A320 and 0.0001 for c172

    Packet* dat = malloc(sizeof(Packet));
    if (dat == NULL) {
        fprintf(stderr, "Failed to allocate memory for Packet.\n");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < 3; i++) {
        dat->p1[i] = q1[i];
        dat->p2[i] = q2[i];
        dat->runway[i] = q2[i];
    }
    dat->interval = INTERVAL;
    dat->start_altitude = EMERGENCY_ALTITUDE;
    dat->windspeed = WIND_SPEED * 1.68781 / 364173.0;
    dat->wind_heading = WIND_HEADING;
    dat->airspeed = BEST_GLIDING_AIRSPEED * 1.68781 / 364173.0;
    dat->baseline_g = BEST_GLIDE_RATIO;
    dat->dirty_g = DIRTY_CONFIG_GLIDE_RATIO;
    dat->angle = configArray[14];
    dat->min_rad = (BEST_GLIDING_AIRSPEED * BEST_GLIDING_AIRSPEED) / (11.29 * tan(dat->angle * M_PI / 180)) / 364173.0;

    Seg basic_trajectory = basic_path(dat);
    Seg2 wind_1 = model_wind(basic_trajectory, dat);

    free(dat); // Free the allocated memory

    return wind_1;
}