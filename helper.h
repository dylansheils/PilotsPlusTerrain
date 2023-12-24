#ifndef _HELPER_H_
#define _HELPER_H_

#include "demo.h"
#include <stdbool.h>

#define ARRSIZE(arr) (sizeof(arr)/sizeof(*(arr))) //for quicksort

#define FILL(arr, val) \
for(int i_##arr = 0; i_##arr < sizeof arr / sizeof *arr; ++i_##arr) \
{ \
    arr[i_##arr][3] = val;\
}

//-----------------------Mathematical constants
#define PI 3.14159265
#define KNOTS_TO_METERS_PER_SEC 1.68781
#define METERS_TO_LATLONG_UNITS 364173.0
#define DEG_TO_RAD (M_PI / 180.0)


//-----------------------Sensor data that can be computed by the DDDAS approach 
//#define Rg_straight 17.25 //Best Glide ratio in straight line motion
//#define Rg_dirty 9
                                        // cvarela: using g' = g_0*cos(bank_angle) glide ratio
//#define Rg_20 Rg_straight*cos(20*PI/180) //Best Glide ratio for 20 degree banked turns
//#define Rg_30 Rg_straight*cos(30*PI/180) //Best Glide ratio for 30 degree banked turns
//#define Rg_45 Rg_straight*cos(45*PI/180) //Best Glide ratio for 45 degree banked turns

#define OMEGA_30_DEGREE_BANK 0.2832 //rate of change of heading in standard rate turn for 30 degree bank angle =0.2832 rad/second expressed in units

//#define WIND_HEADING 3.14159265 //in radians wrt EAST
//#define WIND_HEADING 0.0 //in radians wrt EAST
//#define WIND_HEADING 3.14159265/2.0 //in radians wrt EAST
//#define WIND_HEADING (3.14159265/2.0)*3.0 //in radians wrt EAST
//-----------------------Structures that are used 

typedef struct Pair //used to hold new points along heading
{
	double x;
 	double y;
	double heading;
}Pair;

//for any curve
typedef struct Curve 
{
	double points[1000][5]; //x,y,heading,radius,altitude
	int len_curve; 
	int turns;

	bool spiral;
	bool extended;
	Pair centre;

	double shift; //stores the shift of end where needed
	char instructions[1000];
}Curve;

//collection of curves
typedef struct Seg2 
{
	Curve aug_C1;
        Curve aug_SLS;
        Curve aug_C2;
	Curve aug_SPIRAL;
	Curve aug_EXTENDED;
        bool spiral;
        bool extended;
	double total_shift;
	double end_alt;
}Seg2;

//for spiral
typedef struct Circle 
{
	double circum[1000][5]; //x,y,heading,radius,altitude
	int len_c; 
	int turns;
}Circle;

//for discrete thetas
typedef struct Thetas 
{
	double thetas[500]; //an angle
}Thetas;


typedef struct Packet //used to send initial and start points to new thread and to store a path
{	
	double interval; //interval of points in Dubins Path. SENT TO demo()!!
	double p1[3]; //x,y,heading initial config
	double p2[3]; //final config for each call
	double runway[3]; //config of runway
	double current_path[8000][5]; //x,y,heading,radius,altitude
	int len_cp; 

	//emergency variables
	int angle;
	double min_rad;
	double start_altitude;     
	double airspeed;
	double windspeed;
	double wind_heading;
	double baseline_g;
	double dirty_g;

	//misc
	int file_name;
        char alphabet;

}Packet;



typedef struct Seg //to store segments of a dubins path
{
	double C1[1000][5]; //x,y,heading,radius,altitude
	int lenc1;
	double SLS[1000][5];
	int lensls;
	double C2[1000][5];
	int lenc2;	
	double Spiral[1000][5];
	int lenspiral;
	Pair spiral_centre;
	double spiral_start_angle;

	bool extended;
}Seg;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} Point4D;


//-----------------------Helper functions

//for inplace sorting by qsort() in thread function
int compare(const void *a, const void *b);

//finds distance of a point from a line
double pointdist(double a, double b, double c, int x, int y);

//finds the position of a point wrt a line.  
double position(double A, double B, double C, double x, double y);

//finds a new pont along a given heading 
Pair along_heading(double x0, double y0, double theta);

//finds a new pont along a given heading at a given distance
Pair along_heading_at_distance(double x0, double y0, double theta, double distance);

//finds centre of turn given starting point x0,y0; heading of starting point theta and radius of turn.   
Pair circle_centre(double x0, double y0, double theta, double radius);

//returns heading of line between two points in radians
double heading(double x1, double y1, double x2, double y2);

//spits path into segments and returns them.
Seg split(Result dub);

//Mathematical function that calculates Horizontal Distance between two points
double horizontal(double x1, double y1, double x2, double y2);

//Mathematical function that calculates Horizontal Distance between two points in 3D
double distance_3d(double x1, double y1, double z1, double x2, double y2, double z2);

//Mathematical function that takes in heading WRT E=0 in radian and returns heading WRT N=0 in degrees
double azmth(double E_rad);

//Mathematical function that calculates new altitude for straight line motion
double heightS(double last_height, double distance, double Rg_straight);

//Mathematical function that calculates new altitude for 30 degree banked turns
double heightBC(double last_height, double distance, int angle, double Rg_straight);

//Assigns altitude at each point of DP
Seg assign_altitude(Seg parts, double last_height, double last_x, double last_y, int angle, double Rg_straight);

//new print function
void print_trajectory(Seg path, int angle, double rnwy_x, double rnwy_y, double rnwy_heading);

//Fixes altitude difference at end
Seg fix_difference(Seg parts, double diff);

//generates circles for spiral
Seg generate_spiral(Seg path, double radius, int angle, double Rg_straight);

//Function to find extended runway segment
Seg find_extended_runway(Seg path, double rnwy_x, double rnwy_y, double rnwy_heading, double init_x, double init_y, double init_heading, double init_altitude , int angle, double min_radius, double interval, double Rg_straight,double Rg_dirty);

//-------------- Functions for modelling wind --------------------------------
//Funtion that finds inscribed angle of C1
double inscribed_c1(Seg path, double radius);

//Funtion that finds inscribed angle of C2
double inscribed_c2(Seg path, double radius);

//Function that finds centre of an arc given three points on that arc
Pair find_centre(double x1, double y1, double x2, double y2, double x3, double y3);

//function to determine rletive position of a point wrt centre shift line
double pos_wrt_centre(Pair initial_centre, Pair point, double wind_heading);

//Functions that determines if turns are clockwise or counterclockwise
int orientation(double x1, double y1, double x2, double y2, double x3, double y3);

//Function to generate 100 discrete theta's for the first curve
Thetas generate_thetasA(Seg path, Pair initial_centre, double radius);

//Function to generate 100 discrete theta's for the first curve
Thetas generate_thetasB(Seg path, Pair initial_centre, double radius);

//Function that augments first 2d curve of Dubins for wind and returns a modified 2d curve
//omega= rate of change of heading in standard rate turn
Curve wind_curveA(Seg path,double wind_heading,double wind_velocity,double omega, double radius, double start_altitude, double initial_x, double initial_y, int angle, double baseline_g, double airspeed ); 

//Function that modifies a straight line segment to reflect effect of wind on a line segment
Curve wind_SLS(Seg path,double wind_heading,double wind_velocity, Curve augmented_curve_A, double baseline_g, double airspeed,double Rg_dirty); 

//Function that finds the centre of turn (without wind) given original dubins, (x,y,heading) of augmented SLS and radius of turn
Pair find_clockwise_centre(double x0, double y0, double heading, double radius);

//Function that augments second 2d curve of Dubins for wind and returns a modified 2d curve
Curve wind_curveB(Seg path, double wind_heading, double wind_velocity, double omega, double radius, Curve augmented_SLS, int angle, double baseline_g, double airspeed, double rnwy_heading);

//Function to mdel effect of wind on spiral
Curve wind_spiral(Seg path,double wind_heading, double  wind_velocity, double omega, double radius, Curve augmented_curve_B, int angle, double baseline_g, double airspeed, double rnwy_heading);

//Function to model effect of wind on extended segment
Curve wind_extended(Seg path_with_spiral,double wind_heading,double wind_velocity,Curve augmented_spiral,Curve augmented_C2,double rnwy_x,double rnwy_y,double rnwy_heading, double baseline_g, double airspeed,double Rg_dirty);

//function that calculates altitude for curves with wind
double curve_altitude(double last_height, double distance, int angle, double airspeed_heading, double wind_heading,double wind_velocity, double baseline_g, double airspeed);

//Mathematical function that calculates new altitude for lines with wind
double line_altitude(double last_height, double distance, double airspeed_heading, double wind_heading, double wind_velocity, bool extended, double baseline_g, double airspeed ,double Rg_dirty);

//Function that assigns altitudes to the points in augmented segments
void assign_wind_altitude(Curve augmented_curve_A, Curve augmented_SLS, Curve augmented_curve_B, double start_altitude, double last_x, double last_y, int angle, double wind_heading, double wind_velocity);

//function that saves augmented path in file
void save_wind_in_file(Curve augmented_curve_A, Curve augmented_SLS, Curve augmented_curve_B, Curve augmented_spiral, Curve augmented_extended, int filename, char alphabet);

double** segment_to_4D_array(Seg2* seg2);

double** linearInterpolation(double** pointsMatrix);

void print_path(double** input_path);

void free_4d_array(double** array, int size);
//----------------------------------------------------------------------------------------------------------------------------------

#endif
