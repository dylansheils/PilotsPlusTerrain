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

//main EnTRY function
Seg basic_path(Packet data)
{
    Packet pack; //for storing complete path
 
        int i, limit,j,k;
        
//unpacking packet
        double q1[3];
    double q2[3];
    double min_radius=data.min_rad;
    double start_altitude=data.start_altitude;
    int angle=data.angle;
    double WIND_VELOCITY=data.windspeed;
    double baseline_g=data.baseline_g;

        for(i=0;i<3;i++)
    {
        q1[i]=data.p1[i];
        q2[i]=data.p2[i];
    }
    
    Result dubins=demo(q1,q2,min_radius, data.interval); //sending configs to demo to generate DP
        //dubins.arr =[long, lat, heading, ?unknown]

    Seg dubin_parts=split(dubins); //sending DP to split into segmentss

    dubin_parts=assign_altitude(dubin_parts, start_altitude, q1[0], q1[1], angle, data.baseline_g);//Send dubin_parts to assign_altitude() to get alti for each point

    //generates possible spiral segment with altitude
    Seg path_with_spiral= generate_spiral(dubin_parts,min_radius,angle,data.baseline_g);

    path_with_spiral= find_extended_runway(path_with_spiral,q2[0],q2[1],q2[2],q1[0],q1[1],q1[2],start_altitude, angle, min_radius, data.interval, data.baseline_g, data.dirty_g); //finds extended runway
    print_trajectory(path_with_spiral, angle, q2[0],q2[1],q2[2]); //saving to file
    
//  printf("No wind trajectory generated!\n");
    return path_with_spiral;
}

Seg2 model_wind(Seg path_with_spiral, Packet data)
{
        int i, limit,j,k;
        
//unpacking packet
        double q1[3];
    double q2[3];
    double min_radius=data.min_rad;
    double start_altitude=data.start_altitude;
    int angle=data.angle;
    double WIND_VELOCITY = data.windspeed;
    double WIND_HEADING =  data.wind_heading;
    double baseline_g=data.baseline_g;

        for(i=0;i<3;i++)
    {
        q1[i]=data.p1[i];
        q2[i]=data.p2[i];
    }
    
    Seg2 wind_path; 
    wind_path.spiral=false;
    wind_path.extended=false;
    wind_path.end_alt=0.0;

        Curve augmented_curve_A= wind_curveA(path_with_spiral,WIND_HEADING, WIND_VELOCITY, OMEGA_30_DEGREE_BANK, min_radius,start_altitude, q1[0], q1[1], angle, baseline_g, data.airspeed); //send first curve to be modified by wind
    wind_path.aug_C1=augmented_curve_A;

    Curve augmented_SLS= wind_SLS(path_with_spiral,WIND_HEADING, WIND_VELOCITY,augmented_curve_A,baseline_g, data.airspeed, data.dirty_g); //send middle straight line segment to be modified
    wind_path.aug_SLS=augmented_SLS;    
    Curve augmented_curve_B= wind_curveB(path_with_spiral,WIND_HEADING, WIND_VELOCITY,OMEGA_30_DEGREE_BANK, min_radius, augmented_SLS, angle,baseline_g, data.airspeed,q2[2]); //send second curve to be modified
    wind_path.aug_C2=augmented_curve_B;
    Curve augmented_spiral;
    augmented_spiral.spiral=false;
    Curve augmented_extended;
    augmented_extended.extended=false;

    if(path_with_spiral.lenspiral>0) //augmenting spiral
    {
        augmented_spiral= wind_spiral(path_with_spiral,WIND_HEADING, WIND_VELOCITY,OMEGA_30_DEGREE_BANK, min_radius, augmented_curve_B, angle,baseline_g, data.airspeed,q2[2]);
        wind_path.aug_SPIRAL=augmented_spiral;
        wind_path.spiral=true;
    }
    if(path_with_spiral.extended) //augmenting extended runway
    {
        augmented_extended= wind_extended(path_with_spiral,WIND_HEADING, WIND_VELOCITY, augmented_spiral, augmented_curve_B, q2[0],q2[1],q2[2],baseline_g, data.airspeed, data.dirty_g);
        wind_path.aug_EXTENDED=augmented_extended;
        wind_path.extended=true;

    }
    save_wind_in_file(augmented_curve_A,  augmented_SLS, augmented_curve_B, augmented_spiral, augmented_extended, data.file_name, data.alphabet);//saves augmented path in file

    //calculate total shift in path
    if (wind_path.extended)
    {
        wind_path.total_shift= horizontal(data.runway[0], data.runway[1], wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve-1][0], wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve-1][1]);  
        wind_path.end_alt=wind_path.aug_EXTENDED.points[wind_path.aug_EXTENDED.len_curve-1][4];
    }
    else
    {
        if(wind_path.spiral)
        {
            wind_path.total_shift= horizontal(data.runway[0], data.runway[1], wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve-1][0], wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve-1][1]);
            wind_path.end_alt=wind_path.aug_SPIRAL.points[wind_path.aug_SPIRAL.len_curve-1][4];     
        }
        else
        {
            wind_path.total_shift= horizontal(data.runway[0], data.runway[1], wind_path.aug_C2.points[wind_path.aug_C2.len_curve-1][0], wind_path.aug_C2.points[wind_path.aug_C2.len_curve-1][1]);
            wind_path.end_alt=wind_path.aug_C2.points[wind_path.aug_C2.len_curve-1][4];     
    
        }
    }

//  printf("Wind modelled! \n");
    return wind_path;        
}