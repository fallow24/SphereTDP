#ifndef IMU_MERGER_H
#define IMU_MERGER_H



#include <stdio.h>
#include <stdlib.h>
#include <phidget22.h>

#include <unistd.h>


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <signal.h>

#define SERIAL_0 415233
#define SERIAL_1 297652
#define SERIAL_2 425377

#define DATA_RATE_DEFAULT 250
#define IMU_DATA_RATE_DEFAULT 250

#define DEFAULT_MADGWICK_GAIN 0.1


//often used calculations
#define M_PI_BY_180 0.017453292
#define precalc_180_BY_M_PI 57.29577951
#define ONE_THIRD 0.333333333


typedef struct quaternion
{
    double  w;
    double  x;
    double  y;
    double  z;
};


extern bool quiet;
//actual data rate of publishing. will be set to DATA_RATE_DEFAULT if not else stated
extern int data_rate;

geometry_msgs::PoseStamped output_msg;


//orientation
float q0; 
float q1; 
float q2; 
float q3; 
float px;
float py;
float pz;
//gain of madgwickfilter
extern float gain_; 
//gain of complemnetary
extern float alpha;
//gyroscope
float gx; 
float gy; 
float gz; 
float gx1;
float gy1;
float gz1;
float gx2;
float gy2;
float gz2;
float gx0;
float gy0;
float gz0;
//accelerometer
float ax; 
float ay; 
float az;
float ax1;
float ay1;
float az1;
float ax2;
float ay2;
float az2;
float ax0;
float ay0;
float az0; 

float vel_x;
float vel_y;
float vel_z;


float dt; 

extern int firstRead;

extern bool slow;

ros::Time lastTime;

//Functions
//fast algorithm for inverse square root
float invSqrt(float f);
//fast algorithm for sqrtr
float fast_sqrt(const float& f);
//initalizing procedure
int CCONV init();
//handlign arguments giving to programm
int argumentHandler(int argc, char *argv[]);
//main filter function
void madgwick_and_complementary_Filter();

//listing most pof parameters for debugging
void debugMessage(int place);
//for safe shutdown
 void mySigIntHandler(int sig);
// calculate qauternion an write it into the qW to qZ
void quatFromEuler(float *qW, float *qX, float *qY, float *qZ, float roll, float pitch, float yaw);
//calculate euler an write it into yaw pitch roll
void eulerFromQuat(float *roll, float *pitch, float *yaw, float qW, float qX, float qY, float qZ);
//normalize quaternione with the help of inverse suqareroot
void normalizeQuat(float *q0,float *q1, float *q2, float *q3);
//ovewrite q0-q3 with an estimation of accelerometer and the desired yaw (can not be set from accelerometer
void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw);
//quaternion operations
void quaternion_slerp(quaternion *l, quaternion *r, quaternion *o, double weight);
double quaternion_dot_product(quaternion *l, quaternion *r);
void quaternion_normalize(quaternion *q);
void quaternion_copy(quaternion *original_q, quaternion *copy_q);
#endif
