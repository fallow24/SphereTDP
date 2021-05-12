/*----------------------------------
    Code by Jasper Zevering
    jasper.zevering@stud-mail.uni-wuerzburg.de
    31.01.2021
---------------------------------*/

#include "imuJasper.h"


//DEFINING EXTERN VARIABLES OF HEADER
//quiet mode
bool quiet = false;
//gain of madgwickfilter
float gain_ = DEFAULT_MADGWICK_GAIN;
//gain of complemnetary
float alpha = 0.02;
//Mechnaism for checking if everything works
int firstRead = 0;
//slow-mode ( (g_new = (g_old +g_0+ g_1 + g_2)/4))
bool slow = false;
// shoulr roll pitch yw in degrees be printed every loop
bool printRPY = false;
//
bool debugMode = false;
//
int data_rate = DATA_RATE_DEFAULT;
//
float data_intervall = 1 / DATA_RATE_DEFAULT;
//
int imu_data_rate = IMU_DATA_RATE_DEFAULT;
//gravity ing
float gravityValue = 1;
//automaticgain
float autogain = 0;
//float radius in m
float r = 0.29;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

//Handler for data from spatial0
void CCONV onSpatial0_SpatialData(PhidgetSpatialHandle
                            ch,
                            void *ctx,
                            const double acceleration[3],
                            const double angularRate[3],
                            const double magneticField[3],
                            double timestamp
) {
    if (!quiet) { ROS_INFO("Getting Data"); }
    int serialNr;
    int spatialNr;

    float *gxTemp;
    float *gyTemp;
    float *gzTemp;
    float *axTemp;
    float *ayTemp;
    float *azTemp;

    Phidget_getDeviceSerialNumber((PhidgetHandle)
                                          ch, &serialNr);
    switch (serialNr) {
        case SERIAL_0:
            spatialNr = 0;
            gxTemp = &gx0;
            gyTemp = &gy0;
            gzTemp = &gz0;
            axTemp = &ax0;
            ayTemp = &ay0;
            azTemp = &az0;
            break;
        case SERIAL_1:
            spatialNr = 1;
            gxTemp = &gx1;
            gyTemp = &gy1;
            gzTemp = &gz1;
            axTemp = &ax1;
            ayTemp = &ay1;
            azTemp = &az1;
            break;
        case SERIAL_2:
            spatialNr = 2;
            gxTemp = &gx2;
            gyTemp = &gy2;
            gzTemp = &gz2;
            axTemp = &ax2;
            ayTemp = &ay2;
            azTemp = &az2;
            break;
        default:
            ROS_WARN("Attention! IMU with serial %d gives data bug was not defined as one of teh three in header!",
                     serialNr);

    }

    if (!quiet) {
        ROS_INFO("----------Spatial %d ----------\n", spatialNr);
        ROS_INFO("Acceleration: \t%lf  |  %lf  |  %lf\n", acceleration[0], acceleration[1], acceleration[2]);
        ROS_INFO("AngularRate: \t%lf  |  %lf  |  %lf\n", angularRate[0], angularRate[1], angularRate[2]);
        ROS_INFO("MagneticField: \t%lf  |  %lf  |  %lf\n", magneticField[0], magneticField[1], magneticField[2]);
        ROS_INFO("Timestamp: %lf\n", timestamp);
        ROS_INFO("----------\n");
    }

    if (firstRead == spatialNr) {


        firstRead++;
        ROS_INFO("Receiving from IMU %d works! Serial: %d", spatialNr, serialNr);
    }


    *
            gxTemp = angularRate[0] * M_PI_BY_180;
    *
            gyTemp = angularRate[1] * M_PI_BY_180;
    *
            gzTemp = angularRate[2] * M_PI_BY_180;
    *
            axTemp = acceleration[0];
    *
            ayTemp = acceleration[1];
    *
            azTemp = acceleration[2];


}
//Handler for data from spatial1

// we have to invert the gyro data as imus using not teh rigth handed conversion ut left handed. thefore the y axis is inverted (leading to gy noz havign to be converted)
int combineRAWData() {

    ax = ax0;
    ay = -ay0;
    az = az0;
    if (firstRead < 4) {
        gx = -(gx1 + gx2 + gx0) * ONE_THIRD;
        gy = (gy1 + gy2 + gy0) * ONE_THIRD;
        gz = -(gz1 + gz2 + gz0) * ONE_THIRD;

        ovrwrtOrientWithAcc(ax, ay, az, 0);
        firstRead = 4;

    } else if (!slow) {
        gx = -(gx1 + gx2 + gx0) * ONE_THIRD;
        gy = (gy1 + gy2 + gy0) * ONE_THIRD;
        gz = -(gz1 + gz2 + gz0) * ONE_THIRD;

    } else {

        gx = -(gx1 + gx2 + gx0 + gx) * 0.25;
        gy = (gy1 + gy2 + gy0 + gy) * 0.25;
        gz = -(gz1 + gz2 + gz0 + gz) * 0.25;

    }


}


void madgwick_and_complementary_Filter() {

    ros::Time tmp = ros::Time::now();
    dt = (tmp.toNSec() - lastTime.toNSec()) * 0.000000001;


    lastTime = tmp;
    //if dt is 10% longer then it should be, we consider Lacking
    if (dt > data_intervall * 1.1) { ROS_WARN("LACKING, cannot hold up with set Data-Rate!"); }
    if (dt <= 0) {
        dt = data_intervall;
        ROS_WARN("Problem with dt, using 1/datarate");
    }


//Helper variables
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

// Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);





//calculating the factors descriign if there is active rotation.
    float factorX = (0.5 * gx) * (0.5 * gx);
    float factorY = (0.5 * gy) * (0.5 * gy);
    float factorZ = (0.5 * gz) * (0.5 * gz);
    if (factorX > 1)factorX = 1;
    if (factorY > 1)factorY = 1;
    if (factorZ > 1)factorZ = 1;


    //adapt gains automatically
    if (!autogain <= 0) {
        float maxFac = std::max(std::max(factorY, factorZ), factorX);
        if (maxFac < 0.1)maxFac = 0;
        //alpha is one magnitude less, as the standard values for both are 0.2 and 0.02
        gain_ = autogain * maxFac;
        alpha = autogain * 0.1 * (1 - maxFac);


    }



// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
// Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

//------------------------
//az=1;

// Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

// Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

// Apply feedback step 
        qDot1 -= gain_ * s0;
        qDot2 -= gain_ * s1;
        qDot3 -= gain_ * s2;
        qDot4 -= gain_ * s3;
    }

// Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

// Normalise quaternion
    normalizeQuat(&q0, &q1, &q2, &q3);

//Complementary Filter




    float acc_roll = atan2f(ay, az);
    float acc_pitch = atan2f(-ax, fast_sqrt(ay * ay + az * az));
//get RPY from quaternion

//acc_roll= atan(copysign(1.0,az)*ay/sqrt(0.01*ax*ax+az*az));


    float roll, pitch, yaw;
    eulerFromQuat(&roll, &pitch, &yaw, q0, q1, q2, q3);
//the actual complementary step

//gimbal lock avoidence
    if (abs(q2 * q1 + q0 * q3) - 0.5 < 0.01) {
        acc_roll = roll;
    }


    float q00, q11, q22, q33;
    quatFromEuler(&q00, &q11, &q22, &q33, acc_roll, acc_pitch, yaw);

    quaternion qAcc, qCur, qNex;

    qAcc.w = q00;
    qAcc.x = q11;
    qAcc.y = q22;
    qAcc.z = q33;

    qCur.w = q0;
    qCur.x = q1;
    qCur.y = q2;
    qCur.z = q3;

    quaternion_slerp(&qCur, &qAcc, &qNex, alpha);

    q0 = qNex.w;
    q1 = qNex.x;
    q2 = qNex.y;
    q3 = qNex.z;

    normalizeQuat(&q0, &q1, &q2, &q3);;

    eulerFromQuat(&roll, &pitch, &yaw, q0, q1, q2, q3);

    if (printRPY || !quiet) {
        ROS_INFO("Roll: %f Pitch: %f Yaw: %f, just ACC: Roll:%f, Pitch %f", roll * precalc_180_BY_M_PI,
                 pitch * precalc_180_BY_M_PI,
                 yaw * precalc_180_BY_M_PI, acc_roll * precalc_180_BY_M_PI, acc_pitch * precalc_180_BY_M_PI);
    }




//rotation matrix (from robot to world)

    //First row of the rotation matrix
    float r00 = 1 - 2 * (q2 * q2 + q3 * q3);
    float r01 = 2 * (q1 * q2 - q0 * q3);
    float r02 = 2 * (q1 * q3 + q0 * q2);

    // Second row of the rotation matrix
    float r10 = 2 * (q1 * q2 + q0 * q3);
    float r11 = 1 - 2 * (q1 * q1 + q3 * q3);
    float r12 = 2 * (q2 * q3 - q0 * q1);

    // Third row of the rotation matrix
    float r20 = 2 * (q1 * q3 - q0 * q2);
    float r21 = 2 * (q2 * q3 + q0 * q1);
    float r22 = 1 - 2 * (q1 * q1 + q2 * q2);



    //gravitational vector : rotate 001 from world ointo robot
    float grav_x = r20;
    float grav_y = r21;
    float grav_z = r22;


    vel_x += (ax - grav_x) * dt;
    vel_y += (ay - grav_y) * dt;
    vel_z += (az - grav_z) * dt;
    if (debugMode)
        ROS_INFO("acceleration is  %f and %f and %f , corrected: %f and %f and %f  ", ax, ay, az, (ax - grav_x),
                 (ay - grav_y), (az - grav_z));
    //couple it to the rotation speed!
    //rotation around x enables translation-verlocity  in y and z direction (robot frame)
    vel_x *= std::max(factorY, factorZ);
    vel_y *= std::max(factorZ, factorX);
    vel_z *= std::max(factorX, factorY);
    //integrate velocity to the points an rotate into world frame


    //use this matrix tranformed to get back rotation
    float vel_x_world = r00 * vel_x + r01 * vel_y + r02 * vel_z;
    float vel_y_world = r10 * vel_x + r11 * vel_y + r12 * vel_z;
    float vel_z_world = r20 * vel_x + r21 * vel_y + r22 * vel_z;


    float rot_x_world = r00 * gx + r01 * gy + r02 * gz;
    float rot_y_world = r10 * gx + r11 * gy + r12 * gz;
    //we do not nee rot aroudn z as it would not lead to translation in our opinion
    //float rot_z_world = m[0][2] * gx + m[1][2] * gy + m[2][2] * gz;
    //the velocity is gyro_measurm01t/2/PI*r*2*PI = gyroMeasument*r
    float vel_x_world_rot = rot_y_world * r;
    float vel_y_world_rot = -rot_x_world * r;

//limit velocity_z to the mean og vel_x by to and vel_y y rot. As Otherwise exponential error integration could happen.
    float mean_vel_world_XY = fast_sqrt(vel_x_world_rot * vel_x_world_rot + vel_y_world_rot * vel_y_world_rot);
    if (abs(vel_z_world) > mean_vel_world_XY) vel_z_world = std::copysign(1.0, vel_z_world) * mean_vel_world_XY;

//to prevent uncontrollable expoential error interation when rolling over a long time in one direction (the factor_x for exapmple is 1, thefore the original problem with acceöerometer integration occurs)we limit the velocity to 110% of the corressponending velocity by rotation.
    if (abs(vel_x_world) > abs(vel_x_world_rot) * 1.1)
        vel_x_world =
                std::copysign(1.0, vel_x_world) * abs(vel_x_world_rot) *
                1.1;
    if (abs(vel_y_world) > abs(vel_y_world_rot) * 1.1)
        vel_y_world =
                std::copysign(1.0, vel_y_world) * abs(vel_y_world_rot) *
                1.1;


//subtract velocity in z (othwerwie we would roll through the obstacle rather then over it). if vz^2 is bigger then the rot velocity, we should take 0. because it somehow means we are falling (mor translation in z than possible by rotation). The square method takes care of this. 
    float vz2 = vel_z_world * vel_z_world;
    vel_x_world_rot = std::copysign(1.0, vel_x_world_rot) * fast_sqrt(vel_x_world_rot * vel_x_world_rot - vz2);
    vel_y_world_rot = std::copysign(1.0, vel_y_world_rot) * fast_sqrt(vel_y_world_rot * vel_y_world_rot - vz2);
    px += (vel_x_world + vel_x_world_rot) * 0.5 * dt;
    py += (vel_y_world + vel_y_world_rot) * 0.5 * dt;
    pz += vel_z_world * dt;

}

int argumentHandler(int argc, char *argv[]) {

    for (int i = 1; i < argc; ++i) {
        std::string argi = argv[i];
        if (argi.compare("-q") == 0) {
            quiet = true;
            ROS_INFO(
                    "Quiet Mode activated! No receiing data or repetetive Messages will be shown. Start-up information, Errors and warning etc will be shown!!\n");
        }

        if (argi.compare("-gain") == 0) {
            if (i + 1 < argc) {
                i++;
                gain_ = atof(argv[i]);
                ROS_INFO("gain set to %f\n", gain_);

            }
        }
        if (argi.compare("-alpha") == 0) {
            if (i + 1 < argc) {
                i++;
                alpha = atof(argv[i]);
                ROS_INFO("alpha for compelmentary  set to %f\n", alpha);

            }
        }
        if (argi.compare("-autogain") == 0) {
            if (i + 1 < argc) {
                i++;
                autogain = atof(argv[i]);
                gain_ = 0;
                alpha = autogain * 0.1;
                ROS_INFO("autogain set to %f\n. Thefore gain_ is set to 0 and alpha initally to %f", autogain,
                         autogain * 0.1);

            }
        }

        if (argi.compare("-slow") == 0) {
            slow = true;
            ROS_INFO("slow mode (g_new = (g_old +g_0+ g_1 + g_2)/4)\n");
        }
        if (argi.compare("-rate") == 0) {
            if (i + 1 < argc) {
                i++;
                int temp = atoi(argv[i]);
                if (temp > 0 && temp < 501) {
                    data_rate = temp;
                    data_intervall = 1.0 / temp;
                    ROS_INFO("data rate set to %d\n", data_rate);
                } else {
                    ROS_WARN("data rate allowed between 1 and 500. Stays default at %d", DATA_RATE_DEFAULT);
                }
            }
        }
        if (argi.compare("-printRPY") == 0) {
            printRPY = true;
            ROS_INFO("Printing Roll Pitch Yaw activated");
        }


        if (argi.compare("-imu_rate") == 0) {
            if (i + 1 < argc) {
                i++;
                int temp = atoi(argv[i]);
                if (temp > 0 && temp < 251) {
                    imu_data_rate = temp;
                    ROS_INFO("IMU data rate set to %d\n", imu_data_rate);
                } else {
                    ROS_WARN("IMU _ data rate allowed between 1 and 250. Stays default at %d", IMU_DATA_RATE_DEFAULT);
                }
            }
        }
        if (argi.compare("-debug") == 0) {
            debugMode = true;
        }

    }
}

void CCONV
detachHandler(PhidgetHandle
ch,
void *ctx
){
int serialNr;
Phidget_getDeviceSerialNumber((PhidgetHandle)
ch, &serialNr);
ROS_ERROR("IMU with Serial Nr: %d detached!!!", serialNr);

}


int CCONV

init() {


    ROS_INFO("Initalizing");

    ROS_INFO("REsetting Phidgets");
    Phidget_resetLibrary();


    output_msg.header.frame_id = "map";
    PhidgetSpatialHandle
            spatial0, spatial1, spatial2;

    //Create your Phidget channels
    PhidgetSpatial_create(&spatial0);
    PhidgetSpatial_create(&spatial1);
    PhidgetSpatial_create(&spatial2);

    //Set addressing parameters to specify which channel to open (if any)
    Phidget_setDeviceSerialNumber((PhidgetHandle)
    spatial0, 415233);
    Phidget_setDeviceSerialNumber((PhidgetHandle)
    spatial1, 297652);
    Phidget_setDeviceSerialNumber((PhidgetHandle)
    spatial2, 425377);

    //set the handler which handels detaching evcents
    Phidget_setOnDetachHandler((PhidgetHandle)
    spatial0, detachHandler, NULL);
    Phidget_setOnDetachHandler((PhidgetHandle)
    spatial1, detachHandler, NULL);
    Phidget_setOnDetachHandler((PhidgetHandle)
    spatial2, detachHandler, NULL);

    ROS_INFO("Now Attaching the IMUS! Giving it maximum 3 seconds!");
    //Open your Phidgets and wait for attachment
    ros::Time begin = ros::Time::now();
    Phidget_openWaitForAttachment((PhidgetHandle)
    spatial0, 1000);
    Phidget_openWaitForAttachment((PhidgetHandle)
    spatial1, 1000);
    Phidget_openWaitForAttachment((PhidgetHandle)
    spatial2, 1000);
    if (ros::Time::now().toSec() - begin.toSec() >= 3) {
        ROS_WARN(
                "Not looking good! Either slow Attachment or Timeout. Connection still possible, lets wait for receiving data!");
    } else {
        ROS_INFO(
                "Attaching sucessfull. Note: This does not mean we receive data, just that we found the IMU. Data receiving is checked in the next steps. ");
    }

    int imu_data_intervall = std::round(1000.0 / imu_data_rate);
    //Set the data rate (set by user or IMU_DEFAULT_DATA_RATE
    PhidgetSpatial_setDataInterval(spatial0, imu_data_intervall);
    PhidgetSpatial_setDataInterval(spatial1, imu_data_intervall);
    PhidgetSpatial_setDataInterval(spatial2, imu_data_intervall);

    ROS_WARN("ZEROING, DO NOT MOVE IMUs!!!");
    ros::Duration(0.5).sleep();
    PhidgetSpatial_zeroGyro(spatial0);
    PhidgetSpatial_zeroGyro(spatial1);
    PhidgetSpatial_zeroGyro(spatial2);
    PhidgetSpatial_zeroAlgorithm(spatial0);
    PhidgetSpatial_zeroAlgorithm(spatial1);
    PhidgetSpatial_zeroAlgorithm(spatial2);
    ROS_INFO("Zeroing done!");

    //Assign any event handlers you need before calling open so that no events are missed.
    PhidgetSpatial_setOnSpatialDataHandler(spatial0, onSpatial0_SpatialData, NULL);
    PhidgetSpatial_setOnSpatialDataHandler(spatial1, onSpatial0_SpatialData, NULL);
    PhidgetSpatial_setOnSpatialDataHandler(spatial2, onSpatial0_SpatialData, NULL);
    //initilizind data for filtering
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;


    vel_x = 0;
    vel_y = 0;
    vel_z = 0;
    //initilaize output message
    output_msg.pose.position.x = 0;
    output_msg.pose.position.y = 0;
    output_msg.pose.position.z = 0;
    output_msg.pose.orientation.x = q1;
    output_msg.pose.orientation.y = q2;
    output_msg.pose.orientation.z = q3;
    output_msg.pose.orientation.w = q0;


    ROS_INFO("Initalisation done");


}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "imuHandling", ros::init_options::NoSigintHandler);
    ROS_INFO("Starting Node");
    if (argc > 1) {
        argumentHandler(argc, argv);
    }
    signal(SIGINT, mySigIntHandler);

    ros::NodeHandle nH;
    init();


    ros::Publisher pose_pub = nH.advertise<geometry_msgs::PoseStamped>("posePub_merged", 1000);

    ros::Rate loop_rate(data_rate);

    //wait for every imu to send data
    while (firstRead < 3 && !g_request_shutdown) {
        ROS_WARN("Waiting for all IMUs to deliever data");
        ros::Rate loop_rate_StartUp(1);
        loop_rate_StartUp.sleep();
    }
    //the gyro send right after starting just the value  0 for 1-3 seconds. We dont need that
    while (gx0 == 0 && gx1 == 0 && gx2 == 0 && !g_request_shutdown) {
        ROS_INFO("Waiting for all gyros to start giving data (thy gyros need a little bit of time to start up)");

        ros::Rate loop_rate_StartUp(1);
        loop_rate_StartUp.sleep();

    }
    lastTime = ros::Time::now();

    ROS_INFO("SUCCESS!!! Everything from Start-Up (including initalizations) done, starting Loop!");
    while (ros::ok() && !g_request_shutdown) {


        combineRAWData();
        madgwick_and_complementary_Filter();

        output_msg.pose.position.x = px;
        output_msg.pose.position.y = py;
        output_msg.pose.position.z = pz;
        output_msg.pose.orientation.w = q0;
        output_msg.pose.orientation.x = q1;
        output_msg.pose.orientation.y = q2;
        output_msg.pose.orientation.z = q3;

        pose_pub.publish(output_msg);

        ros::spinOnce();
        loop_rate.sleep();
        if (debugMode)debugMessage(1);


    }

    ROS_INFO("CLOSING EVERYTHING, GOODBYE!");
    Phidget_resetLibrary();

    ros::shutdown();


}

//no freaking idea how, but it works, thanks quake 3 arena!
float invSqrt(float f) {
    float halff = 0.5f * f;
    float y = f;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halff * y * y));
//we take the secodn iteration, as the overall calculation time is so big, that we can take this little step for better performance 
    y = y * (1.5f - (halff * y * y));
    return y;
}

//fast sqrt function
float fast_sqrt(const float &f) {
    if (f <= 0)return 0;
    union {
        int i;
        float n;
    } u;
    u.i = 0x2035AD0C + (*(int *) &f >> 1);
    return f / u.n + u.n * 0.25f;
}

void debugMessage(int place) {
    ROS_INFO("----------------------------------");
    ROS_INFO("This is a debug message from place %d", place);

    ROS_INFO("q0: %f", q0);

    ROS_INFO("q1: %f", q1);

    ROS_INFO("q2: %f", q2);

    ROS_INFO("q3: %f", q3);

    ROS_INFO("px: %f", px);

    ROS_INFO("py: %f", py);
    ROS_INFO("pz: %f", pz);
    ROS_INFO("dt: %f", dt);
    ROS_INFO("gx: %f; gy: %f; gz: %f ", gx, gy, gz);

    ROS_INFO("gx0: %f; gy0: %f; gz0: %f ", gx0, gy0, gz0);
    ROS_INFO("gx1: %f; gy1: %f; gz1: %f ", gx1, gy1, gz1);
    ROS_INFO("gx2: %f; gy2: %f; gz1: %f ", gx2, gy2, gz2);
    ROS_INFO("ax: %f; ay: %f; az: %f ", ax, ay, az);
    ROS_INFO("ax0: %f; ay0: %f; az0: %f ", ax0, ay0, az0);
    ROS_INFO("ax1: %f; ay1: %f; az1: %f ", ax1, ay1, az1);
    ROS_INFO("ax2: %f; ay2: %f; az2: %f ", ax2, ay2, az2);
}

// Replacement SIGINT handler
void mySigIntHandler(int sig) {
    g_request_shutdown = 1;
}

void quatFromEuler(float *qW, float *qX, float *qY, float *qZ, float roll, float pitch, float yaw) {

    yaw *= 0.5;
    pitch *= 0.5;
    roll *= 0.5;

    float cpsi = cos(yaw);
    float spsi = sin(yaw);
    float cth = cos(pitch);
    float sth = sin(pitch);
    float cphi = cos(roll);
    float sphi = sin(roll);

    *qW = cpsi * cth * cphi + spsi * sth * sphi;
    *qX = cpsi * cth * sphi - spsi * sth * cphi;
    *qY = cpsi * sth * cphi + spsi * cth * sphi;
    *qZ = spsi * cth * cphi - cpsi * sth * sphi;

    normalizeQuat(qW, qX, qY, qZ);
}

void eulerFromQuat(float *roll, float *pitch, float *yaw, float qW, float qX, float qY, float qZ) {

    *roll = atan2f(qW * qX + qY * qZ, 0.5f - qX * qX - qY * qY);
    *pitch = asinf(-2.0f * (qX * qZ - qW * qY));
    *yaw = atan2f(qX * qY + qW * qZ, 0.5f - qY * qY - qZ * qZ);


}

void normalizeQuat(float *q0, float *q1, float *q2, float *q3) {


    float recipNorm = invSqrt(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    *q0 *= recipNorm;
    *q1 *= recipNorm;
    *q2 *= recipNorm;
    *q3 *= recipNorm;


}

void ovrwrtOrientWithAcc(float ax, float ay, float az, float yaw) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);

    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    float acc_roll = atan2(ay, az);
    float acc_pitch = atan2(-ax, fast_sqrt(ay * ay + az * az));

    quatFromEuler(&q0, &q1, &q2, &q3, acc_roll, acc_pitch, yaw);
    ROS_INFO("Initial: Roll: %f Pitch: %f Yaw: %f", acc_roll * precalc_180_BY_M_PI, acc_pitch * precalc_180_BY_M_PI,
             yaw * precalc_180_BY_M_PI);

}

void quaternion_slerp(quaternion *l, quaternion *r, quaternion *o, double weight) {
    double dot = quaternion_dot_product(l, r);
    if (dot > 0.9995) {
        o->w = l->w + (r->w - l->w) * weight;
        o->x = l->x + (r->x - l->x) * weight;
        o->y = l->y + (r->y - l->y) * weight;
        o->z = l->z + (r->z - l->z) * weight;
        quaternion_normalize(o);
        return;
    }

    if (dot > 1)
        dot = 1;
    else if (dot < -1)
        dot = -1;

    double theta_0 = acos(dot);
    double theta = (0. < theta_0 && theta_0 < M_PI_2) ? theta_0 * weight : (theta_0 - M_PI) * weight;

    o->w = r->w - l->w * dot;
    o->x = r->x - l->x * dot;
    o->y = r->y - l->y * dot;
    o->z = r->z - l->z * dot;

    quaternion_normalize(o);

    o->w = l->w * cos(theta) + o->w * sin(theta);
    o->x = l->x * cos(theta) + o->x * sin(theta);
    o->y = l->y * cos(theta) + o->y * sin(theta);
    o->z = l->z * cos(theta) + o->z * sin(theta);
}

double quaternion_dot_product(quaternion *l, quaternion *r) {
    return l->w * r->w + l->x * r->x + l->y * r->y + l->z * r->z;
}

void quaternion_normalize(quaternion *q) {
    double norm = invSqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm == 0)
        norm = 0.0000001;
    q->w *= norm;
    q->x *= norm;
    q->y *= norm;
    q->z *= norm;
}

void quaternion_copy(quaternion *original_q, quaternion *copy_q) {
    copy_q->w = original_q->w;
    copy_q->x = original_q->x;
    copy_q->y = original_q->y;
    copy_q->z = original_q->z;
}
