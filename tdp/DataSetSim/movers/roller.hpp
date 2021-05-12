#ifndef ROLLER_H
#define ROLLER_H 

#include "mover.hpp"

class Roller: public Mover
{
public:
    Roller(vector<double> init_pose,
          double radius,
          vector<double> pose_noise_mean,
          vector<double> pose_noise_std): 
        Mover(init_pose),
        radius(radius),
        rot_rate(M_PI_4), // rad/s
        curr_x_dist_ang_vel(0),
        curr_z_dist_ang_vel(0) 
    {
        if(VERBOSITY_LEVEL == 1)
        {
            cout << "Initializing random distributions:" << endl;
            cout << "Pose noise about X: mean " << pose_noise_mean[0] 
                 << " std: " << pose_noise_std[0] << "\n"
                 << "Pose noise about Z: mean " << pose_noise_mean[1]
                 << " std: " << pose_noise_std[1] << endl;  
        }
        this->xpose_distribution =
            normal_distribution<double>(pose_noise_mean[0], pose_noise_std[0]);
        this->zpose_distribution =
            normal_distribution<double>(pose_noise_mean[1], pose_noise_std[1]);            
    }
    ~Roller(){};

    virtual void move(double dt);
private:
    // Mersenne twister PRNG, initialized with seed from previous
    // random device instance
    mt19937 randomness_source;
    normal_distribution<double> xpose_distribution;
    normal_distribution<double> zpose_distribution;

    double radius;
    double rot_rate;
    double curr_x_dist_ang_vel;
    double curr_z_dist_ang_vel;
    virtual double get_xpose_noise();
    virtual double get_zpose_noise();
};

#endif //ROLLER_H