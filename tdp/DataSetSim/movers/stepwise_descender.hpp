#ifndef STEPWISE_DESCENDER_H
#define STEPWISE_DESCENDER_H

#include "mover.hpp"

class Stepwise_Descender: public Mover
{
public:
    Stepwise_Descender(vector<double> init_pose,
          double rot_rate,
          double desc_rate,
          double desc_step,
          vector<double> pose_noise_mean,
          vector<double> pose_noise_std): Mover(init_pose)
    {
        this->desc_rate = desc_rate;
        this->desc_step = desc_step;
        this->rot_rate = rot_rate;
        this->rpose_distribution =
            normal_distribution<double>(pose_noise_mean[0], pose_noise_std[0]);
        this->hpose_distribution =
            normal_distribution<double>(pose_noise_mean[1], pose_noise_std[1]);
    }
    ~Stepwise_Descender(){};

    double get_rot_rate(){return this->rot_rate;}
    double get_desc_rate(){return this->desc_rate;}
    double get_desc_step(){return this->desc_step;}
    bool get_state(){return this->rotating;}

    void toggle_state();
    virtual void move(double dt);

private:
    // Mersenne twister PRNG, initialized with seed from previous
    // random device instance
    mt19937 randomness_source;
    normal_distribution<double> rpose_distribution;
    normal_distribution<double> hpose_distribution;

    double desc_rate;
    double rot_rate;
    double desc_step;
    double last_height;
    // True == we want to rotate, false == we want to descend
    bool rotating =  true;

    virtual double get_rpose_noise();
    virtual double get_hpose_noise();
};

#endif //STEPWISE_DESCENDER_H