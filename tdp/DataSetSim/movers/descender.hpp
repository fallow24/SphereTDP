#ifndef DESCENDER_H
#define DESCENDER_H 

#include "mover.hpp"

class Descender: public Mover
{
public:
    Descender(vector<double> init_pose,
          double rot_rate,
          double desc_rate,
          vector<double> pose_noise_mean,
          vector<double> pose_noise_std): Mover(init_pose)
    {
        this->desc_rate = desc_rate;
        this->rot_rate = rot_rate;
        this->rpose_distribution =
            normal_distribution<double>(pose_noise_mean[0], pose_noise_std[0]);
        this->hpose_distribution =
            normal_distribution<double>(pose_noise_mean[1], pose_noise_std[1]);
    }
    ~Descender(){};

    double get_rot_rate(){return this->rot_rate;}
    double get_desc_rate(){return this->desc_rate;}

    virtual void move(double dt);
private:
    // Mersenne twister PRNG, initialized with seed from previous
    // random device instance
    mt19937 randomness_source;
    normal_distribution<double> rpose_distribution;
    normal_distribution<double> hpose_distribution;

    double desc_rate;
    double rot_rate;
    virtual double get_rpose_noise();
    virtual double get_hpose_noise();
};

#endif //DESCENDER_H