#include "descender.hpp"

void Descender::move(double dt)
{
    vector<double> curr_real_pose = this->get_real_pose();
    vector<double> curr_assumed_pose = this->get_assumed_pose();
    double angle_increment = this->get_rot_rate()*dt;
    double descend_increment = this->get_desc_rate()*dt;

    /* We descend only in y direction and rotate about the y axis
     * noise occures by increasing or decreasing the expection
     * rotation and descend by a random factor */

    curr_real_pose[1] -= descend_increment*
                           (1+this->get_hpose_noise());
    curr_real_pose[4] = to_pi_range(curr_real_pose[4] +
                angle_increment*(1+this->get_rpose_noise()));

    curr_assumed_pose[1] -= descend_increment;
    curr_assumed_pose[4] = to_pi_range(curr_assumed_pose[4] + angle_increment);

    this->set_real_pose(curr_real_pose);
    this->set_assumed_pose(curr_assumed_pose);
}

double Descender::get_rpose_noise()
{
    // This randomly sampled value is a percentage of the
    // actually desired movement increment
    return this->rpose_distribution(this->randomness_source);
}

double Descender::get_hpose_noise()
{
    // This randomly sampled value is a percentage of the
    // actually desired movement increment
    return this->hpose_distribution(this->randomness_source);
}

