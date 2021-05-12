#include "stepwise_descender.hpp"

void Stepwise_Descender::move(double dt)
{
    vector<double> curr_real_pose = this->get_real_pose();
    vector<double> curr_assumed_pose = this->get_assumed_pose();

    // We're still in the rotating state
    if(this->get_state())
    {
        curr_real_pose[4] += this->get_rot_rate()*dt*
                                (1 + this->get_rpose_noise());
        curr_assumed_pose[4] += this->get_rot_rate()*dt;

        if(curr_assumed_pose[4] >= 2*M_PI)
        {
            this->toggle_state();
            curr_real_pose[4] = to_pi_range(curr_real_pose[4]);
            curr_assumed_pose[4] = to_pi_range(curr_assumed_pose[4]);
            this->last_height = curr_assumed_pose[1];
        }
    }
    else // We're descending
    {
        curr_real_pose[1] -= this->get_desc_rate()*dt*
                                (1+this->get_hpose_noise());
        curr_assumed_pose[1] -= this->get_desc_rate()*dt;

        if(abs(curr_assumed_pose[1] - this->last_height) >= this->get_desc_step())
        {
            this->toggle_state();
        }
    }

    this->set_real_pose(curr_real_pose);
    this->set_assumed_pose(curr_assumed_pose);
}


double Stepwise_Descender::get_rpose_noise()
{
    // This randomly sampled value is a percentage of the
    // actually desired movement increment
    return this->rpose_distribution(this->randomness_source);
}


double Stepwise_Descender::get_hpose_noise()
{
    // This randomly sampled value is a percentage of the
    // actually desired movement increment
    return this->hpose_distribution(this->randomness_source);
}


inline void Stepwise_Descender::toggle_state()
{
    bool temp = !this->get_state();
    this->rotating = temp;
}
