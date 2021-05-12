#include "roller.hpp"

/* We assume the noisy spherical robot mainly rotates about one axis.
 * At each timestep there is also a disturbance torque about the 
 * x and z axes (i.e. intended rotation axis and other ground rotation)
 * that is determined my a normal distribution N(mu,sigma). 
 * from this force we can compute the change in the current velocities
 * and then compute the pose by integrating.
 *
 * We assume the constant desired rotation rate is an eights rotation in 1 sec, i.e.#
 * about the x axis thus translating into the room towars the y-axis with
 *      pi/4 per s
 */
void Roller::move(double dt)
{
    vector<double> curr_real_pose = this->get_real_pose();
    vector<double> curr_assumed_pose = this->get_assumed_pose();
    double angle_increment = this->rot_rate*dt; // rad

    /* Update the current disturbance velocities by integrating
     * the sampled disturbance torque 
     * torque about the x-axis leads to angular velocity about x 
     * torque about the z-axis leads to angular velocity about z */ 
    this->curr_x_dist_ang_vel += this->get_xpose_noise()*dt; // in rad/s
    this->curr_z_dist_ang_vel += this->get_zpose_noise()*dt; // in rad/s
    
    /* We move in z and x direction. In z direction purposefully and randomly,
     * in x-direction only randomly
     * Doing a eights rotation per second
     */
    vector<double> real_delta_euler_angle =
            {angle_increment + this->curr_x_dist_ang_vel*dt,
            0,
            // This noise is one order of magnitude higher 
            this->curr_z_dist_ang_vel*dt};

    vector<double> assumed_delta_euler_angle =
            {angle_increment, 0, 0};

    // Transform Rotaion into robot frame, only needed if we want to rotate 
    // about world axis axis.
    //world2robot_frame(this->get_real_pose(), true, real_delta_euler_angle);
    //world2robot_frame(this->get_real_pose(), true, assumed_delta_euler_angle);
    
    // z-axis rotation corresponds to x-axis translation
    curr_real_pose[0] -= real_delta_euler_angle[2]*this->radius;
    curr_assumed_pose[0] -= assumed_delta_euler_angle[2]*this->radius;
    // x-axis rotation corresponds to z-axis tranlation
    curr_real_pose[2] += real_delta_euler_angle[0]*this->radius;
    curr_assumed_pose[2] += assumed_delta_euler_angle[0]*this->radius;
   // We have to make sure we stay within [-pi,pi]
    curr_real_pose[3] = to_pi_range(curr_real_pose[3] + real_delta_euler_angle[0]);
    curr_real_pose[5] = to_pi_range(curr_real_pose[5] + real_delta_euler_angle[2]);
    curr_assumed_pose[3] = to_pi_range(curr_assumed_pose[3] + assumed_delta_euler_angle[0]);
    curr_assumed_pose[5] = to_pi_range(curr_assumed_pose[5] + assumed_delta_euler_angle[2]);

    this->set_real_pose(curr_real_pose);
    this->set_assumed_pose(curr_assumed_pose);
}

double Roller::get_xpose_noise()
{
    // This randomly sampled value is a percentage of the
    // actually desired angle increment
    return this->xpose_distribution(this->randomness_source);
}

double Roller::get_zpose_noise()
{
    // This randomly sampled value is a percentage of the
    // actually desired angle increment
    return this->zpose_distribution(this->randomness_source);
}
