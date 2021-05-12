#include <iostream>
#include "../constants.hpp"
#include "livoxMid40.hpp"

void LivoxMid40::get_sensing_pattern(vector<pair<double, double> > &pattern,
                                     double dt)
{
    if(dt > MIN_SIM_RESOLUTION && VERBOSITY_LEVEL == 1)
    {
        cout << "WARNING: for the Livox Mid 40 scanner "
             << "you should choose a simulation resolution "
             << "that is smaller than " << MIN_SIM_RESOLUTION << endl;
    }

    /* Get half the fov, as only half is required to find
     * the factor to multply the sinusoidal function with to achieve
     * the extend of the measurement
     */
    double elev = deg_to_rad(get<0>(this->get_fov()))*0.5;
    double azi = deg_to_rad(get<1>(this->get_fov()))*0.5;

    /* During each simulation time step dt, we need
     * to subdivide to find the livox pattern traced
     * during this timestep.
     * In 1s the livox collects 100,000 pts, therefor in
     * dt it collects pts = dt * 100,000 / 1s.
     * Therefore the sim_step should be 1/100,000
     */
    double sim_step = 0.00001;
    /* The rotational speeds omega1 and omega2 as well as their
     * rates of change are chosen such that we have 5 "Leaves" and
     * within 0.5 s we have ~50% coverage, within 1 second we have ~95%
     * coverage as specified by the livox scanner specs
     * The r values are the maximal distance, and are computed from the FOV that
     * is given.
     */
    double s = 100.0, n = sqrt(20),
           fov_elev_factor = tan(elev), fov_azi_factor = tan(azi);

    /* Adding all the elev azimuth pairs to the pattern */
    for(double t = this->internal_time; t < this->internal_time + dt; t += sim_step)
    {
        double elev = fov_elev_factor*atan2(sin(s*(n*t + 0.25*M_PI))*cos(s*t), 1);
        double azi  = fov_azi_factor*atan2(sin(s*(n*t + 0.25*M_PI))*sin(s*t), 1)
                      +  this->azimuth_offset;
        pattern.push_back(make_pair(rad_to_deg(elev),
                                    rad_to_deg(azi)));
    }
    /* Increase the internal time such that we don't start at the same
     * spot each timestep */
    this->internal_time += dt;
}
