#include "livoxMid100.hpp"

void LivoxMid100::get_sensing_pattern(vector<pair<double, double> > &pattern,
                                      double dt)
{
    vector<pair<double,double> > scan1_pattern, scan2_pattern, scan3_pattern;
    this->scan1.get_sensing_pattern(scan1_pattern, dt);
    this->scan2.get_sensing_pattern(scan2_pattern, dt);
    this->scan3.get_sensing_pattern(scan3_pattern, dt);

    pattern.insert(pattern.end(), scan3_pattern.begin(), scan3_pattern.end());
    pattern.insert(pattern.end(), scan2_pattern.begin(), scan2_pattern.end());
    pattern.insert(pattern.end(), scan1_pattern.begin(), scan1_pattern.end());
}
