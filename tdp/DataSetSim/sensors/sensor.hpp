#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <random>
#include <iostream>
#include "../utils.hpp"
using namespace std;

class Sensor
{
public:

    Sensor(pair<double,double> range,
        pair<double, double> fov,
        pair<double, double> resolution):
        range(range), fov(fov), resolution(resolution){};

    virtual ~Sensor(){};

    /* Defines the noise model of the range measurement
     * @returns a value in the range [-n, n] that represent
     *  the deviation from the true measured value
     */
    virtual double get_range_noise() = 0;

    /* Returns a vector that contains the scanning pattern coordinates
     * as azimuth and elevation. Given the dt maybe only a partial scanning pattern
     * can be completed. */
    virtual void get_sensing_pattern(vector<pair<double, double> > &pattern,
                                     double dt) = 0;

    /* Returns the maximal range of the sensor */
    virtual pair<double,double> get_range()
    {
        return this->range;
    }
    /* Returns the fov of the sensor in degree
     * fov[0]: angle in sensor y-direction
     * fov[1]: angle in sensor z-directin
     */
    virtual pair<double, double> get_fov()
    {
        return this->fov;
    }

    /* Returns the angular resolution between rays.
     * can be different for horizontal and vertical
     * rays
     * resolution[0]: resolution in sensor y-direction
     * resolution[1]: resolution in sensor x-direction
     */
    virtual pair<double, double> get_resolution()
    {
        return this->resolution;
    }

private:
    pair<double, double> range;
    pair<double, double> fov;
    pair<double, double> resolution;
};

#endif //SENSOR_H
