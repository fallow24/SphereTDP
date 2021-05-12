#ifndef LINE_SCANNER_H
#define LINE_SCANNER_H 

#include "sensor.hpp"

/* An ideal line scanner with arbitrary FOV.*/
class Line_Scanner: public Sensor
{
public:

    Line_Scanner(pair<double, double> range,
        pair<double, double> fov,
        pair<double, double> resolution):
        Sensor(range,fov,resolution)
    {};
    ~Line_Scanner(){};

    virtual double get_range_noise();
    virtual void get_sensing_pattern(vector<pair<double, double> > &pattern,
                                     double dt);
};

#endif //LINE_SCANNER_H