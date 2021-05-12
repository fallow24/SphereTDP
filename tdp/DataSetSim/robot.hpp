#ifndef ROBOT_H
#define ROBOT_H

#include "movers/mover.hpp"
#include "movers/roller.hpp"
#include "movers/descender.hpp"
#include "movers/stepwise_descender.hpp"

#include "sensors/sensor.hpp"
#include "sensors/line_scanner.hpp"
#include "sensors/noisy_line_scanner.hpp"
#include "sensors/livoxMid40.hpp"
#include "sensors/livoxMid100.hpp"
#include "sensors/livoxMid70C.hpp"

class Robot
{
public:
    Robot(Mover *mover, Sensor *sensor)
    {
        this->mover = mover;
        this->sensor = sensor;
    }
    ~Robot()
    {
        delete this->sensor;
        delete this->mover;
    }

    Sensor *get_sensor() {return this->sensor;}
    Mover *get_mover() {return this->mover;}

    void set_sensor(Sensor *sensor){this->sensor = sensor;}
    void set_mover(Mover *mover){this->mover = mover;}

private:
    Sensor *sensor;
    Mover *mover;
};


#endif //ROBOT_H
