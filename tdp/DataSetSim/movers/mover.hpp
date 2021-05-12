#ifndef MOVER_H
#define MOVER_H

#include <vector>
#include <random>

#include "../utils.hpp"

using namespace std;

class Mover
{
public:
    Mover(vector<double> init_pose):
       real_pose(init_pose), assumed_pose(init_pose){};

    virtual ~Mover(){};

    /* Defines the movement of the associated robot.
     * Changes the stored pose by the movement that happens over
     * @dt seconds */
    virtual void move(double dt) = 0;

    /* Returns the true current pose of the sensor */
    virtual vector<double> get_real_pose(){return this->real_pose;}

    /* Sets the true current pose of the sensor the given pose*/
    virtual void set_real_pose(vector<double> new_pose){this->real_pose = new_pose;}

    /* Returns the assumed current pose of the sensor */
    virtual vector<double> get_assumed_pose(){return this->assumed_pose;}

    /* Sets the true current pose of the sensor the given pose*/
    virtual void set_assumed_pose(vector<double> new_pose){this->assumed_pose = new_pose;}
private:
    vector<double> real_pose;
    vector<double> assumed_pose;
};

#endif // MOVER_H
