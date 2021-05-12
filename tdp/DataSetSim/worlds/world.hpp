#ifndef WORLD_H
#define WORLD_H

#include <iostream>
#include <vector>

#include "../utils.hpp"

using namespace std;

class World
{
public:
    World(){};
    virtual ~World(){};

    /**
     * Returns the coordinates of intersection of the given ray,
     * defined by the root vector and the slope vector, with the world
     */
    virtual void get_ray_intersection(vector<double> root,
                                      vector<double> slope,
                                      vector<double> &intersection) = 0;

    virtual bool is_inside_world(vector<double> pose) = 0;
};

#endif //WORLD_H
