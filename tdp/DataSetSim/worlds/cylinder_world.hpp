#ifndef CYLINDER_WORLD_H
#define CYLINDER_WORLD_H 

#include "world.hpp"

class Cylinder_World: public World
{
public:
    Cylinder_World(double radius, double height):
        radius(radius), height(height) {};
    ~Cylinder_World(){};

    virtual void get_ray_intersection(vector<double> root,
                                      vector<double> slope,
                                      vector<double> &intersection);

    virtual bool is_inside_world(vector<double> pose);

    // Accessors
    double get_radius(){return this->radius;}
    double get_height(){return this->height;}

private:
    double radius;
    double height;
};

#endif //CYLINDER_WORLD_H