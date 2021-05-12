#ifndef RECTANGLE_WORLD_H
#define RECTANGLE_WORLD_H 

#include "world.hpp"

class Rectangle_World: public World
{
public:
    Rectangle_World(double height, double width, double depth):
        height(height), width(width), depth(depth){};
    ~Rectangle_World(){};

    virtual void get_ray_intersection(vector<double> root,
                                      vector<double> slope,
                                      vector<double> &intersection);

    virtual bool is_inside_world(vector<double> pose);

    // Accessors
    double get_height(){return this->height;}
    double get_width(){return this->width;}
    double get_depth(){return this->depth;}

private:
    double height, width, depth;
};

#endif //RECTANGLE_WORLD_H