#include "rectangle_world.hpp"

void Rectangle_World::get_ray_intersection(vector<double> root,
                                           vector<double> slope,
                                           vector<double> &intersection)
{
    intersection[0] = intersection[1] = intersection[2] = 0;

    /* Compute the intersection with any of the the planes of the rectangle
     * given the ray. If one intersection is found we can stop as no further
     * intersection can exist */

    /* If we're withing rectangle
     * bounds if x axis is zero, then there is an intersection.
     * If the slope is parallel to the plane (i.e. slope in the given axis = 0),
     * there is no need to check for an intersection
     * Check "left" and "right" plane:
     */
    if(slope[0] != 0)
    {
        // "Left" plane
        intersection[0] = -this->get_width()*0.5;
        intersection[1] = root[1] + (-this->get_width()*0.5 - root[0])/slope[0] * slope[1];
        intersection[2] = root[2] + (-this->get_width()*0.5 - root[0])/slope[0] * slope[2];
        if(abs(intersection[1]) <= this->get_height()*0.5 &&
           abs(intersection[2]) <= this->get_depth()*0.5 &&
           // Check that the intersection is in viewing direction of the robot
           0 <= (-this->get_width()*0.5 - root[0])/slope[0])
        {
            return;
        }

        // "Right" plane
        intersection[0] = this->get_width()*0.5;
        intersection[1] = root[1] + (this->get_width()*0.5 - root[0])/slope[0] * slope[1];
        intersection[2] = root[2] + (this->get_width()*0.5 - root[0])/slope[0] * slope[2];
        if(abs(intersection[1]) <= this->get_height()*0.5 &&
           abs(intersection[2]) <= this->get_depth()*0.5 &&
           // Check that the intersection is in viewing direction of the robot
           0 <= (this->get_width()*0.5 - root[0])/slope[0])
        {
            return;
        }
    }

    /* Check top and bottom plane */
    if(slope[1] != 0)
    {
        // "bottom" plane
        intersection[0] = root[0] + (-this->get_height()*0.5 - root[1])/slope[1] * slope[0];
        intersection[1] = -this->get_height()*0.5;
        intersection[2] = root[2] + (-this->get_height()*0.5 - root[1])/slope[1] * slope[2];
        if(abs(intersection[0]) <= this->get_width()*0.5 &&
           abs(intersection[2]) <= this->get_depth()*0.5 &&
           // Check that the intersection is in viewing direction of the robot
           0 <= (-this->get_height()*0.5 - root[1])/slope[1])
        {
            return;
        }

        // "top" plane
        intersection[0] = root[0] + (this->get_height()*0.5 - root[1])/slope[1] * slope[0];
        intersection[1] = this->get_height()*0.5;
        intersection[2] = root[2] + (this->get_height()*0.5 - root[1])/slope[1] * slope[2];
        if(abs(intersection[0]) <= this->get_width()*0.5 &&
           abs(intersection[2]) <= this->get_depth()*0.5 &&
           // Check that the intersection is in viewing direction of the robot
           0 <= (this->get_height()*0.5 - root[1])/slope[1])
        {
            return;
        }
    }

    /* Check front and back plane */
    if(slope[2] != 0)
    {
        // "front" plane
        intersection[0] = root[0] + (-this->get_depth()*0.5 - root[2])/slope[2] * slope[0];
        intersection[1] = root[1] + (-this->get_depth()*0.5 - root[2])/slope[2] * slope[1];
        intersection[2] = -this->get_depth()*0.5;
        if(abs(intersection[0]) <= this->get_width()*0.5 &&
           abs(intersection[1]) <= this->get_height()*0.5 &&
           // Check that the intersection is in viewing direction of the robot
           0 <= (-this->get_depth()*0.5 - root[2])/slope[2])
        {
            return;
        }

        // "back" plane
        intersection[0] = root[0] + (this->get_depth()*0.5 - root[2])/slope[2] * slope[0];
        intersection[1] = root[1] + (this->get_depth()*0.5 - root[2])/slope[2] * slope[1];
        intersection[2] = this->get_depth()*0.5;
        if(abs(intersection[0]) <= this->get_width()*0.5 &&
           abs(intersection[1]) <= this->get_height()*0.5 &&
           // Check that the intersection is in viewing direction of the robot
           0 <= (this->get_depth()*0.5 - root[2])/slope[2])
        {
            return;
        }
    }

    // No interrsection is within range return empty vector
    intersection.clear();
    return;
}

bool Rectangle_World::is_inside_world(vector<double> pose)
{
    return (abs(pose[0]) <= this->get_width()*0.5 &&
            abs(pose[1]) <= this->get_height()*0.5 &&
            abs(pose[2]) <= this->get_depth()*0.5);
}
