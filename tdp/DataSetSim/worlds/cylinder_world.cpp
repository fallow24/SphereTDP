#include <cassert>
#include "cylinder_world.hpp"

using namespace std;

void Cylinder_World::get_ray_intersection(vector<double> root,
                                          vector<double> slope,
                                          vector<double> &intersection)
{
    /* Now we check with intersections with the top, bottom end the
     * cylindrical shell. If one intersection is found we can stop
     * since no further intersection can exist.
     *
     * Basically we parametrize a ray via root+lambda*slope = radius
     * and find the parameter lambda that fullfilles the equation.
     * Using this we can find the other coordinates and then check
     * whether the resulting intersection is within our world.
     */

    /* Find intersection with cylindrical shell */
    if(slope[0] != 0 || slope[2] != 0)
    {
        /* This is the magic factor that we have to multiply the slope by to
         * find the intersection with the cylindrical shell.
         * Basically we find the parameter lambda such that sqrt(x^2+z^2) = r
         * How to find this? Do not ask me, ask your lord and savior
         * wolfram alpha!
         */
        double radius = this->get_radius();
        double lambda = 1/(slope[0]*slope[0] + slope[2]*slope[2]) *
            (sqrt(slope[2]*slope[2]*(radius*radius - root[0]*root[0]) +
                  2*root[0]*slope[0]*root[2]*slope[2] +
                  slope[0]*slope[0]*(radius*radius - root[2]*root[2]))
            - root[0]*slope[0] - root[2]*slope[2]);

        intersection[0] = root[0] + lambda*slope[0];
        intersection[1] = root[1] + lambda*slope[1];
        intersection[2] = root[2] + lambda*slope[2];

        // Check that we're actually at a cylindrical wall
        assert(abs(sqrt(intersection[0]*intersection[0]
            + intersection[2]*intersection[2]) - radius) < 0.1);

        // If we're in bounds of the cylinder ...
        if(intersection[1] <= this->get_height() &&
           intersection[1] >= 0 &&
           // ...and we're looking in sensor direction
           lambda >= 0)
        {
            // we found an intersection, we can return
            return;
        }
    }

    /* Check top and bottom plane */
    if(slope[1] != 0)
    {
        // "bottom" plane
        intersection[0] = root[0] - root[1]/slope[1] * slope[0];
        intersection[1] = 0;
        intersection[2] = root[2] - root[1]/slope[1] * slope[2];
        if(sqrt(intersection[0]*intersection[0]
            + intersection[2]*intersection[2]) <= this->get_radius() &&
           // Check that the intersection is in viewing direction of the robot
           0 <= - root[1]/slope[1])
        {
            return;
        }

        // "top" plane
        intersection[0] = root[0] + (this->get_height() - root[1])/slope[1] * slope[0];
        intersection[1] = this->get_height();
        intersection[2] = root[2] + (this->get_height() - root[1])/slope[1] * slope[2];
        if(sqrt(intersection[0]*intersection[0]
            + intersection[2]*intersection[2]) <= this->get_radius() &&
           // Check that the intersection is in viewing direction of the robot
           0 <= (this->get_height() - root[1])/slope[1])
        {
            return;
        }
    }

    // No interrsection is within range return empty vector
    intersection.clear();
    return;
}

bool Cylinder_World::is_inside_world(vector<double> pose)
{
    return (sqrt(pose[0]*pose[0] + pose[2]*pose[2]) <= this->get_radius() &&
            pose[1] <= this->get_height() &&
            pose[1] >= 0);
}
