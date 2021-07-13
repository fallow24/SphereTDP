#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <cmath>
#include <iostream>

#include "constants.hpp"

using namespace std;
/* Coordinate transformation functions. Uses the current pose
 * of the robot to compute the coordinates of the given point
 * in the respective other coordinate frame */
void robot2world_frame(vector<double> pose, bool rotate_only, vector<double> &robot_coordinates);
void world2robot_frame(vector<double> pose, bool rotate_only, vector<double> &world_coordinates);
/* Helper function for computing the distance between two
 * points in space using the euclidean distance */
double eucl_dist(vector<double> first, vector<double> second);
void normalize(vector<double> & vec);

/* Helper for Radians and degree conversion */
double deg_to_rad(double deg);
double rad_to_deg(double rad);
double to_pi_range(double rad);

/* Coordinate Transformations */
void cartesian_to_cylindrical(vector<double> &cart_coord);
void cylindrical_to_cartesian(vector<double> &cyli_coord);

#endif // UTILS_H
