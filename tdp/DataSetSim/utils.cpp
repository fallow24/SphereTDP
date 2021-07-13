#include "utils.hpp"

/* Coordinate transformation functions. Uses the current pose
 * of the robot to compute the coordinates of the given point
 * in the respective other coordinate frame */
void robot2world_frame(vector<double> pose, bool rotate_only, vector<double> &robot_coordinates)
{
  // Store the current coordinate values
  double x = robot_coordinates[0], y = robot_coordinates[1],
         z = robot_coordinates[2];
  // Find translation required by the transformation
  double tx, ty, tz;
  if(rotate_only)
  {
    tx = ty = tz = 0.0;
  }
  else
  {
    tx = pose[0]; ty = pose[1]; tz = pose[2];
  }

  double roll = pose[3], pitch = pose[4], yaw = pose[5];
  /* Compute rotation matrix from euler angles. Where
   *     |r11 r12 r13|
   * R = |r21 r22 r23|
   *     |r31 r32 r33|
   */
  // Using roll-pitch-yaw
  double r11 = cos(pitch)*cos(yaw),
         r12 = -cos(pitch)*sin(yaw),
         r13 = sin(pitch),
         r21 = cos(roll)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch),
         r22 = cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw),
         r23 = -cos(pitch)*sin(roll),
         r31 = sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch),
         r32 = cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw),
         r33 = cos(roll)*cos(pitch);

  /* Execute homogeneous transformation:
   *        x_new = | R T |  |x|
   *         |1|    | 0 1 |  |1|
   */
  robot_coordinates[0] = r11*x + r12*y + r13*z + tx;
  robot_coordinates[1] = r21*x + r22*y + r23*z + ty;
  robot_coordinates[2] = r31*x + r32*y + r33*z + tz;
}

void world2robot_frame(vector<double> pose, bool rotate_only, vector<double> &world_coordinates)
{
 // Store the current coordinate values
  double x = world_coordinates[0],
         y = world_coordinates[1],
         z = world_coordinates[2];
  double roll = pose[3], pitch = pose[4], yaw = pose[5];
  /* Compute rotation matrix from euler angles. Where
   *     |r11 r12 r13|
   * R = |r21 r22 r23| This is the transpose of the inverse transformation
   *     |r31 r32 r33|
   */
  // Using roll-pitch-yaw
  double r11 = cos(pitch)*cos(yaw),
         r12 = cos(roll)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch),
         r13 = sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch),
         r21 = -cos(pitch)*sin(yaw),
         r22 = cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw),
         r23 = cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw),
         r31 = sin(pitch),
         r32 = -cos(pitch)*sin(roll),
         r33 = cos(roll)*cos(pitch);

  /* Find translation via t' = -R*t */
  double tx, ty, tz;
  if(rotate_only)
  {
    tx = ty = tz = 0.0;
  }
  else
  {
    tx = - (r11*pose[0] + r12*pose[1] + r13*pose[2]);
    ty = - (r21*pose[0] + r22*pose[1] + r23*pose[2]);
    tz = - (r31*pose[0] + r32*pose[1] + r33*pose[2]);
  }
  /* Execute homogeneous transformation:
   *        x_new = | R T | x
   *                | 0 1 |
   */
  world_coordinates[0] = r11*x + r12*y + r13*z + tx;
  world_coordinates[1] = r21*x + r22*y + r23*z + ty;
  world_coordinates[2] = r31*x + r32*y + r33*z + tz;
}

/* Helper function for computing the distance between two
 * points in space using the euclidean distance */
double eucl_dist(vector<double> first, vector<double> second)
{
    return sqrt((first[0] - second[0])*(first[0] - second[0])
            +   (first[1] - second[1])*(first[1] - second[1])
            +   (first[2] - second[2])*(first[2] - second[2]));
}

/* Helper to normalize a vector */
void normalize(vector<double> & vec)
{
  double mag = 0;
  for(double val: vec){ mag += val*val;}
  double norm_fac = 1/sqrt(mag);
  for(uint i = 0; i < vec.size(); i++)
  {
    vec[i] *= norm_fac;
  }
}

/* Helper for Radians and degree conversion */
double deg_to_rad(double deg) {/* pi/180*deg */ return M_PI*0.00555555555*deg;}
double rad_to_deg(double rad) {/* 180/Pi*rad */ return M_1_PI*180.0*rad;}

/* Helper to map a radian value to [-pi,pi] */
double to_pi_range(double rad)
{
  if(rad > M_PI) return rad - 2*M_PI;
  else if(rad < - M_PI) return rad + 2*M_PI;
  else return rad;
}

/* Coordinate transformations */
void cartesian_to_cylindrical(vector<double> &cart_coord)
{
  /* In our case we definde the left handed cylindrical coordinate
   * such that
   *  rho := distance in the z-x-plane
   *  phi := angle around the y-axis (Left hand rotation)
   *         0 inside y-z-axis (defined on [-pi,pi] )
   *  z := height along the y-axis
   */

  double x = cart_coord[0], y = cart_coord[1], z = cart_coord[2];
  cart_coord[0] = sqrt(x*x+z*z);
  if(x == 0 && y == 0) cart_coord[1] = 0;
  else cart_coord[1] = atan2(x,z);
  cart_coord[2] = y;
}


void cylindrical_to_cartesian(vector<double> &cyli_coord)
{
  double rho = cyli_coord[0], phi = cyli_coord[1], z = cyli_coord[2];

  cyli_coord[0] = rho*sin(phi);
  cyli_coord[1] = z;
  cyli_coord[2] = rho*cos(phi);
}
