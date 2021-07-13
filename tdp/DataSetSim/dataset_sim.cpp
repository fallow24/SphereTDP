#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
//#include <omp.h>

#include "dataset_sim.hpp"
#include "constants.hpp"
#include "parsing.hpp"

using namespace std;

string Dataset_Simulator::get_ray_result(double elevation, double azimuth)
{
    vector<double> root{0.0,0.0,0.0},
                   slope{0.0,0.0,0.0},
                   intersection{0.0,0.0,0.0};
    /* Compute robot frame representation of ray:
     * The root is always in the origin of the robot, hence we can leave it as is,
     * For the slope we use the azimuth and elevation to find the ray
     * We define
     *    azimuth := angle from z-axis to point in z-x-axis
     *    elevation := angle from z-axis to point in z-y-axis */
    slope[0] = - cos(elevation)*sin(azimuth);
    slope[1] = sin(elevation);
    slope[2] = cos(azimuth)*cos(elevation);
    // Normalize slope
    double norm_factor = 1/sqrt(slope[0]*slope[0] + slope[1]*slope[1] + slope[2]*slope[2]); 
    slope[0] *= norm_factor;
    slope[1] *= norm_factor;
    slope[2] *= norm_factor; 
    //Get robot pose
    auto pose = this->get_robot()->get_mover()->get_real_pose();
    // transfer to world coordinate frame
    robot2world_frame(pose, false, root);
    // Slope only needs to be rotated, not translated
    robot2world_frame(pose, true, slope);
    // find the intersection vector
    this->get_world()->get_ray_intersection(root,slope, intersection);

    if(VERBOSITY_LEVEL == 2)
    {
      cout << "Intersection World coordinate: " << intersection[0] << " "
                                                << intersection[1] << " "
                                                << intersection[2] << endl;
    }
    if(intersection.empty()){return "";}
    else
    {
      /* Add range noise to intersection by adding a random portion
       * of the slope to it. Probability distribution is defined in
       * the sensor model. Since the slope is a unit vector the
       * magnitude is defined my the magnitude of the sensor noise model
       * inside the sensor class.
       *  noisy_intersection = intersection + random_variable*slope
       */
      double range = eucl_dist(intersection, root);
      double noise_factor = this->get_robot()->get_sensor()->get_range_noise()*range;
      intersection[0] += noise_factor*slope[0];
      intersection[1] += noise_factor*slope[1];
      intersection[2] += noise_factor*slope[2];
      // Check whether the intersection is within the range of the sensor
      if(range < this->get_robot()->get_sensor()->get_range().first ||
         range > this->get_robot()->get_sensor()->get_range().second)
      {
        // If not, the point should not be measured
        intersection.clear();
        return "";
      }

      // backtransfer from world to robot coordinate system
      world2robot_frame(pose, false, intersection);
      // return vector as string
      stringstream xyz;
      xyz << fixed << setprecision(4) << showpoint;
      for (auto i = intersection.begin(); i != intersection.end(); ++i)
      {
         xyz << *i << " ";
      }
      xyz << endl;

      return xyz.str();
    }
}

void Dataset_Simulator::write_all_sensor_points(fstream &scan_file, double dt)
{
  /* A vector representing the scanning pattern as pairs of elevation/azimuth */
  vector<pair<double, double> > pattern;
  this->get_robot()->get_sensor()->get_sensing_pattern(pattern, dt);
  stringstream points;
  
  //#pragma omp parallel for
  for(unsigned k = 0; k < pattern.size(); k++)
  {
    pair<double,double> *coordinates = &pattern[k];
    string results = get_ray_result(deg_to_rad(get<0>(*coordinates)),
                deg_to_rad(get<1>(*coordinates)));

    //#pragma omp critical
    points << results;
  }

  scan_file << points.str();
}

void Dataset_Simulator::write_pose(ostream &pose_file, bool write_true_pose)
{
  auto val = write_true_pose ? 
        this->get_robot()->get_mover()->get_real_pose() : 
        this->get_robot()->get_mover()->get_assumed_pose();

  stringstream ss;
  ss << fixed << setprecision(4) << showpoint;
  ss << val[0] << " " << val[1] << " " << val[2] << " "
     << rad_to_deg(val[3]) << " "
     << rad_to_deg(val[4]) << " "
     << rad_to_deg(val[5]) << endl;

  pose_file << ss.str();
}

int main(int argc, char const *argv[])
{
    string out_file_path;
    double sim_time, dt;
    bool write_true_poses;
    World *world;
    Robot *robot;

    try{
      write_true_poses = parse_options(argc, argv,
        out_file_path, sim_time, dt, robot, world);
    } catch(exception &e)
    {
      cerr << "Error while parsing settings: " << e.what() << std::endl;
      exit(1);
    }

    Dataset_Simulator sim(world, robot, sim_time);
    int file_number = 0;
    for(double curr_time = 0;
        curr_time <= sim.get_sim_time();
        curr_time += dt)
    {
      // Open the appropriate files
      char* file_name = new char[50]();

      sprintf(file_name,
              out_file_path.back()=='/' ? "%sscan%03d.3d" : "%s/scan%03d.3d",
              out_file_path.c_str(), file_number);
      fstream scan_file(file_name, ios_base::out);

      sprintf(file_name,
              out_file_path.back()=='/' ? "%sscan%03d.pose" : "%s/scan%03d.pose",
              out_file_path.c_str(), file_number);
      fstream pose_file(file_name ,ios_base::out);

      // Write data to files
      sim.write_all_sensor_points(scan_file, dt);
      sim.write_pose(pose_file);

      if(write_true_poses)
      {
        sprintf(file_name,
                out_file_path.back()=='/' ? "%strue_pose/scan%03d.pose" : "%s/true_pose/scan%03d.pose",
                out_file_path.c_str(), file_number);
        fstream pose_file(file_name ,ios_base::out);
        sim.write_pose(pose_file, write_true_poses);
      }

      //Close Files
      scan_file.close();
      pose_file.close();
      delete[] file_name;

      // Console Logging
      if(VERBOSITY_LEVEL == 1)
      {
        cout << "T= " << curr_time << " Wrote Pose:  ";
        sim.write_pose(cout);
      }

      // Simulate robot movement
      sim.get_robot()->get_mover()->move(dt);
      file_number++;
    }
    return 0;
}
