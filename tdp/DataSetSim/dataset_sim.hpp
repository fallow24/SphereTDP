#ifndef DATASET_SIMULATOR_H
#define DATASET_SIMULATOR_H

#include <cassert>
#include <string>

#include "worlds/world.hpp"
#include "utils.hpp"
#include "robot.hpp"

class Dataset_Simulator
{
public:
    Dataset_Simulator(World *world, Robot *robot, double sim_time):
        world(world), robot(robot), sim_time(sim_time)
    {
        // check that sensor is inside world
        assert(world->is_inside_world(robot->get_mover()->get_real_pose()));
    };
    ~Dataset_Simulator()
    {
        delete world;
        delete robot;
    }

    /** Returns the string to be written into a scan file
     * from the current pose using the azimuth and elevation
     * of the ray from the given orientation
     * In particular this function transforms the azitmuth and
     * elevation into a ray described in the world coordinate
     * system and forwards this to the ray intersection method
     * of the world
     */
    string get_ray_result(double azimuth, double elevation);


    /* Writes all measured points at the current pose of the robot
     * into the given file stream */
    void write_all_sensor_points(fstream &scan_file, double dt);

    /* Writes the current pose of the robot
     * into the given file stream */
    void write_pose(ostream &pose_file, bool write_true_pose = false);

    // Accessors
    double get_sim_time(){return this->sim_time;}
    World *get_world(){return this->world;}
    Robot *get_robot(){return this->robot;}

private:
    World *world;
    Robot *robot;
    double sim_time;
};

#endif //DATASET_SIMULATOR_H
