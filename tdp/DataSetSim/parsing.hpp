#ifndef PARSING_H
#define PARSING_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <boost/program_options.hpp>

#include "worlds/world.hpp"
#include "worlds/rectangle_world.hpp"
#include "worlds/cylinder_world.hpp"
#include "robot.hpp"

bool parse_options(int argc, char const *argv[],
                  string &out_file_path, double &sim_time, double &dt,
                  Robot *&robot, World *&world);


#endif // PARSING_H
