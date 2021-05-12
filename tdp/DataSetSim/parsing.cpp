#include "parsing.hpp"

namespace po = boost::program_options;

enum world_type {RECTANGLE_WORLD = 0, CYLINDER_WORLD};
enum sensor_type {LINE_SCANNER = 0, NOISY_LINE_SCANNER, LIVOXMID40, LIVOXMID100, LIVOXMID70C};
enum mover_type {ROLLER = 0, DESCENDER, STEPWISE_DESCENDER};

istream& operator>>(istream &in, sensor_type &sensor)
{
    string token;
    in >> token;
    if (token == "LINE_SCANNER")
        sensor = LINE_SCANNER;
    else if (token == "NOISY_LINE_SCANNER")
        sensor = NOISY_LINE_SCANNER;
    else if (token == "LIVOXMID40")
        sensor = LIVOXMID40;
    else if (token == "LIVOXMID100")
        sensor = LIVOXMID100;
    else if (token == "LIVOXMID70C")
        sensor = LIVOXMID70C;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

istream& operator>>(istream &in, mover_type &mover)
{
    string token;
    in >> token;
    if (token == "ROLLER")
        mover = ROLLER;
    else if(token == "DESCENDER")
        mover = DESCENDER;
    else if(token == "STEPWISE_DESCENDER")
        mover = STEPWISE_DESCENDER;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

istream& operator>>(istream &in, world_type &world)
{
    string token;
    in >> token;
    if (token == "RECTANGLE_WORLD")
        world = RECTANGLE_WORLD;
    else if(token == "CYLINDER_WORLD")
        world = CYLINDER_WORLD;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

/* Function used to check that 'opt1' and 'opt2' are not specified
   at the same time. */
void sensor_option_conflict(const po::variables_map & vm,
                     const sensor_type type,
                     const char *option)
{
  if (vm.count("sensor") && vm["sensor"].as<sensor_type>() == type)
    if (vm.count(option))
       throw std::logic_error(std::string("This sensor type is incompatible with ")
                         + option);
}

/* Function used to check that if 'for_what' is specified, then
   'required_option' is specified too. */
void sensor_option_dependency(const po::variables_map & vm,
                              const sensor_type type,
                              const char *option)
{
  if (vm.count("sensor") && vm["sensor"].as<sensor_type>() == type)
    if (!vm.count(option))
       throw std::logic_error(std::string("This sensor type needs ")
                         + option + " to be set");
}
/* Function used to check that 'opt1' and 'opt2' are not specified
   at the same time. */
void world_option_conflict(const po::variables_map & vm,
                     const world_type type,
                     const char *option)
{
  if (vm.count("world") && vm["world"].as<world_type>() == type)
    if (vm.count(option) != 0)
       throw std::logic_error(std::string("This world type is incompatible with ")
                         + option);
}

/* Function used to check that if 'for_what' is specified, then
   'required_option' is specified too. */
void world_option_dependency(const po::variables_map & vm,
                              const world_type type,
                              const char *option)
{
  if (vm.count("world") && vm["world"].as<world_type>() == type)
    if (!vm.count(option))
       throw std::logic_error(std::string("This world type needs ")
                         + option + " to be set");
}

/* Function used to check that 'opt1' and 'opt2' are not specified
   at the same time. */
void mover_option_conflict(const po::variables_map & vm,
                     const mover_type type,
                     const char *option)
{
  if (vm.count("mover") && vm["mover"].as<mover_type>() == type)
    if (vm.count(option))
       throw std::logic_error(std::string("This mover type is incompatible with ")
                         + option);
}

/* Function used to check that if 'for_what' is specified, then
   'required_option' is specified too. */
void mover_option_dependency(const po::variables_map & vm,
                              const mover_type type,
                              const char *option)
{
  if (vm.count("mover") && vm["mover"].as<mover_type>() == type)
    if (!vm.count(option))
       throw std::logic_error(std::string("This mover type needs ")
                         + option + " to be set");
}

bool parse_options(int argc, char const *argv[],
                  string &out_file_path, double &sim_time, double &dt,
                  Robot *&robot, World *&world)
{
    /* Temporary variables to read into */
    sensor_type sensor_ty;
    mover_type mover_ty;
    world_type world_ty;
    Sensor *sensor; Mover *mover;
    double height, width, depth, cylinder_radius,
           radius, rot_rate, desc_rate, desc_step,
           range_mean, range_std;
    pair<double, double> range, fov, resolution;
    bool write_true_pose = false;
    // Left Hand Coordinate System
    vector<double> init_pose = {};
    // Contains pose noise def for x and z axis
    vector<double> pose_mean = {};
    vector<double> pose_std = {};
    /*************************************/

    // TODO add options for robot type and make other options dependend on that
    po::options_description input("Input options");
    input.add_options()
        ("help,h", "produce help message")
        ("outpath,o", po::value<string>(&out_file_path)->default_value("./3dtk_files"), "output path")
        ("true_pose,p", po::bool_switch(&write_true_pose)->default_value(false), "Whether to also write the true pose")
        ("minrange", po::value<double>(&range.first)->default_value(20.0), "Minimal Laserscanner Range [cm]")
        ("maxrange", po::value<double>(&range.second)->default_value(20000.0), "Maximal Laserscanner Range [cm]")
        ("sim_time,t", po::value<double>(&sim_time)->default_value(122), "Simulation Duration [s]")
        ("dt", po::value<double>(&dt)->default_value(0.1), "Simulation time step dt [s]")
        ("elevFOV", po::value<double>(&fov.first)->default_value(120), "Field of View in Elevation[deg]")
        ("aziFOV", po::value<double>(&fov.second)->default_value(120), "Field of View in Azimuth[deg]")
        ("elevRes", po::value<double>(&resolution.first)->default_value(1), "Resolution in Elevation[deg]")
        ("aziRes", po::value<double>(&resolution.second)->default_value(1), "Resolution in Azimuth[deg]")
        ("height", po::value<double>(&height)->default_value(150.0), "Height of rectangle world [cm]")
        ("width", po::value<double>(&width), "Width of rectangle_world [cm]")
        ("depth", po::value<double>(&depth), "Depth of rectangle_world [cm]")
        ("cylRadius", po::value<double>(&cylinder_radius), "Radius of the cylindrical world [cm]")
        ("sensor", po::value<sensor_type>(&sensor_ty)->default_value(LINE_SCANNER), "Choose sensor type: LINE_SCANNER, NOISY_LINE_SCANNER, LIVOXMID40, LIVOXMID100, LIVOXMID70C")
        ("mover", po::value<mover_type>(&mover_ty)->default_value(ROLLER), "Choose mover type: ROLLER")
        ("world", po::value<world_type>(&world_ty)->default_value(RECTANGLE_WORLD), "Choose world type: RECTANGLE_WORLD, CYLINDER_WORLD")
        ("radius", po::value<double>(&radius)->default_value(10.0), "Radius of Spherical Robot [cm]")
        ("rotRate", po::value<double>(&rot_rate)->default_value(M_PI_4), "Rotation rate of the robot [rad/s]")
        ("descRate", po::value<double>(&desc_rate), "Descention rate of the robot [cm/s]")
        ("descStep", po::value<double>(&desc_step), "Descention step of the stepwise descender [cm]")
        ("range_mean", po::value<double>(&range_mean)->default_value(0.0), "Mean of the range measurement noise [part of Range]")
        ("range_std", po::value<double>(&range_std)->default_value(0.0), "Standard Deviation of the range measurement noise [part of Range]")
        ("pose_mean", po::value<vector<double> >(&pose_mean)->multitoken(), "Mean of the pose measurement noise [disturbance torque about <x axis, z axis> for ROLLER or <radial noise, height noise> for DESCENDER]")
        ("pose_std", po::value<vector<double> >(&pose_std)->multitoken(), "Standard Deviation of the pose measurement noise [disturbance torque about <x axis, z axis> for ROLLER or <radial noise, height noise> for DESCENDER]")
        ("init_pose", po::value<vector<double> >(&init_pose)->multitoken(), "Initial pose of the robot x,y,z, roll,pitch,yaw")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, input), vm);

    if (vm.count("help")) {
      cout << input << endl;
      cout << "Example Usage" << endl;
      cout << "bin/dataset_sim --sensor LIVOXMID100 --mover ROLLER --world RECTANGLE_WORLD --width 300 --depth 1000 --height 200 --elevFOV 38.4 --aziFOV 98.4" <<endl;
      exit(0);
    }

    po::notify(vm);

    sensor_option_conflict(vm, LINE_SCANNER, "range_mean");
    sensor_option_conflict(vm, LINE_SCANNER, "range_std");

    world_option_dependency(vm, RECTANGLE_WORLD, "width");
    world_option_dependency(vm, RECTANGLE_WORLD, "height");
    world_option_dependency(vm, RECTANGLE_WORLD, "depth");
    world_option_conflict(vm, RECTANGLE_WORLD, "cylRadius");

    world_option_dependency(vm, CYLINDER_WORLD, "cylRadius");
    world_option_dependency(vm, CYLINDER_WORLD, "height");
    world_option_conflict(vm, CYLINDER_WORLD, "depth");
    world_option_conflict(vm, CYLINDER_WORLD, "width");

    mover_option_dependency(vm, ROLLER, "radius");
    mover_option_conflict(vm, ROLLER, "descRate");

    mover_option_dependency(vm, DESCENDER, "descRate");
    mover_option_dependency(vm, STEPWISE_DESCENDER, "descStep");


    // If no values or not enough values 
    // we assume no noise and pushback the corresponding 
    // entry
    while(pose_mean.size() < 2){
      pose_mean.push_back(0.0);

    }

    while(pose_std.size() < 2){
      pose_mean.push_back(0.0);

    }

    if(pose_mean.size() != 2)
    {
      cerr << "Not correct number of values given for pose mean" << endl;
      exit(1); 
    }
    if(pose_std.size() != 2)
    {
      cerr << "Not correct number of values given for pose std" << endl;
      exit(1); 
    }
    // init pose default value
    if(init_pose.empty())
    {
      init_pose.push_back(0.0);
      init_pose.push_back(-height*0.5 + radius);
      init_pose.push_back(-depth*0.5 + radius);


      init_pose.push_back(0);
      init_pose.push_back(0);
      init_pose.push_back(0);
    }
    else
    {
      if(init_pose.size() != 6)
      {
        cerr << "Not correct number of values given for initial pose" << endl;
        exit(1);
      }
    }

    /* Create world and sensor based on given type */
    switch(sensor_ty)
    {
      case LINE_SCANNER:
        sensor = new Line_Scanner(range, fov, resolution);
      break;
      case NOISY_LINE_SCANNER:
        sensor = new Noisy_Line_Scanner(range, fov, resolution, range_mean, range_std);
      break;
      case LIVOXMID40:
        sensor = new LivoxMid40(range, fov, resolution, range_mean, range_std);
      break;
      case LIVOXMID100:
        sensor = new LivoxMid100(range, fov, resolution, range_mean, range_std);
      break;
      case LIVOXMID70C:
        sensor = new LivoxMid70C(range, fov, resolution, range_mean, range_std);
      break;
      default:
        cerr << "Specified non existing sensor type" << endl;
        assert(false);
      break;
    }

    switch(mover_ty)
    {
      case ROLLER:
        mover = new Roller(init_pose, radius,
          pose_mean, pose_std);
      break;
      case DESCENDER:
        mover = new Descender(init_pose,
          rot_rate, desc_rate,
          pose_mean, pose_std);
      break;
      case STEPWISE_DESCENDER:
        mover = new Stepwise_Descender(init_pose,
          rot_rate, desc_rate, desc_step,
          pose_mean, pose_std);
      break;
      default:
        cerr << "Specified non-existing mover type" <<endl;
        assert(false);
      break;
    }

    switch(world_ty)
    {
      case RECTANGLE_WORLD:
        world = new Rectangle_World(height, width, depth);
      break;
      case CYLINDER_WORLD:
        world = new Cylinder_World(cylinder_radius, height);
      break;
      default:
        cerr << "Specified non-existing world type" << endl;
        assert(false);
      break;
    }

    robot = new Robot(mover, sensor);

    return write_true_pose;
}
