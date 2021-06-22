#ifndef __CLUSTER_H
#define __CLUSTER_H

#include <string>
#include <iostream>
#include <omp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "slam6d/scan.h"
#include "slam6d/io_types.h"
#include "scanio/framesreader.h"
#include "planescan.h"

using namespace std;
namespace po = boost::program_options;

ostream& bold_on(ostream& os)
{
    return os << "\e[1m";
}

ostream& bold_off(ostream& os)
{
    return os << "\e[0m";
}

// Boost needs to convert from string to IOType. Use this validation function.
void validate(boost::any& v, const std::vector<std::string>& values,
              IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

int existsDir(const char* path)
{
    struct stat info;
    if (stat( path, &info ) != 0) return 0;
    else if ( info.st_mode & S_IFDIR ) return 1;
    else return 0;
}

int parse_options(  int argc,
                    char** argv,
                    bool& scanserver,
                    string& scandir,
                    IOType &type,
                    bool &quiet, 
                    int &maxDist, 
                    int &minDist, 
                    int &octree, 
                    double &red,
                    int& start,
                    int& end,
                    int& k_nearest,
                    int& n_threads,
                    double& d_growth,
                    int& n_min_clusterpoints, 
                    double& eps_alpha_similarity)
{
    po::options_description generic("Generic options");
    po::options_description input("Program options");
    po::options_description hidden("Hidden options");
    generic.add_options()
        ("help,h", "Display a very helpful message");
    input.add_options()
        ("scanserver", po::bool_switch(&scanserver)->default_value(false),
            "Use scanserver as input method.")
        ("format,f", po::value<IOType>(&type)->default_value(UOS, "uos"),
            "chose input format from {uos, uosr, uos_rgb[=clustering], uos_normal, old, xyz}.")
        ("start,s", po::value<int>(&start)->default_value(0),
            "Skip the first <arg> scans.")
        ("end,e", po::value<int>(&end)->default_value(-1),
            "Stop at scan index <arg>")
        ("max,m", po::value<int>(&maxDist)->default_value(-1),
            "neglegt all data points with a distance larger than <arg> 'units'")
        ("min,M", po::value<int>(&minDist)->default_value(-1),
            "neglegt all data points with a distance smaller than <arg> 'units'")
        ("reduce,r", po::value<double>(&red)->default_value(-1.0),
            "turns on octree based point reduction (voxel size= <arg>)")
        ("octree,O", po::value<int>(&octree)->default_value(0),
            "Use randomized octree based point reduction (pts per voxel=<arg>)")
        ("k_nearest,K", po::value(&k_nearest)->default_value(20),
            "Use <arg> nearest neighbours in local normal calculation. Is ignored if uos_normal format is used.")
        ("n_threads,t", po::value<int>(&n_threads)->default_value(1),
            "Use <arg> threads. Each thread clusters a subscan.")
        ("d_growth,g", po::value(&d_growth)->default_value(5.0),
            "Threshold for cluster region growing. Stop region growing if there is no new point nearer than <arg>.")
        ("min_cluster_size,z", po::value(&n_min_clusterpoints)->default_value(100),
            "Do not consider a cluster unless it has more then <arg> points in it.")
        ("eps_similarity,y", po::value(&eps_alpha_similarity)->default_value(5),
            "Two normals are considered similar if their angle is smaller then <arg> in degrees.");
    hidden.add_options()
        ("input-dir", po::value<string>(&scandir), "input dir");    
    // All options together
    po::options_description alloptions;
    alloptions.add(generic).add(input).add(hidden);

    // Only commandline, visible with --help
    po::options_description cmdoptions;
    cmdoptions.add(generic).add(input);

    // positional argument for input directory 
    po::positional_options_description pos;
    pos.add("input-dir", 1); // max 1 pos arg

    // Map and store option inputs to variables
    po::variables_map vars;
    po::store( po::command_line_parser(argc, argv).
                options(alloptions).
                positional(pos).
                run(), 
                vars);

    // help display msg
    if ( vars.count("help") )
    {
        cout << cmdoptions;
        cout << endl << "Example usage:" << endl 
            << bold_on << "\t bin/cluster -f uosr -s 0 -e 12 -K 20 -t 12 -g 5" << bold_off 
            << endl;
        exit(0);
    }
    po::notify(vars);

    // Add trailing directory slash if there is none. 
    // Works differently when compiling under Windows
#ifndef _MSC_VER
    if (scandir[ scandir.length()-1 ] != '/') scandir = scandir + "/";
#else
    if (scandir[ scandir.length()-1]  != '\\') scandir = scandir + "\\";
#endif

    // return with success exit code
    return 0;
}

#endif // __CLUSTER_H