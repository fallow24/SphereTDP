/** @file 
 *  @brief Class has only static members that handle Plane IO.
 *  Reads "normals.list" and "planeXXX.n" files that have been previously
 *  exported with bin/planes, utilizing the hough transform to detect
 *  planes.
 *  
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#include "ioplanes.h"

Planes PlaneIO::allPlanes;

void PlaneIO::read(std::string dir)
{
    string dir2 = dir;
    // check if the given path exists
    if ( !existsDir( dir.c_str() ) ) 
    {
        std::cout << "Using path: " << dir << std::endl;
        std::cout << "Invalid plane path. Doesnt exist." << std::endl;
        exit(0); 
    }
    dir += "normals.list";
    dir2 += "planes.list";

    // Read the normals.list file to get all planeXXX.n files 
    std::vector<std::string> planefiles;
    std::vector<std::string> hullfiles;
    std::ifstream input, hinput;
    std::string line, hline; 
    input.open( dir.c_str() );
    hinput.open( dir2.c_str() );
    while ( std::getline( input, line ) && std::getline(hinput, hline))
    {
        line = line.substr( 7, line.size() ); // remove the "Normal " entry
        hline = hline.substr( 6, hline.size() ); // Remove the "Plane " entry
        planefiles.push_back( line );
        hullfiles.push_back( hline );
    }
    input.close();
    input.clear();
    hinput.close();
    hinput.clear();

    // open planeXXX.n files and normals and centerpoint
    for ( int i = 0; i < planefiles.size(); ++i )
    {
        const auto& path = planefiles[i];
        const auto& hpath = hullfiles[i];
        std::vector<double*> points = *new vector<double*>(0);
        double p[3];
        double n[3];
        double x[3];
        std::cout << "Reading " << path << " for normal information." << std::endl;
        std::cout << "Reading " << hpath << " for convex hull information." << std::endl;
        // extract points
        input.open( path.c_str() );
        hinput.open ( hpath.c_str() );
        while ( input.good() )
        {
            try
            {
                hinput >> p[0] >> p[1] >> p[2];
                points.push_back(p);
                input >> n[0] >> n[1] >> n[2];
                input >> x[0] >> x[1] >> x[2];
            }
            catch (...) { break; }
        }
        input.close();
        input.clear();
        hinput.close();
        hinput.clear();

        // Construct ConvexPlane objects
        allPlanes.push_back( new NormalPlane(n, x, points) );
    }

}