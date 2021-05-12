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
    // check if the given path exists
    if ( !existsDir( dir.c_str() ) ) 
    {
        std::cout << "Using path: " << dir << std::endl;
        std::cout << "Invalid plane path. Doesnt exist." << std::endl;
        exit(0); 
    }
    dir += "normals.list";

    // Read the normals.list file to get all planeXXX.n files 
    std::vector<std::string> planefiles;
    std::ifstream input;
    std::string line; 
    input.open( dir.c_str() );
    while ( std::getline( input, line ) )
    {
        line = line.substr( 7, line.size() ); // remove the "Normal " entry
        planefiles.push_back( line );
    }
    input.close();
    input.clear();
 
    // open planeXXX.n files and read points
    std::vector<Point> points;
    for ( const auto& path : planefiles )
    {
        double n[3];
        double x[3];
        std::cout << "Reading " << path << std::endl;
        std::cout.flush();
        // extract points
        input.open( path.c_str() );
        while ( input.good() )
        {
            try
            {
                input >> n[0] >> n[1] >> n[2];
                input >> x[0] >> x[1] >> x[2];
            }
            catch (...) { break; }
        }
        input.close();
        input.clear();

        // Construct ConvexPlane objects
        allPlanes.push_back( new NormalPlane(n, x) );

    }

}