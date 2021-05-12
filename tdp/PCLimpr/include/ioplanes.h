/** @file
 *  @brief Representation of a plane either in normal or convex hull form.
 *  Wrapper class that extends the ConvexPlane class and adds 
 *  IO funtionality. 
 * 
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#ifndef __IO_CONV_PLANE_H_
#define __IO_CONV_PLANE_H_

#include "normalplane.h"
#include "slam6d/normals.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>
#include <sstream> 

typedef std::vector<NormalPlane*> Planes;

class PlaneIO {

public:
    static void read(std::string);
    static Planes allPlanes;

private: 
    static int existsDir(const char* path)
    {
        struct stat info;
        if (stat( path, &info ) != 0) return 0;
        else if ( info.st_mode & S_IFDIR ) return 1;
        else return 0;
    }
    // wrapper around Dot product function
    static double dot(Point &p, double *p2)
    {
        double *p1 = new double[3];
        p1[0] = p.x;
        p1[1] = p.y;
        p1[2] = p.z;
        return Dot(p1, p2);
    }
};


#endif //__NORM_CONV_PLANE_H_