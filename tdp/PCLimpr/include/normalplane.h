/** @file
 *  @brief Representation of a plane in normal form.
 *  
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#ifndef __NORMALPLANE_H_
#define __NORMALPLANE_H_

#include "slam6d/globals.icc"

class NormalPlane
{
public:
    NormalPlane();
    NormalPlane(double*, double*);
    ~NormalPlane();

    double n[3]; // normal
    double x[3]; // reference point
    double rho; // distance from ref to origin

    double& operator()(int); // get normal xyz refs
    double& operator[](int); // get reference xyz refs
};

#endif // __NORMALPLANE_H_