/** @file
 *  @brief Representation of a plane in normal form.
 *  
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#include "normalplane.h"

NormalPlane::NormalPlane(double *n, double *x)
{
    for(int i = 0; i < 3; i++) 
    {
        this->n[i] = n[i];
        this->x[i] = x[i];
    }
    rho = Dot(n, x);
}

double& NormalPlane::operator()(int idx)
{
    return n[idx];
}

double& NormalPlane::operator[](int idx)
{
    return x[idx];
}