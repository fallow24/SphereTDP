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
    if(fabs(n[0]) < fabs(n[1])) {
        if(fabs(n[1]) < fabs(n[2])) {
        direction = 'z';
        } else {
        direction = 'y';
        }
    } else if (fabs(n[2]) < fabs(n[0])){
        direction = 'x';
    } else {
        direction = 'z';
    }
}

NormalPlane::NormalPlane(double *n, double *x, vector<double*> &ps)
    : NormalPlane(n, x) 
{
    addConvexHull(ps);
    // Convert to Point array.
    _hull_parr = new Point[ps.size()];
    for ( int i = 0; i < ps.size(); ++i) 
    {
        _hull_parr[i].x = ps[i][0];
        _hull_parr[i].y = ps[i][1];
        _hull_parr[i].z = ps[i][2];
    }
}

double& NormalPlane::operator()(int idx)
{
    return n[idx];
}

double& NormalPlane::operator[](int idx)
{
    return x[idx];
}

void NormalPlane::addConvexHull(std::vector<double*> &ps)
{
    hull = ps;
}

Point* NormalPlane::hullAsPointArr()
{
    return _hull_parr;
}

int NormalPlane::isLeft(Point P0, Point P1, Point P2 )
{
return ( (P1.x - P0.x) * (P2.y - P0.y)
                - (P2.x -  P0.x) * (P1.y - P0.y) );
}

int NormalPlane::cn_PnPoly(Point P, Point* V, int n)
{
    int cn = 0;    // the  crossing number counter

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {    // edge from V[i]  to V[i+1]
    if (((V[i].y <= P.y) && (V[i+1].y > P.y))     // an upward crossing
        || ((V[i].y > P.y) && (V[i+1].y <=  P.y))) { // a downward crossing
            // compute  the actual edge-ray intersect x-coordinate
            float vt = (float)(P.y  - V[i].y) / (V[i+1].y - V[i].y);
            if (P.x <  V[i].x + vt * (V[i+1].x - V[i].x)) // P.x < intersect
                ++cn;   // a valid crossing of y=P.y right of P.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if  odd (in)
}

int NormalPlane::wn_PnPoly( Point P, Point* V, int n )
{
    int    wn = 0;    // the  winding number counter

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to  V[i+1]
        if (V[i].y <= P.y) {          // start y <= P.y
            if (V[i+1].y  > P.y)      // an upward crossing
                if (isLeft( V[i], V[i+1], P) > 0)  // P left of  edge
                    ++wn;            // have  a valid up intersect
        }
        else {                        // start y > P.y (no test needed)
            if (V[i+1].y  <= P.y)     // a downward crossing
                if (isLeft( V[i], V[i+1], P) < 0)  // P right of  edge
                    --wn;            // have  a valid down intersect
        }
    }
    return wn;
}

void NormalPlane::convert3Dto2D(Point p, char dir, Point& out)
{
    switch(dir) {
        case 'x': out.x = p.y;
                out.y = p.z;
                break;
        case 'y': out.x = p.x;
                out.y = p.z;
                break;
        case 'z': out.x = p.x;
                out.y = p.y;
                break;
        default: throw std::runtime_error("default branch taken");
    }
}

void NormalPlane::convert3Dto2D(Point* ps, int n, char dir, Point* out)
{
    for (int i = 0; i < n; ++i)
        convert3Dto2D(ps[i], dir, out[i]);
}

double NormalPlane::nearestLineSegment(const Point& p, const Point* polygon, int n, Point& p1, Point& p2)
{
    double D_min = __DBL_MAX__;
    int j_min = 0, i_min = 1;
         int j = j_min;
    for (int i = i_min; i < n; )
    {
        Point pch1 = polygon[j];
        Point pch2 = polygon[i];
        Point pch2_1 = pch2 - pch1;
        Point p_pch1 = p - pch1;
        Point pch2_p = pch2 - p;
        double r = dot( pch2_1, p_pch1 ) / dot( pch2_1, pch2_1 );
        double D;
        if (0 < r && r < 1) {
            D = sqrt( dot( p_pch1, p_pch1 ) - dot( pch2_1, p_pch1 ) );
        } else if (r <= 0) {
            D = sqrt( dot( p_pch1, p_pch1 ) ); 
        } else {
            D = sqrt( dot( pch2_p, pch2_p ) ); 
        }
        if (D < D_min) {
            D_min = D;
            j_min = j;
            i_min = i;
        } 
        j++;
        i++;
    }
    p1 = polygon[j_min];
    p2 = polygon[i_min];
    return D_min;
}

NormalPlane::~NormalPlane()
{

}