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
#include "slam6d/scan.h"
#include <vector>

using namespace std;

// Small little helper:
static inline double dot(Point p1, Point p2)
{
    double *p1d = new double[3];
    p1d[0] = p1.x;
    p1d[1] = p1.y;
    p1d[2] = p1.z;
    double *p2d = new double[3];
    p2d[0] = p2.x;
    p2d[1] = p2.y;
    p2d[2] = p2.z;
    return Dot(p1d, p2d);
}

class NormalPlane
{
public:
    NormalPlane();
    NormalPlane(double*, double*);
    NormalPlane(double*, double*, vector<double*>&);
    ~NormalPlane();

    double n[3]; // normal
    double x[3]; // reference point
    double rho; // distance from ref to origin
    vector<double*> hull; // convexhull
    char direction;

    double& operator()(int); // get normal xyz at index
    double& operator[](int); // get reference xyz at index

    void addConvexHull(vector<double*> &ps);

    Point* hullAsPointArr();

    // BEGIN COPYRIGHT
// Copyright 2001, 2012, 2021 Dan Sunday
// for the following code snippet: 

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
    static inline int
    isLeft( Point, Point, Point );
   
    //===================================================================

    // cn_PnPoly(): crossing number test for a point in a polygon
    //      Input:   P = a point,
    //               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
    //      Return:  0 = outside, 1 = inside
    // This code is patterned after [Franklin, 2000]
    static int
    cn_PnPoly( Point P, Point* V, int n );
    //===================================================================

    // wn_PnPoly(): winding number test for a point in a polygon
    //      Input:   P = a point,
    //               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
    //      Return:  wn = the winding number (=0 only when P is outside)
    static int
    wn_PnPoly( Point P, Point* V, int n );
    
    //===================================================================


    // END COPYRIGHT

    /*
    * Calculates the nearest line segment of point <p> to <polygon>. 
    * @param p - input: The point <p> that is checked.
    * @param poly - input: The <polygon> to check with.
    * @param p1 - output: start point of nearest line segment.
    * @param p2 - output: end point of nearest line segment.
    * @returns The minimum distance to the line segment.
    */
    static double nearestLineSegment(const Point& p, const Point* poly, int n, Point& p1, Point& p2);

    /*
     * Converts a 3D point that lies on a plane to 2D.
     * @param p - input: The point to be converted.
     * @param dir - input: main direction of the plane normal.
     * @param out - output: Resulting point.
     */
    static void convert3Dto2D(Point p, char dir, Point& out);

    /*
     * Converts multiple 3D points that lie on a plane into 2D.
     * @param ps - input: The points to be converted.
     * @param n - input: nr. of points.
     * @param dir - input: main direction of the plane normal.
     * @param out - output: Resulting points.
     */
    static void convert3Dto2D(Point* ps, int n, char dir, Point* out);

private:
    Point* _hull_parr;
};



#endif // __NORMALPLANE_H_