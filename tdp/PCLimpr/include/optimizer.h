/** @file 
 *  @brief This file implements the 6D gradient descent 
 *  to minimize point to plane correspondant distances. 
 *
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#ifndef __NEWTONS_H_
#define __NEWTONS_H_

#include <newmat/newmat.h>

#include <utility>
#include "planescan.h"

using namespace NEWMAT;

#define EPS_CONVERGENCE (-1)
#define MAX_ITER (-1)
#define ALPHA 0.001
#define RPOS_ALPHA_SCALE 100

// Used to lock some dimensions from being corrected.
enum Dimensions {
    ALL,            // performs 6D optimization
    ROLLING,        // only optimizes rotX, as well as y and z (forward rolling motion, e.g. L.U.N.A sphere)
    DESCENDING,     // only optimizes rotY and y (e.g. DAEDALUS descent phase) 
    ROTATING        // only optimizes rotY (e.g. terrestial stationary RIEGL laserscanner)
};

class Optimizer
{
public:
    Optimizer();
    // Construct with Optimizer(x0, jacobian)
    Optimizer(Matrix&, Matrix&);
    // Construct from a PlaneScan directly
    Optimizer(PlaneScan*);
    // Construct with predefined dimensions to optimize
    Optimizer(PlaneScan*, Dimensions);
    ~Optimizer();

    // Run <arg> iterations of gradient descent 
    void iterate(int);
    void operator()(int);

    // Run until state difference between iterations is smaller then <arg>
    void iterate(double);
    void operator()(double);

    // Combination of both top approaches
    /*
     * The overloaded operator() allows you to initialize 
     * and run an Optimizer instance in the following manner:
     * 
     * Optimizer iter(X_0, jacobian);
     * Optimizer::setMaxIter(10000);
     * Optimizer::setEpsConvergence(0.0001);
     * iter();
     * 
     * Use -E and -i to set these.
     * Multiple criteria are supported.
     * Running without criteria runs 1000 iterations.
     */
    void iterate(int, double);
    void iterate();
    void operator()(void);

    // Auto-detect optimal initial alpha
    void iterateAuto();

    void setPlaneScan(PlaneScan&);
    void setState(ColumnVector&);
    void setJacobian(Matrix&);

    // Overwrites predefined header constants
    static void setEpsConvergence(double);
    static void setMaxIter(int);
    // Sets a static alpha to use for every state variable
    static void setAlphaInit(double);
    static void setRPosAlphaScale(double);
    static void setAnim(bool);
    static void setQuiet(bool);
    static void setAuto(bool);
    static void setUpdateCor(int);

    // Returns minimum state vector Xmin
    ColumnVector getResult();
    void setState(ColumnVector);
    
    // relabel the points in the scan after transformations have been applied
    void relabel();

private:
    PlaneScan* ps;
    ColumnVector X0; // initial state. keep in order to reset if needed.
    ColumnVector X; // State vector / iteration variable. Represents x0 in first iteration.
    ColumnVector X1; // State vector of the very last iteration.
    ColumnVector Xmin; // always holds the state where RMSE is minimum.
    ColumnVector dX; // Update applied to X in every iteration.
    ColumnVector alpha; // gradient descent factor / convergence rate.
    ColumnVector Ga; // decaying squared accumulated gradient, used for adaptive alpha
    ColumnVector Xa; // decaying squared accumulated update, used for adaptive alpha
    Matrix J; // Jacobian (Gradient)

    double lastErr; // stores last point-2-plane error --> sqrt(ps->calcError())
    double a; // each optimizer can have different alphas

    Dimensions dim;

    static int _update_cor; // update correspondences this often
    static int _max_iter; // run this many iterations before updating correspondences
    static double _eps_convergence;
    static double _alpha; // static alpha used for initilazition
    static double _rPos_a_scale; // scales alpha applied to translation
    static bool _anim;
    static bool _quiet;
    static bool _auto; // auto detect optimum alpha, used for speedup
 
    // Computes only one step of gradient descent. Updates Transformation and writes animation.
    void step();
    // Resets the PlaneScan object to its initial state, i.e. the orientation in the .pose file
    void reset();
    
    // Update jacobian from state vector X 
    void updateJacobian();
    // Update planescan object from state vector X
    void updateScan();
    void adaptAlpha();

    
};

#endif // __NEWTONS_S