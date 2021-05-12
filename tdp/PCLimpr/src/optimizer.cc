/** @file 
 *  @brief This file implements the 6D gradient descent 
 *  to minimize point to plane correspondant distances. 
 *
 *  @author Fabian Arzberger, JMU, Germany.
 * 
 *  Released under the GPL version 3.
 */

#include "optimizer.h"
#include <newmat/newmatio.h> //For debuging
#include "planescan.h"
#include "slam6d/globals.icc"

// static members, program options
int Optimizer::_max_iter = MAX_ITER;
double Optimizer::_eps_convergence = EPS_CONVERGENCE;
double Optimizer::_alpha = ALPHA;
double Optimizer::_rPos_a_scale = RPOS_ALPHA_SCALE;
bool Optimizer::_anim = false;
bool Optimizer::_quiet = false;
bool Optimizer::_auto = true;
int Optimizer::_update_cor = 10;

// Base constructor
Optimizer::Optimizer()
{
    this->alpha = ColumnVector(6);
    /*
     * Typically, a change in orientation has much more impact on
     * the total error than a change in position. While translating 
     * the scan in x, y or z, the error grows linearily for all points.
     * However, when rotating about x, y or z, points that are further
     * away from the robot are moved even more dramatic, leading to 
     * a higher sensibility on the error function. For this reason,
     * the alpha applied on orientation must be much smaller than the 
     * alpha applied on the position.
     */ 
    this->a = _alpha;
    this->alpha << a 
                << a  
                << a
                << a * _rPos_a_scale 
                << a * _rPos_a_scale 
                << a * _rPos_a_scale; 
    this->J = ColumnVector(6);
    this->Ga = ColumnVector(6);
    this->Xa = ColumnVector(6);
    // Do not buffer these from RAM ! C++ optimization is shit
    Ga << 0 << 0 << 0 << 0 << 0 << 0;
    Xa << 0 << 0 << 0 << 0 << 0 << 0;
}

Optimizer::Optimizer(PlaneScan* pScan) : Optimizer()
{
    // Setup current state vector
    X0 = ColumnVector(6);
    X0 << pScan->rPosTheta[0]  // left-handed roll (rotX)
        << pScan->rPosTheta[1] // left-handed pitch (rotY)
        << pScan->rPosTheta[2] // left-handed yaw (rotZ)
        << pScan->rPos[0] //x
        << pScan->rPos[1] //y
        << pScan->rPos[2];//z
    this->X = X0;
    this->Xmin = X;
    this->ps = pScan;
    updateJacobian(); // init once before start iteration
    // add initial frame to begin the animation with
    if( _anim ) ps->addFrame( Scan::ICP ); // using ICP enum for nice colors in bin/show. nothing actually ICP here.
}

Optimizer::Optimizer(PlaneScan* pScan, Dimensions d) : Optimizer(pScan)
{
    dim = d;
}

/*
 * Static setters, program options
 */

void Optimizer::setMaxIter(int val)
{
    _max_iter = val;
}

void Optimizer::setEpsConvergence(double val)
{
    _eps_convergence = val;
}

void Optimizer::setAlphaInit(double val)
{
    _alpha = val;
}

void Optimizer::setRPosAlphaScale(double val)
{
    _rPos_a_scale = val;
}

void Optimizer::setAnim(bool val)
{
    _anim = val;
}

void Optimizer::setQuiet(bool val)
{
    _quiet = val;
}

void Optimizer::setAuto(bool val)
{
    _auto = val;
}

void Optimizer::setUpdateCor(int val)
{
    _update_cor = val;
}

/*
 * Instance accessors / mutators
 */

ColumnVector Optimizer::getResult()
{
    X = Xmin;
    updateScan();
    ps->addFrame( Scan::ICP );
    return X;
}

Optimizer::~Optimizer() {
        
}

Optimizer::Optimizer(Matrix &x0, Matrix &j) : Optimizer()
{
    X = x0;
    J = j;
}

void Optimizer::setState(ColumnVector &x) 
{
    X = x;
}

void Optimizer::setJacobian(Matrix &jac)
{
    J = jac;
}

void Optimizer::setState(ColumnVector state)
{
    // actually, i dont know if they are all necessary.
    // however, it works, so i wont change it.
    X0 = state;
    X = state;
    X1 = state;
    updateScan();
}

void Optimizer::relabel()
{
    // After having found the best transformation, label points again and repeat
    ps->correspondences.clear();
    ps->labelPoints(_quiet);
}

void Optimizer::adaptAlpha()
{
    // 1e-3 is a magic number that works quite well, used to avoid division with 0 
    switch(dim)
    {
        case ALL:
            alpha(1) = a * sqrt(1e-3 + Xa(1) ) / sqrt( 1e-3 + Ga(1) );
            alpha(2) = a * sqrt(1e-3 + Xa(2) ) / sqrt( 1e-3 + Ga(2) );
            alpha(3) = a * sqrt(1e-3 + Xa(3) ) / sqrt( 1e-3 + Ga(3) );
            alpha(4) = a * _rPos_a_scale * sqrt(1e-3 + Xa(4) ) / sqrt( 1e-3 + Ga(4) );
            alpha(5) = a * _rPos_a_scale * sqrt(1e-3 + Xa(5) ) / sqrt( 1e-3 + Ga(5) );
            alpha(6) = a * _rPos_a_scale * sqrt(1e-3 + Xa(6) ) / sqrt( 1e-3 + Ga(6) );
            break;
        case ROLLING:
            alpha(1) = a * sqrt(1e-3 + Xa(1) ) / sqrt( 1e-3 + Ga(1) );
            alpha(2) = 0;
            alpha(3) = 0;
            alpha(4) = 0;
            alpha(5) = a * _rPos_a_scale * sqrt(1e-3 + Xa(5) ) / sqrt( 1e-3 + Ga(5) );
            alpha(6) = a * _rPos_a_scale * sqrt(1e-3 + Xa(6) ) / sqrt( 1e-3 + Ga(6) );
            break;
        case DESCENDING:
            alpha(1) = 0;
            alpha(2) = a * sqrt(1e-3 + Xa(2) ) / sqrt( 1e-3 + Ga(2) );
            alpha(3) = 0;
            alpha(4) = 0;
            alpha(5) = a * _rPos_a_scale * sqrt(1e-3 + Xa(5) ) / sqrt( 1e-3 + Ga(5) );
            alpha(6) = 0;
            break;
        case ROTATING:
            alpha(1) = 0;
            alpha(2) = a * sqrt(1e-3 + Xa(2) ) / sqrt( 1e-3 + Ga(2) );
            alpha(3) = 0;
            alpha(4) = 0;
            alpha(5) = 0;
            alpha(6) = 0;
            break;
    }
}

void Optimizer::step()
{
    X1 = X;
    lastErr = sqrt(ps->calcErr());
    /*
    * Gradient descent step, using AdaDelta:
    * https://arxiv.org/pdf/1212.5701.pdf
    */
    Ga = 0.99*Ga + 0.01*SP(J, J); // acumulate averaged squared gradient
    adaptAlpha(); // use Ga and Xa to update alpha
    dX = SP(alpha, J); // compute update step
    Xa = 0.99*Xa + 0.01*SP(dX, dX); // accumulate averaged squared update
    X = X - dX; // apply update        
    updateScan(); // write new transformation to planescan object
    updateJacobian(); // set J(X_k) to J(X_k+1) for next iteration
    if ( sqrt(ps->calcErr()) < lastErr ) {
        Xmin = X; // always keep current minimum
        // only store relevant frame for animation, that is if RMSE decreased.
        if (_anim) ps->addFrame( Scan::ICP ); 
    }
}

void Optimizer::iterate(int n)
{
    if (ps->isEmpty()) return;
    for (int k = 0; k < _update_cor; ++k) 
    {
        for (int i = 0; i < n; ++i)
        {
            if ( !_quiet )
            {
                cout << "i = " << i << " in scan " << ps->identifier;
                cout << ", RMSE = " <<  sqrt( ps->calcErr() )   << endl;
            }
            step();
        }
        getResult();
        relabel();
    }
}

void Optimizer::operator()(int n)
{
    this->iterate(n);
}

// Iterates until convergence. Use with care! 
void Optimizer::iterate(double eps)
{
    if (ps->isEmpty()) return;
    for (int k = 0; k < _update_cor ;++k)
    {
        do
        {
            if ( !_quiet )
            {
                cout << "Scan " << ps->identifier;
                cout << ", RMSE = " <<  sqrt( ps->calcErr() )  << endl;
            }
            step();
        } while ( fabs(sqrt(ps->calcErr()) - lastErr) > eps );
        // After having found the best transformation, label points again and repeat
        getResult();
        relabel();
    }
}

// Iterates until convergence. Use with care! 
void Optimizer::operator()(double eps)
{
    this->iterate(eps);
}

// At maximum n iterations, quickstop if difference is too small
void Optimizer::iterate(int n, double eps)
{
    if (ps->isEmpty()) return;
    for (int k = 0; k < _update_cor; ++k) 
    {
        for (int i = 0; i < n; ++i)
        {
            if ( !_quiet )
            {
                cout << "i = " << i << " in scan " << ps->identifier;
                cout << ", RMSE = " <<  sqrt( ps->calcErr() )  << endl;
            }
            step();
            //If eps criterium is met, stop iteration.
            if ( fabs( sqrt(ps->calcErr()) - lastErr ) < eps ) break;
        }
        // After having found the best transformation, label points again and repeat
        getResult();
        relabel();
    }
}

void Optimizer::iterate()
{
    if ( _max_iter != -1 && _eps_convergence != -1 )
        this->iterate(_max_iter, _eps_convergence);
    else if ( _eps_convergence == -1 && _max_iter != -1)
        this->iterate(_max_iter);
    else if ( _max_iter == -1 && _eps_convergence != -1)
        this->iterate(_eps_convergence);
    else
        this->iterate(1000);
}

// Checks program options and iterates in the specified way
void Optimizer::operator()(void)
{
    if (_auto) { this->iterateAuto(); }
    iterate();
}


// Update jacobian matrix from state X
void Optimizer::updateJacobian()
{
    if (ps->isEmpty()) return;

    // Sum over correspondences
    Matrix sum = ColumnVector(6);
    sum << 0 << 0 << 0 << 0 << 0 << 0; // just to be sure.
    for (const auto& cor : ps->correspondences)
    {
        double nx = cor.second->n[0];
        double ny = cor.second->n[1];
        double nz = cor.second->n[2];
        double x = cor.first[0];
        double y = cor.first[1];
        double z = cor.first[2];

        double phi =  X(1) ; // rot x
        double theta = X(2) ; // rot y
        double psi =  X(3); // rot z
        double tx = X(4); // x
        double ty = X(5); // y
        double tz = X(6); // z
        
        double sph = sin(phi);
        double st = sin(theta);
        double sps = sin(psi);
        double cph = cos(phi);
        double ct = cos(theta);
        double cps = cos(psi);

        double D = nx*(x*ct*cps - y*cph*sps + z*st + tx - cor.second->x[0])
            + ny*(x*(cph*sps+cps*sph*st) + y*(cph*cps-sph*st*sps) - z*ct*sph + ty - cor.second->x[1])
            + nz*(x*(sph*sps-cph*cps*st) + y*(cps*sph+cph*st*sps) + z*cph*ct + tz - cor.second->x[2]);
        
        double dEdPhi = nx*y*sph*sps 
            + ny*(x*(cps*cph*st-sps*sph) + y*(-sph*cps-cph*st*sps) - z*ct*cph)
            + nz*(x*(cph*sps+sph*cps*st) + y*(cps*cph-sph*st*sps) - z*ct*sph);
        double dEdTheta = nx*(-x*st*cps+z*ct)
            + ny*(x*(ct*cps*cph) + y*(-ct*sph*sps) + z*st*sph)
            + nz*(-x*ct*cph*cps + y*ct*cph*sps - z*st*cph);
        double dEdPsi = nx*(-x*sps*ct - y*cps*cph)
            + ny*(x*(cps*cph-sps*sph*st) + y*(-sps*cph-cps*sph*st))
            + nz*(x*(cps*sph+sps*cph*st) + y*(-sps*sph+cps*cph*st));

        Matrix jacobian = ColumnVector(6);
        jacobian << 2*D*dEdPhi
            << 2*D*dEdTheta
            << 2*D*dEdPsi
            << 2*D*nx
            << 2*D*ny
            << 2*D*nz;

        sum += jacobian;
    }
    double norm = 1.0 / ps->correspondences.size();
    this->J = norm * sum;
}

// Update PlaneScan object from state X 
void Optimizer::updateScan()
{
    ps->rPosTheta[0] = X(1); // roll
    ps->rPosTheta[1] = X(2); // pitch
    ps->rPosTheta[2] = X(3); // yaw 
    ps->rPos[0] = X(4);
    ps->rPos[1] = X(5);
    ps->rPos[2] = X(6);
    EulerToMatrix4(ps->rPos, ps->rPosTheta, ps->transMat);
}

void Optimizer::reset()
{
    X = X0;
    Xmin = X;
    updateScan();
    Ga << 0 << 0 << 0 << 0 << 0 << 0;
    Xa << 0 << 0 << 0 << 0 << 0 << 0;
}

// Finds the optimal alpha to initialize AdaDelta
void Optimizer::iterateAuto()
{
    a = 0.01;
    double initErr, err;
    for (int i = 0; i < 10; ++i)
    {
        initErr = sqrt(ps->calcErr());
        for (int k = 0; k < 10; ++k) step();
        err = sqrt(ps->calcErr());
        reset();
        if ( err < initErr ) {
            if (!_quiet) cout << "Optimal alpha = " << a << endl;
            iterate(); // start the actual iteration, using specified criteria
            return;
        } else {
            a *= 0.1;
        }
    }
}