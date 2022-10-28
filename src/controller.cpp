/**
 *	\file src/controller.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

controller::controller() : saturator( )
{
    nInputs = 0;
    nOutputs = 0;
}


controller::controller( unsigned int _nInputs,
                        unsigned int _nOutputs,
                        float _samplingTime ) : saturator( _nOutputs, _samplingTime), filter( _nOutputs, _samplingTime )
{
    if ( ( _nOutputs != _nInputs ) && ( _nOutputs != 1 ) )
        _nOutputs = 1;

    nInputs = _nInputs;
    nOutputs = _nOutputs;

    samplingTime = _samplingTime;

    u = VectorXf::Zero( nOutputs );
    uSatDiff = VectorXf::Zero( nOutputs );
    lastU = u;
    yRef = VectorXf::Zero( nInputs );
}


controller::controller( unsigned int _nInputs,
                        unsigned int _nOutputs,
                        float _samplingTime,
                        float _omega_0 ) : saturator( _nOutputs, _samplingTime ), filter( _omega_0, _nOutputs, _samplingTime )
{
    if ( ( _nOutputs != _nInputs ) && ( _nOutputs != 1 ) )
        _nOutputs = 1;

    nInputs = _nInputs;
    nOutputs = _nOutputs;

    samplingTime = _samplingTime;

    u = VectorXf::Zero( nOutputs );
    uSatDiff = VectorXf::Zero( nOutputs );
    lastU = u;
    yRef = VectorXf::Zero( nInputs );
}


controller::controller( const controller& rhs ) : saturator( rhs )
{
    nInputs = rhs.nInputs;
    nOutputs = rhs.nOutputs;

    samplingTime = rhs.samplingTime;

    u = rhs.u;
    yRef = rhs.yRef;
}


controller::~controller(  ){}


void controller::setPolynomialReference( const MatrixXf& _refCoeff )
{
    if ( _refCoeff.rows() != nInputs )
        throw std::invalid_argument("Incorrect number of reference trajectories given");
    else
        refCoeff = _refCoeff;
}


void controller::step( double currentTime, const VectorXf& _x, const VectorXf& _yRef )
{
    VectorXf temp( nU );

    if ( _x.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of inputs given to controller");

    // Set reference trajectory
    if ( _yRef.size() > 0 )
    {
        if ( _yRef.size() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given");
        else
            yRef = _yRef;
    }
    else
        yRef.setZero();

    // Determine PID control action
    if ( nOutputs > 0 )
        determineControlAction( yRef - _x,u );
    else
        u.setZero();


    // Saturate control input
    temp = u;
    saturate( u );

    // Anti wind-up
    for (unsigned int i=0; i<nU; ++i)
    {
        if (temp(i)-u(i) > 0)
            uSatDiff(i) = temp(i)-u(i);
        else
            uSatDiff(i) = 0.0;
    }

    // Filter signal
    filterSignal( u, u );


    // Save last control output
    lastU = u;
}


void controller::step( double currentTime, const VectorXf& _x )
{
    if ( _x.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of inputs given to controller");

    // Get reference trajectory
    yRef.setZero();

    if ( refCoeff.rows() > 0 )
    {
        if ( refCoeff.rows() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given");
        else
            for ( unsigned int i=0; i<refCoeff.rows(); ++i)
            {
                for ( unsigned int j=0; j<refCoeff.cols(); ++j)
                {
                    yRef(i) += refCoeff(i,j) * pow( currentTime,refCoeff.cols()-j-1 );
                }
            }
    }
    else
        yRef.setZero();

    // Determine PID control action
    if ( nOutputs > 0 )
        determineControlAction( yRef - _x,u );
    else
        u.setZero();
    
    // Saturate control input
    saturate( u );

    // Save last control output
    lastU = u;
}
