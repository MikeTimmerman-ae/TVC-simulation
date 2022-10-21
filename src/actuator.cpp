/**
 *	\file src/actuator.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

actuator::actuator(  ) : saturator(  ) {}


actuator::actuator( int _nu, VectorXf _initControl, float _samplingTime  ) : saturator( _nu, _samplingTime )
{
    nu = _nu;
    samplingTime = _samplingTime;
    
    if ( _initControl.size() > _nu )
        throw std::invalid_argument("Incorrect number of initial control inputs given");
    else if ( _initControl.size() == 0 )
        lastU = VectorXf::Zero( _nu );
    else
        lastU = _initControl;
}


actuator::actuator( const actuator& rhs ) : saturator( rhs )
{
    nu = rhs.nu;
    samplingTime = rhs.samplingTime;

    lastU = rhs.lastU;

    lowerLimits = rhs.lowerLimits;
    upperLimits = rhs.upperLimits;

    lowerRateLimits = rhs.lowerRateLimits;
    upperRateLimits = rhs.upperRateLimits;
}


actuator::~actuator(  ) {}



void actuator::actuate( VectorXf& _u )
{
    // Saturate control input
    saturate( _u );

    // Compute control input rate
    controlRate = (_u - lastU) / samplingTime;

    // Save last control output
    lastU = _u;
}


