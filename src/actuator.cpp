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

actuator::actuator(  ) {}


actuator::actuator( int _nu, VectorXf _initControl, float _samplingTime  )
{
    lowerLimits = VectorXf::Ones( _nu )*-1000000;
    upperLimits = VectorXf::Ones( _nu )*1000000;

    lowerRateLimits = VectorXf::Ones( _nu )*-1000000;
    upperRateLimits = VectorXf::Ones( _nu )*1000000;

    nu = _nu;
    samplingTime = _samplingTime;
    if ( _initControl.size() > _nu )
        throw std::invalid_argument("Incorrect number of initial control inputs given");
    else if ( _initControl.size() == 0 )
        lastU = VectorXf::Zero( _nu );
    else
        lastU = _initControl;
}


actuator::actuator( const actuator& rhs )
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


void actuator::setLowerControlLimit( const VectorXf& _lowerLimits  )
{
    if ( _lowerLimits.size() != nu ) 
        throw std::invalid_argument("Incorrect number of lower control limits given");
    else
        lowerLimits = _lowerLimits;
}

void actuator::setUpperControlLimit( const VectorXf& _upperLimits )
{
    if ( _upperLimits.size() != nu )
        throw std::invalid_argument("Incorrect number of upper control limits given");
    else
        upperLimits = _upperLimits;
}

void actuator::setLowerControlLimit( int idx, const float _lowerLimit )
{
    if ( idx >= nu )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        lowerLimits = VectorXf::Ones(nu)*_lowerLimit;
    else
        lowerLimits(idx) = _lowerLimit;
}

void actuator::setUpperControlLimit( int idx, const float _upperLimit )
{
    if ( idx >= nu )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        upperLimits = VectorXf::Ones(nu)*_upperLimit;
    else
        upperLimits(idx) = _upperLimit;
}


void actuator::setLowerRateLimit( const VectorXf& _lowerRateLimits )
{
    if ( _lowerRateLimits.size() != nu ) 
        throw std::invalid_argument("Incorrect number of lower rate limits given");
    else
        lowerRateLimits = _lowerRateLimits;
}

void actuator::setUpperRateLimit( const VectorXf& _upperRateLimits )
{
    if ( _upperRateLimits.size() != nu )
        throw std::invalid_argument("Incorrect number of upper rate limits given");
    else
        upperRateLimits = _upperRateLimits;
}

void actuator::setLowerRateLimit( int idx, const float _lowerRateLimit )
{
    if ( idx >= nu )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        lowerRateLimits = VectorXf::Ones(nu)*_lowerRateLimit;
    else
        lowerRateLimits(idx) = _lowerRateLimit;
}

void actuator::setUpperRateLimit( int idx, const float _upperRateLimit )
{
    if ( idx >= nu )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        upperRateLimits = VectorXf::Ones(nu)*_upperRateLimit;
    else
        upperRateLimits(idx) = _upperRateLimit;
}




void actuator::actuate( VectorXf& _u )
{
    unsigned int i;

    // Consitency check
    if ( _u.size() != nu )
        throw std::invalid_argument("Incorrect number of control signals given");
    
    // Set upper and lower bounds
    VectorXf Uub( upperLimits );
    VectorXf Ulb( lowerLimits );

    for ( i=0; i<nu; ++i )
    {
        if ( lastU(i) + samplingTime*lowerRateLimits(i) > Ulb(i) )
            Ulb(i) = lastU(i) + samplingTime*lowerRateLimits(i);
        if ( lastU(i) + samplingTime*upperRateLimits(i) < Uub(i) )
            Uub(i) = lastU(i) + samplingTime*upperRateLimits(i);
    }

    // Update control input
    for ( i=0; i<nu; ++i )
    {
        if ( _u(i) > Uub(i) )
            _u(i) = Uub(i);
        if ( _u(i) < Ulb(i) )
            _u(i) = Ulb(i);
    }
    controlRate = (_u - lastU) / samplingTime;
    lastU = _u;
}


