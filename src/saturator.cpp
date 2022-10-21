/**
 *	\file src/saturator.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

saturator::saturator(  ){}


saturator::saturator( unsigned int _nU, float _samplingTime )
{   
    lowerLimits = VectorXf::Ones( _nU )*-1000000;
    upperLimits = VectorXf::Ones( _nU )*1000000;

    lowerRateLimits = VectorXf::Ones( _nU )*-1000000;
    upperRateLimits = VectorXf::Ones( _nU )*1000000;

    nU = _nU;
    samplingTime = _samplingTime;
}


saturator::saturator( const saturator& rhs )
{
    lowerLimits = rhs.lowerLimits;
    upperLimits = rhs.upperLimits;

    lowerRateLimits = rhs.lowerRateLimits;
    upperRateLimits = rhs.upperRateLimits;

    nU = rhs.nU;
    samplingTime = rhs.samplingTime;
    lastU = rhs.lastU;
}


saturator::~saturator(  ){}



void saturator::setLowerControlLimit( const VectorXf& _lowerLimit )
{
    if ( _lowerLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control limits given");
    
    lowerLimits = _lowerLimit;
}

void saturator::setLowerControlLimit( int idx, float _lowerLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        lowerLimits = VectorXf::Ones(nU)*_lowerLimit;
    else
        lowerLimits(idx) = _lowerLimit;
}

void saturator::setUpperControlLimit( const VectorXf& _upperLimit )
{
    if ( _upperLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control limits given");
    
    upperLimits = _upperLimit;
}

void saturator::setUpperControlLimit( int idx, float _upperLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        upperLimits = VectorXf::Ones(nU)*_upperLimit;
    else
        upperLimits(idx) = _upperLimit;
}


void saturator::setLowerRateLimit( const VectorXf& _lowerRateLimit )
{
    if ( _lowerRateLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control rate limits given");
    
    lowerRateLimits = _lowerRateLimit;
}

void saturator::setLowerRateLimit( int idx, float _lowerRateLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        lowerRateLimits = VectorXf::Ones(nU)*_lowerRateLimit;
    else
        lowerRateLimits(idx) = _lowerRateLimit;
}

void saturator::setUpperRateLimit( const VectorXf& _upperRateLimit )
{
    if ( _upperRateLimit.size() != nU )
        throw std::invalid_argument("Incorrect number of control rate limits given");
    
    upperRateLimits = _upperRateLimit;
}

void saturator::setUpperRateLimit( int idx, float _upperRateLimit )
{
    if ( idx >= nU )
        throw std::invalid_argument("Invalid index for control signal given");
    else if (idx < 0)
        upperRateLimits = VectorXf::Ones(nU)*_upperRateLimit;
    else
        upperRateLimits(idx) = _upperRateLimit;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

void saturator::saturate( VectorXf& _u )
{   
    // consistency check
    if ( _u.size() != nU )
        throw std::invalid_argument("Incorrect number of control signals given");

    // Set upper and lower bounds
    VectorXf Uub( upperLimits );
    VectorXf Ulb( lowerLimits );

    for ( unsigned int i=0; i<nU; ++i )
    {
        if (lastU(i) + samplingTime*lowerRateLimits(i) > Ulb(i))
            Ulb(i) = lastU(i) + samplingTime*lowerRateLimits(i);
        if (lastU(i) + samplingTime*upperRateLimits(i) < Uub(i))
            Uub(i) = lastU(i) + samplingTime*upperRateLimits(i);
    }

    // Update control input
    for ( unsigned int i=0; i<nU; ++i )
    {
        if (_u(i) > Uub(i))
            _u(i) = Uub(i);
        if (_u(i) < Ulb(i))
            _u(i) = Ulb(i);
    }
}