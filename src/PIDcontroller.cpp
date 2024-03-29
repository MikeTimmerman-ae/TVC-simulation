/**
 *	\file src/PIDcontroller.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

PIDcontroller::PIDcontroller(  ) : controller(  ) {}


PIDcontroller::PIDcontroller(   unsigned int _nInputs,
                                unsigned int _nOutputs,
                                float samplingTime    ) : controller( _nInputs, _nOutputs, samplingTime )
{
	pGains = VectorXf::Zero( nInputs );
	iGains = VectorXf::Zero( nInputs );
	dGains = VectorXf::Zero( nInputs );

    iValue = VectorXf::Zero( nInputs );
    dValue = VectorXf::Zero( nInputs );
    pValue = VectorXf::Zero( nInputs );
    lastError = VectorXf::Zero( nInputs );
}


PIDcontroller::PIDcontroller(   unsigned int _nInputs,
                                unsigned int _nOutputs,
                                float samplingTime,
                                float _omega_0    ) : controller( _nInputs, _nOutputs, samplingTime, _omega_0 )
{
	pGains = VectorXf::Zero( nInputs );
	iGains = VectorXf::Zero( nInputs );
	dGains = VectorXf::Zero( nInputs );

    iValue = VectorXf::Zero( nInputs );
    dValue = VectorXf::Zero( nInputs );
    pValue = VectorXf::Zero( nInputs );
    lastError = VectorXf::Zero( nInputs );
}


PIDcontroller::PIDcontroller( const PIDcontroller& rhs ) : controller( rhs )
{
	pGains = rhs.pGains;
	iGains = rhs.iGains;
	dGains = rhs.dGains;

	iValue    = rhs.iValue;
	lastError = rhs.lastError;
}


PIDcontroller::~PIDcontroller(  ){}


void PIDcontroller::setProportionalGains( const VectorXf& _pGains )
{
    if ( _pGains.size() != nInputs )
        throw std::invalid_argument("Number of proportional gains does not match number of controller inputs");
    else
        pGains = _pGains;
}


void PIDcontroller::setIntegralGains( const VectorXf& _iGains )
{
    if ( _iGains.size() != nInputs )
        throw std::invalid_argument("Number of integral gains does not match number of controller inputs");
    else
        iGains = _iGains;
}


void PIDcontroller::setDerivativeGains( const VectorXf& _dGains )
{
    if ( _dGains.size() != nInputs )
        throw std::invalid_argument("Number of derivative gains does not match number of controller inputs");
    else
        dGains = _dGains;
}


void PIDcontroller::init( const VectorXf& _x0, const VectorXf& _initU, const VectorXf& _yRef, double startTime )
{
    if ( _x0.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of state dimensions to initialize controller");

    // Set reference trajectory
    VectorXf yRef( _x0.size() );

    if ( _yRef.size() > 0 )
    {
        if ( _yRef.size() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given to initialize controller");
        else
            yRef = _yRef;
    }
    else
        yRef.setZero();

    // Set initial control output
    lastU = _initU;

    // Set initial error
    lastError = yRef - _x0;
}


void PIDcontroller::init( const VectorXf& _x0, const VectorXf& _initU, double startTime )
{
    if ( _x0.size() != nInputs ) 
        throw std::invalid_argument("Incorrect number of state dimensions to initialize controller");

    // Get reference trajectory
    VectorXf yRef( _x0.size() ); yRef.setZero();

    if ( refCoeff.rows() > 0 )
    {
        if ( refCoeff.rows() != nInputs )
            throw std::invalid_argument("Incorrect number of reference trajectories given to initialize controller");
        else
            for ( unsigned int i=0; i<refCoeff.rows(); ++i)
            {
                for ( unsigned int j=0; j<refCoeff.cols(); ++j)
                {
                    yRef(i) += refCoeff(i,j) * pow( startTime,refCoeff.cols()-j-1 );
                }
            }
    }
    else
        yRef.setZero();

    // Set initial control output
    lastU = _initU;

    // Set initial error
    lastError = yRef - _x0;
}


void PIDcontroller::determineControlAction( const VectorXf& error, VectorXf& output )
{
    unsigned int i;
    double tmp;

    output = VectorXf::Zero( nOutputs );

    // Calculate integral, derivative, proportional value
    for ( i=0; i<nInputs; ++i )
    {
        iValue(i) += iGains(i)*(error(i) + lastError(i)) * samplingTime / 2.0;
        dValue(i) = 2.0*dGains(i)/(2.0/dOmega + samplingTime) * (error(i) - lastError(i)) + (2.0/dOmega-samplingTime)/(2.0/dOmega+samplingTime)*dValue(i);
        pValue(i) =  pGains(i) * error(i);
    }
    
    // Anti wind-up on integral term
    double upperLimitInt;
    double lowerLimitInt;
    for ( i=0; i<nInputs; ++i )
    {
        if (upperLimits(i) > pValue(i))
            upperLimitInt = upperLimits(i) - pValue(i);
        else
            upperLimitInt = 0.0;
        if (lowerLimits(i) < pValue(i))
            lowerLimitInt = lowerLimits(i) - pValue(i);
        else
            lowerLimitInt = 0.0;

        if (iValue(i) > upperLimitInt)
            iValue(i) = upperLimitInt;
        else if (iValue(i) < lowerLimitInt)
            iValue(i) = lowerLimitInt;
    }
    
    // determine ouputs
    for ( i=0; i<nInputs; ++i )
    {
        tmp  = pValue(i);
        tmp += iValue(i);
        tmp += dValue(i);

        if ( nOutputs > 1  )
            output(i) = tmp;
        else
            output(0) += tmp;
    }

    // update last error
    lastError = error;
}

