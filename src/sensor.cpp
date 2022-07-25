/**
 *	\file src/sensor.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

sensor::sensor(  )
{
    omega = VectorXf::Zero(3);
    EulerAnglesVector = VectorXf::Zero(3);
    QuaternionVector = VectorXf::Zero(4);
    AccelVector = VectorXf::Zero(3);
    GravityVector = VectorXf::Zero(3);
    LinAccelVector = VectorXf::Zero(3);

    PositionVector = VectorXf::Zero(3);
}


void sensor::processOutput( VectorXf& _y )
{
    if ( _y.size() != 18 )
        throw std::invalid_argument("Incorrect system output vector given");
    
    omega = _y( seq( 3,5 ) );
    EulerAnglesVector = _y( seq( 0,2 ) );
    QuaternionVector = toQuaternion( EulerAnglesVector );
    AccelVector = _y( seq( 12,14 ) );
    GravityVector = _y( seq( 15,17 ) );
    LinAccelVector = AccelVector - GravityVector;

    PositionVector = _y( seq( 6,8 ) );
}


IMUsensor::IMUsensor(  ) : sensor(  ) {}


void IMUsensor::EulerAngles( VectorXf& _yout )
{
    if ( _yout.size() != EulerAnglesVector.size() )
        throw std::invalid_argument("Incorrect number of output dimensions for euler angles");
    else
        _yout = EulerAnglesVector;
}

void IMUsensor::Quaternion( VectorXf& _yout )
{
    if ( _yout.size() != QuaternionVector.size() )
        throw std::invalid_argument("Incorrect number of output dimensions for quaternion");
    else
        _yout = QuaternionVector;
}

void IMUsensor::AngularVel( VectorXf& _yout )
{
    if ( _yout.size() != omega.size() )
        throw std::invalid_argument("Incorrect number of output dimensions for angular velocity");
    else
        _yout = omega;
}

void IMUsensor::Acceleration( VectorXf& _yout )
{
    if ( _yout.size() != AccelVector.size() )
        throw std::invalid_argument("Incorrect number of output dimensions for accelerarion");
    else
        _yout = AccelVector;
}

void IMUsensor::LinAcceleration( VectorXf& _yout )
{
    if ( _yout.size() != LinAccelVector.size() )
        throw std::invalid_argument("Incorrect number of output dimensions for linear acceleration vector");
    else
        _yout = LinAccelVector;
}

void IMUsensor::GravityVec( VectorXf& _yout )
{
    if ( _yout.size() != GravityVector.size() )
        throw std::invalid_argument("Incorrect number of output dimensions for gravity vector");
    else
        _yout = GravityVector;
}

void IMUsensor::PositionVec( VectorXf& _yout )
{
    if ( _yout.size() != PositionVector.size() )
        throw std::invalid_argument("Incorrect number of output dimensions for position vector");
    else
        _yout = PositionVector;
}