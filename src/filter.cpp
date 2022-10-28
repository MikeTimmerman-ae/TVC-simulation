/**
 *	\file src/filter.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

filter::filter(  ) {}


filter::filter( int _nu, float _samplingTime )
{   
    nu = _nu;
    omega_0 = 0;
    dt = _samplingTime;

    prevX = MatrixXf::Zero(nu,1);
    prevY = MatrixXf::Zero(nu,1);

    a_coeff = VectorXf::Zero(1);
    b_coeff = VectorXf::Zero(2);

    a_coeff[0] = 0;
    b_coeff[0] = 1;
    b_coeff[1] = 0;
}


filter::filter( float _omega_0, int _nu, float _samplingTime )
{   
    nu = _nu;
    omega_0 = _omega_0;
    dt = _samplingTime;

    a_coeff = VectorXf::Zero(1);
    b_coeff = VectorXf::Zero(2);

    prevX = MatrixXf::Zero(nu,1);
    prevY = MatrixXf::Zero(nu,1);

    a_coeff[0] = (2 - omega_0*dt) / (2 + omega_0*dt);
    b_coeff[0] = omega_0*dt / (2 + omega_0*dt);
    b_coeff[1] = b_coeff[0];
}


filter::filter( const filter& rhs )
{
    omega_0 = rhs.omega_0;
    dt = rhs.dt;

    prevX = rhs.prevX;
    prevY = rhs.prevY;

    a_coeff = rhs.a_coeff;
    b_coeff = rhs.b_coeff;
}


filter::~filter(  ){}



//
// PRIVATE MEMBER FUNCTIONS:
//

void filter::filterSignal( VectorXf _x, VectorXf& _y )
{
    for (unsigned int i=0; i<nu; ++i)
    {
        _y(i) = a_coeff[0] * prevY(i,0) + b_coeff[0] * _x(i) + b_coeff[1] * prevX(i,0);
    }

    prevX(seq(0,nu-1),0) = _x;
    prevY(seq(0,nu-1),0) = _y;
}