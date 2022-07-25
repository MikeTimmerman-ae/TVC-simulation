/**
 *	\file src/dynamics.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

dynamics::dynamics( ) {}


dynamics::dynamics( VectorXf _initState,
                    float _initTime,
                    float _samplingTime )
{
    state = _initState;
    time = _initTime;
    samplingTime = _samplingTime;
}


dynamics::~dynamics( ) {}


void dynamics::step( VectorXf& _u, VectorXf& _y )
{
    /* Update system state */
    updateState( _u );

    /* System output */
    VectorXf tmp( ny ); tmp << state[0], state[1], state[8];
    _y = tmp;
}



//
// PRIVATE MEMBER FUNCTIONS:
//

void dynamics::updateState( VectorXf& _u )
{
    // Evaluation at start of interval
    k1 = EOM( time, state, _u );

    // Evaluation at midway of interval
    k2 = EOM( time + 1/2*samplingTime, state + samplingTime*k1/2.0, _u );

    k3 = EOM( time + 1/2*samplingTime, state + samplingTime*k2/2.0, _u );

    // Evaluation at end of interval
    k4 = EOM( time + samplingTime, state + samplingTime*k3, _u );

    // Update system state and current time   
    state = state + samplingTime*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
    time = time + samplingTime;
}


VectorXf dynamics::EOM(   float _t, VectorXf x, const VectorXf& _u    )
{
    VectorXf M(3); M = calculateMoment( _t, x, _u );
    VectorXf F(3); F = calculateForce( _t, x, _u );

    stateDerivative[0] = x[3] + x[4] * tan(x[1]) * sin(x[0]) + x[5] * tan(x[1]) * cos(x[0]);                                    // Phi - roll angle (E-frame)
    stateDerivative[1] = x[4] * cos(x[0]) - x[5] * sin(x[0]);                                                                   // Theta - pitch angle (E-frame)
    stateDerivative[2] = x[4] * sin(x[0]) / cos(x[1]) - x[5]  * cos(x[0]) / cos(x[1]);                                          // Psi - yaw angle (E-frame)
    stateDerivative[3] = ((Iyy-Izz) * x[4]*x[5] + ( pow(x[5],2) - pow(x[4],2) ) * Iyz + Ixy*x[3]*x[5] - Ixz*x[3]*x[4])/Ixx + M(0)/Ixx;     // p - roll rate (B-frame)
    stateDerivative[4] = ((Izz-Ixx) * x[3]*x[5] + ( pow(x[3],2) - pow(x[5],2) ) * Ixz + Iyz*x[4]*x[3] - Ixy*x[4]*x[5])/Iyy + M(1)/Iyy;     // q - pitch rate (B-frame)
    stateDerivative[5] = ((Ixx-Iyy) * x[3]*x[4] + ( pow(x[4],2) - pow(x[3],2) ) * Ixy + Ixz*x[5]*x[4] - Iyz*x[3]*x[5])/Izz + M(2)/Izz;     // r - yaw rate (B-frame)
    stateDerivative[6] = (cos(x[1])*cos(x[2])) * x[9] + (sin(x[0])*sin(x[1])*cos(x[2]) - cos(x[0])*sin(x[2])) * x[10] + (sin(x[0])*sin(x[2]) + cos(x[0])*sin(x[1])*cos(x[2])) * x[11];  // x - position x-axis (E-frame)
    stateDerivative[7] = (cos(x[1])*sin(x[2])) * x[9] + (cos(x[0])*cos(x[2]) + sin(x[0])*sin(x[1])*sin(x[2])) * x[10] + (cos(x[0])*sin(x[1])*sin(x[2]) - sin(x[0])*cos(x[2])) * x[11];  // y - position y-axis (E-frame)
    stateDerivative[8] = (-sin(x[1])) * x[9] + (sin(x[0])*cos(x[1])) * x[10] + (cos(x[0])*cos(x[1])) * x[11];                                                                    // z - position z-axis (E-frame)     
    stateDerivative[9] = x[5]*x[10] - x[4]*x[11] + F(0)/mass;                                                                      // u - velocity x-axis (B-frame)
    stateDerivative[10] = x[3]*x[11] - x[9]*x[5] + F(1)/mass;                                                                     // v - velocity y-axis (B-frame)
    stateDerivative[11] = x[4]*x[9] - x[3]*x[10] + F(2)/mass;                                                                     // w - velocity z-axis (B-frame)

    return stateDerivative;
}


VectorXf dynamics::calculateForce(  float _t, VectorXf& _x, const VectorXf& _u )
{
    float theta1 = _u(0);           // gimbal rotation around x-axis
    float theta2 = _u(1);           // gimbal rotation around y-axis
    float omega1 = _u(2);           // ccw positive rotating propeller rotational velocity (upper prop)
    float omega2 = -_u(2);          // cw negative rotating propeller rotational velocity (bottom prop)

    // Gravity
    VectorXf Fg(3);
    Fg(0)=-sin(_x[1])*9.81*mass;
    Fg(1)=sin(_x[0])*cos(_x[1])*9.81*mass;
    Fg(2)=cos(_x[0])*cos(_x[1])*9.81*mass;

    // Thrust
    VectorXf Ft(3);
    Ft(0)=-sin(theta2) * (kf1*omega1 + kf2*omega2);
    Ft(1)=sin(theta1)*cos(theta2) * (kf1*omega1 + kf2*omega2);
    Ft(2)=-cos(theta1)*cos(theta2) * (kf1*omega1 + kf2*omega2);

    // Aerodynamic Force
    VectorXf Fa(3);
    Fa(0) = 1/2 * rho * pow( _x[9],2 ) * Cdx * Ax;
    Fa(1) = 1/2 * rho * pow( _x[10],2 ) * Cdy * Ay;
    Fa(2) = 1/2 * rho * pow( _x[11],2 ) * Cdz * Az;

    return Fa+Fg+Ft;
}


VectorXf dynamics::calculateMoment(  float _t, VectorXf& _x, const VectorXf& _u )
{
    float theta1 = _u(0);           // gimbal rotation around x-axis
    float theta2 = _u(1);           // gimbal rotation around y-axis
    float omega1 = _u(2);           // ccw positive rotating propeller rotational velocity (upper prop)
    float omega2 = -_u(2);          // cw negative rotating propeller rotational velocity (bottom prop)

    // Gyrocopic moment due to rotation of propellers
    VectorXf Mr(3);
    Mr(0) = 0;
    Mr(1) = 0;
    Mr(2) = 0;

    // Control moment
    VectorXf Mc(3);
    Mc(0) = -rcg*sin(theta1)*cos(theta2) * (kf1*omega1 + kf2*omega2) - sin(theta2) * (km1*omega1 + km2*omega2);
    Mc(1) = -rcg*sin(theta2)*(kf1*omega1 + kf2*omega2) + sin(theta1)*cos(theta2) * (km1*omega1 + km2*omega2);
    Mc(2) = -cos(theta1)*cos(theta2) * (km1*omega1 + km2*omega2);
    
    // Aerodynamic moment
    VectorXf Ma(3);
    Ma(0) = 1/2 * rho * pow( _x[9],2 ) * Cdx * Ax * rcp;
    Ma(1) = 1/2 * rho * pow( _x[10],2 ) * Cdy * Ay * rcp;
    Ma(2) = 0;

    // Thrust Offset
    VectorXf Mt(3);
    Mt(0)= cos(theta1)*cos(theta2) * (kf1*omega1 + kf2*omega2) * thrustOffsetY;
    Mt(1)=-cos(theta1)*cos(theta2) * (kf1*omega1 + kf2*omega2) * thrustOffsetX;
    Mt(2)= 0;
    
    // Disturbance moment
    VectorXf Md=VectorXf::Zero(3);

    return Ma+Md+Mc-Mr+Mt;
}
