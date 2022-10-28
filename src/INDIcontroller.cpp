/**
 *	\file src/INDIcontroller.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


//
// PUBLIC MEMBER FUNCTIONS:
//

INDIcontroller::INDIcontroller(  ) : controller(  )
{
    currentInput = VectorXf::Zero(3);
    controlEffectiveness = MatrixXf::Zero(3,3);
}


INDIcontroller::INDIcontroller( unsigned int _nInputs,
                                unsigned int _nOutputs,
                                float samplingTime    ) : controller( _nInputs, _nOutputs, samplingTime )
{
    currentInput = VectorXf::Zero( _nInputs );
    controlEffectiveness = MatrixXf::Zero( _nOutputs,_nInputs );
}

INDIcontroller::INDIcontroller( unsigned int _nInputs,
                                unsigned int _nOutputs,
                                float samplingTime,
                                float _omega_0   ) : controller( _nInputs, _nOutputs, samplingTime, _omega_0 )
{
    currentInput = VectorXf::Zero( _nInputs );
    controlEffectiveness = MatrixXf::Zero( _nOutputs,_nInputs );
}

INDIcontroller::INDIcontroller( const INDIcontroller& rhs ) : controller( rhs )
{
	currentInput = rhs.currentInput;
    controlEffectiveness = rhs.controlEffectiveness;
}


INDIcontroller::~INDIcontroller(  ){}

//
// PRIVATE MEMBER FUNCTIONS:
//

void INDIcontroller::computeControlEffectiveness( VectorXf& currentAttitude, VectorXf& currentGimbal, VectorXf& currentOmega, VectorXf& parameters )
{   
    Matrix3f temp;

    float theta1 = currentGimbal(0); float theta2 = currentGimbal(1);
    float phi = currentAttitude(0); float theta = currentAttitude(1); float psi = currentAttitude(2);
    float m = parameters(0); float kf = parameters(1);
    
    VectorXf gimbalTransformation(3);
    gimbalTransformation(0) = sin(theta2);
    gimbalTransformation(1) = -sin(theta1)*cos(theta2);
    gimbalTransformation(2) = cos(theta1)*cos(theta2);

    Matrix3f bodyTransfomationPhi;
    bodyTransfomationPhi << 0, sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi), sin(psi)*cos(phi)-cos(psi)*sin(theta)*sin(phi),
                            0, -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi), -cos(psi)*cos(phi)-sin(psi)*sin(theta)*sin(phi),
                            0, cos(theta)*cos(phi), -cos(theta)*sin(phi);

    Matrix3f bodyTransfomationTheta;
    bodyTransfomationTheta <<   -cos(psi)*sin(theta), cos(psi)*cos(theta)*sin(phi), cos(psi)*cos(theta)*cos(phi),
                                -sin(psi)*sin(theta), sin(psi)*cos(theta)*sin(phi), sin(psi)*cos(theta)*cos(phi),
                                -cos(theta), -sin(theta)*sin(phi), -sin(theta)*cos(phi);


    Matrix3f bodyTransfomation;
    bodyTransfomation <<    cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi),
                            sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi),
                            -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);

    temp( seq(0,2),0 ) = bodyTransfomationPhi*gimbalTransformation*currentOmega(0);
    temp( seq(0,2),1 ) = bodyTransfomationTheta*gimbalTransformation*currentOmega(0);
    temp( seq(0,2),2 ) = bodyTransfomation*gimbalTransformation;

    controlEffectiveness =  m/kf*temp.inverse();
    currentInput << phi, theta, currentOmega;
}


void INDIcontroller::determineControlAction( const VectorXf& error, VectorXf& output )
{
    output = currentInput + controlEffectiveness*error;

    // std::cout << controlEffectiveness << std::endl << std::endl;
    // std::cout << error << std::endl;
    // std::cout << output << std::endl;
}
