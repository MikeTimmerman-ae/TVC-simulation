/**
 *	\file include/dynamics.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class dynamics
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:
        /** 
         * @brief Default constructor
         */
        dynamics( );

        /** 
         * @brief Constructor which takes the initial state, time and sampling time
         * 
         * @param[in] _initState        System initial state
         * @param[in] _initTime         Initial time
         * @param[in] _samplingTime     Sampling time
         */
        dynamics(   VectorXf _initState,
                    float _initTime,
                    float _samplingTime  );

        /** 
         * @brief Copy constructor
         * 
         * @param[in] rhs       Right-hand side object
         */
        dynamics(  const dynamics& rhs  );

        /** 
         * @brief Destructor
         */
        virtual ~dynamics( );


        /** 
         * @brief Update system state given an input and return output
         * 
         * @param[in] _u        Control input
         * @param[in] _y        System output
         */
        void step( VectorXf& _u, VectorXf& _y );
    

    //
    // PUBLIC DATA MEMBERS
    //

        VectorXf state;
        float time;



    //
    // PRIVATE MEMBER FUNCTIONS:
    //
    private:
        /** 
         * @brief Update system state using RK45
         * 
         * @param[in] _u        Control input
         */
        void updateState( VectorXf& _u );


        /** 
         * @brief Calculate state derivatives (rhs of EOM)
         * 
         * @param[in] _t        Current time
         * @param[in] _state    Current state
         * @param[in] _u        Control input
         */
        VectorXf EOM(   float _t, VectorXf _state, const VectorXf& _u    );


        /** 
         * @brief Calculate net force acting on system in body-fixed reference frame
         * 
         * @param[in] _t        Current time
         * @param[in] _state    Current state
         * @param[in] _u        Control input
         */
        VectorXf calculateForce(   float _t, VectorXf& _state, const VectorXf& _u    );


        /**
         * @brief Calculate net moments acting on system in body-fixed reference frame
         * 
         * @param[in] _t        Current time
         * @param[in] _state    Current state
         * @param[in] _u        Control input
         */
        VectorXf calculateMoment(   float _t, VectorXf& _state, const VectorXf& _u    );



    //
	// PRIVATE DATA MEMBER:
	//
    private:
        float samplingTime=0.01;

        // System properties
        unsigned int nx = 12;
        unsigned int nu = 3;
        unsigned int ny = 18;
        unsigned int na = 6;

        // Auxiliary state
        VectorXf state_aux=VectorXf::Zero(6);       // Auxiliary output states

        // Atmospheric parameters
        float rho = 1.225;

        // Physical parameters
        float mass=1.75;

        float Ixx=0.118825; float Iyy=0.118825; float Izz=0.0735875;
        float Ixz=0; float Ixy=0; float Iyz=0;

        float rcg = 0.5;                            // Distance from cg to propellers in body-frame [m]

        float Ax = 0.01;                            // Reference area in x-direction in body-frame [m2]
        float Ay = 0.01;                            // Reference area in y-direction in body-frame [m2]
        float Az = 0.126;                           // Reference area in z-direction in body-frame [m2]

        // Aerodynamic parameters
        float rcp = 0.3;                            // Distance from cg to cp in body-frame [m]

        float Cdx = 0.7;                            // Aerodyanic coefficient in x-direction in body-frame [-]
        float Cdy = 0.7;                            // Aerodyanic coefficient in y-direction in body-frame [-]
        float Cdz = 0.45;                           // Aerodyanic coefficient in z-direction in body-frame [-]

        // Propeller parameters
        float kf1 = 0.00377; float kf2 = -0.00377;
        float km1 = 0.01; float km2 = 0.01;

        float thrustOffsetX = 0.01;                  // Offset from propeller vertical thrust in x-dir and cg [m]
        float thrustOffsetY = 0.01;                  // Offset from propeller vertical thrust in y-dir and cg [m]

        // Runge-Kutta 45 integration
        VectorXf k1=VectorXf::Zero(12);
        VectorXf k2=VectorXf::Zero(12);
        VectorXf k3=VectorXf::Zero(12);
        VectorXf k4=VectorXf::Zero(12);
        VectorXf stateDerivative=VectorXf::Zero(12);  
};


