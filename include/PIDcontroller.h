/**
 *	\file include/PIDcontroller.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module


class PIDcontroller : public controller
{
    //
    // PUBLIC MEMBER FUNCTIONS
    //
    public:
        /** Default constructor
         */
        PIDcontroller(  );

        /** Constructor which takes number of inputs and outputs as well as sampling time
         * 
         * @param[in] _nInputs          // Number of inputs
         * @param[in] _nOutputs         // Number of outputs
         * @param[in] samplingTime      // Sampling time
         */
        PIDcontroller( unsigned int _nInputs, unsigned int _nOutputs, float _samplingTime );

        /** Constructor which takes number of inputs and outputs as well as sampling time
         * 
         * @param[in] _nInputs          // Number of inputs
         * @param[in] _nOutputs         // Number of outputs
         * @param[in] samplingTime      // Sampling time
         * @param[in] _omega_0              // Cut-off frequency low-pass filter [rad/s]
         */
        PIDcontroller( unsigned int _nInputs, unsigned int _nOutputs, float _samplingTime, float _omega_0 );

        /** Copy constructor
         * 
         * @param[in] _rhs      Right-hand side object
         */
        PIDcontroller( const PIDcontroller& _rhs );

        /** Destructor
         */
        ~PIDcontroller(  );


        /** 
         * @brief Assign proportional gains to input components
         * 
         * @param[in] _pGains     New proportional weights
         */
        void setProportionalGains( const VectorXf& _pGains );

        /** 
         * @brief Assign integral gains to input components
         * 
         * @param[in] _pGains     New integral weights
         */
        void setIntegralGains( const VectorXf& _iGains );
        
        /** 
         * @brief Assign derivative gains to input components
         * 
         * @param[in] _pGains     New integral weights
         */
        void setDerivativeGains( const VectorXf& _dGains );


        /** 
         * @brief Initilizes the control law with given start values and performs consitency checks
         * 
         * @param[in] _x0               Initial value for differential states
         * @param[in] _initU            Initial output value of controller
         * @param[in] _yRef             Initial value for reference trajectory
         * @param[in] _startTime        Start time
         */
        void init( const VectorXf& _x0, const VectorXf& _initU, const VectorXf& _yRef, double startTime );

        /** Initilizes the control law with given start values and performs consitency checks
         * 
         * @param[in] _x0               Initial value for differential states
         * @param[in] _initU            Initial output value of controller
         * @param[in] _startTime        Start time
         */
        void init( const VectorXf& _x0, const VectorXf& _initU, double startTime );



    //
    // PRIVATE MEMBER FUNCTIONS
    //
    private:
        /**
         * @brief Determine current control action based on current error
         * 
         * @param[in] error     Current error
         * @param[in] output    Current control action
         */
        void determineControlAction( const VectorXf& error, VectorXf& output ) override;

    
    //
    // PRIVATE DATA MEMBERS
    //
    private:
        VectorXf pGains;                // Proportional gains for all input components
        VectorXf iGains;                // Integral gains for all input components
        VectorXf dGains;                // Derivative gains for all input components

        VectorXf iValue;                // Integrated value for all input components - used for integral term
        VectorXf dValue;                // PID derivative term
        VectorXf pValue;                // PID proportional term
        VectorXf lastError;             // Last error input - used for derivative term

        double dOmega = 1;              // Cut-off freq. for low pas filter on derivative term [rad/s]
        double Kaw = 0.0;               // Anti wind-up gain
};