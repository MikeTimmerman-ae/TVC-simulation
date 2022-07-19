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
         * @param[in] _startTime        Start time
         * @param[in] _x                Initial value for differential states
         * @param[in] _yRef             Initial value for reference trajectory
         */
        void init( const VectorXf& _x0, const VectorXf& _yRef, double startTime );

        /** Initilizes the control law with given start values and performs consitency checks
         * 
         * @param[in] _startTime        Start time
         * @param[in] _x                Initial value for differential states
         */
        void init( const VectorXf& _x0, double startTime );



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
        VectorXf lastError;             // Last error input - used for derivative term
};