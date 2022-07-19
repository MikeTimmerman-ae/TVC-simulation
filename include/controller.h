/**
 *	\file include/controller.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module


class controller
{
    //
    // PUBLIC MEMBER FUNCTIONS
    //
    public:
        /** 
         * @brief Default constructor
         */
        controller(  );

        /** 
         * @brief Constructor which takes number of inputs and outputs aswell as sampling time
         * 
         * @param[in] _nInputs              // Number of inputs
         * @param[in] _nOutputs             // Number of outputs
         * @param[in] _samplingTime         // Sampling time
         */
        controller( unsigned int _nInputs, unsigned int _nOutputs, float _samplingTime );

        /** 
         * @brief Copy constructor
         * 
         * @param[in] rhs   Right-hand side object
         */
        controller ( const controller& rhs );

        /** 
         * @brief Destructor
         */
        ~controller(  );


        /** 
         * @brief Set reference trajectory using polynomial coefficients
         * 
         * @param[in] _refCoeff     Matrix containing nth order polynomial coefficients, one row per input signal
         *                          p = p_1*x^n + p_2*x^(n-1) + ... + p_n*x + p_{n+1}
         */
        void setPolynomialReference( const MatrixXf& _refCoeff );


        /** 
         * @brief Perform step of control law based on inputs
         * 
         * @param[in] currentTime   Current time
         * @param[in] _x            Current value of differential states
         * @param[in] _yRef         Current reference trajectory
         */
        void step( double currentTime, const VectorXf& _x, const VectorXf& _yRef );

        /** 
         * @brief Perform step of control law based on inputs and predefined reference
         * 
         * @param[in] currentTime   Current time
         * @param[in] _x            Current value of differential states
         */
        void step( double currentTime, const VectorXf& _x );


        /** 
         * @brief Returns control signal as determined by control law
         * 
         * @param[out] _u   Control signal
         */
        inline void getU( VectorXf& _u );



    //
    // PROTECTED MEMBER FUNCTIONS
    // 
    protected:
        /**
         * @brief Calculate current control action based on current error
         * 
         * @param[in] error     Current error
         * @param[in] ouput     Current control action
         */
        virtual void determineControlAction( const VectorXf& error, VectorXf& output )=0;



    //
    // PROTECTED DATA MEMBER
    //
    protected:
        unsigned int nInputs;           // Number of inputs
        unsigned int nOutputs;          // Number of outpus

        float samplingTime;             // Sampling time

        MatrixXf refCoeff;              // Reference using polynomial coefficients

        VectorXf u;                     // Control inputs
};


