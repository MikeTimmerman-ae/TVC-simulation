/**
 *	\file include/INDIcontroller.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module


class INDIcontroller : public controller
{
    //
    // PUBLIC MEMBER FUNCTIONS
    //
    public:
        /** Default constructor
         */
        INDIcontroller(  );

        /** Constructor which takes number of inputs and outputs as well as sampling time
         * 
         * @param[in] _nInputs          // Number of inputs
         * @param[in] _nOutputs         // Number of outputs
         * @param[in] samplingTime      // Sampling time
         */
        INDIcontroller( unsigned int _nInputs, unsigned int _nOutputs, float _samplingTime );

        /** Constructor which takes number of inputs and outputs as well as sampling time
         * 
         * @param[in] _nInputs          // Number of inputs
         * @param[in] _nOutputs         // Number of outputs
         * @param[in] samplingTime      // Sampling time
         * @param[in] _omega_0          // Cut-off frequency low-pass filter [rad/s]
         */
        INDIcontroller( unsigned int _nInputs, unsigned int _nOutputs, float _samplingTime, float _omega_0 );

        /** Copy constructor
         * 
         * @param[in] _rhs      Right-hand side object
         */
        INDIcontroller( const INDIcontroller& _rhs );

        /** Destructor
         */
        ~INDIcontroller(  );


        /** 
         * @brief Compute control effectiveness term
         * 
         * @param[in] currentAttitude       Current attitude
         * @param[in] currentGimbal         Current gimbal deflection angles
         * @param[in] currentOmega          Current propeller rotational velocity
         */
        void computeControlEffectiveness( VectorXf& currentAttitude, VectorXf& currentGimbal, VectorXf& currentOmega, VectorXf& parameters );


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
        VectorXf currentInput;              // Current input - used to determine incremental input

        Matrix3f controlEffectiveness;      // Control effectiveness term if INDI

};