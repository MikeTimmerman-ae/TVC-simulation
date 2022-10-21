/**
 *	\file include/actuator.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class actuator : public saturator
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:
        /** 
         * @brief Default constructor
         */
        actuator( );

        /**
         * @brief Constructor which takes dimensions of actuator channels
         * 
         * @param[in] _nu                   Number of control signals to be adapted
         * @param[in] _initControl          Initial control input
         * @param[in] _samplingTime         Sampling time
         */
        actuator( int _nu, VectorXf _initControl, float _samplingTime );

        /**
         * @brief Copy constructor
         * 
         * @param[in] rhs       Right-hand side object
         */
        actuator( const actuator& rhs );

        /**
         * @brief Destructor
         */
        ~actuator(  );



        /**
         * @brief Adjust control signals to actuator dynamics
         * 
         * @param[in] _u        Control signals to be adjusted
         */
        void actuate( VectorXf& _u );



    //
    // PUBLIC DATA MEMBERS
    //
    public:
        VectorXf controlRate;               // Rate of change in control signals



    //
    // PRIVATE DATA MEMBERS
    //
    private:
        int nu;                             // Number of actuator channels
        float samplingTime;                 // Sampling time
};