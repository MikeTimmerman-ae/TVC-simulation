/**
 *	\file include/sensor.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class sensor
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:
        /** 
         * @brief Default constructor
         */
        sensor( );


        /**
         * @brief Process system output vector
         * 
         */
        void processOutput( VectorXf& _y );



    //
    // PROTECTED DATA MEMBERS
    //
    protected:
        VectorXf omega;             // Angular velocity vector
        VectorXf EulerAnglesVector; // Absolute orientation (Euler Angles)
        VectorXf QuaternionVector;  // Absolute orientation (Quaternion)
        VectorXf AccelVector;       // Acceleration vector (gravity + linear motion)
        VectorXf LinAccelVector;    // Linear acceleration vector
        VectorXf GravityVector;     // Gravity vector'
        
        VectorXf PositionVector;    // Position vector
};


class IMUsensor : public sensor
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:
        /** 
         * @brief Default constructor
         */
        IMUsensor( );
        

        /** 
         * @brief Obtain Euler angle representation of system attitude
         * 
         * @param[in] _yout         Output vector in which variables are stored
         */
        void EulerAngles( VectorXf& _yout );

        /**
         * @brief Obtain quaternion representation of system attitude
         * 
         * @param[in] _yout         Output vector in which variables are stored
         */
        void Quaternion( VectorXf& _yout );

        /**
         * @brief Obtain angular velocity vector of system
         * 
         * @param[in] _yout         Output vector in which variables are stored
         */
        void AngularVel( VectorXf& _yout );

        /**
         * @brief Obtain acceleration vector of system
         * 
         * @param[in] _yout         Output vector in which variables are stored
         */
        void Acceleration( VectorXf& _yout );

        /**
         * @brief Obtain linear acceleration vector of system (without gravity)
         * 
         * @param[in] _yout         Output vector in which variables are stored
         */
        void LinAcceleration( VectorXf& _yout );

        /**
         * @brief Obtain gravitational acceleration vector of system (without movement)
         * 
         * @param[in] _yout         Output vector in which variables are stored
         */
        void GravityVec( VectorXf& _yout );

        /**
         * @brief Obtain system's positional vector
         * 
         * @param[in] _yout         Output vector in which variables are stored
         */
        void PositionVec( VectorXf& _yout );

};
