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



    //
    // PROTECTED MEMBER FUNCTIONS
    //
    protected:
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
        VectorXf EulerAngles;       // Absolute orientation (Euler Angles)
        VectorXf Quaternion;        // Absolute orientation (Quaternion)
        VectorXf AccelVector;       // Acceleration vector (gravity + linear motion)
        VectorXf LinAccelVector;    // Linear acceleration vector
        VectorXf GravityVec;        // Gravity vector'
        
        VectorXf PositionVec;       // Position vector
};


class IMUsensor : sensor
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
         * @brief 
         * 
         */
        VectorXf EulerAngles();

        /**
         * @brief 
         * 
         */
        VectorXf Quaternion();

        /**
         * @brief 
         * 
         */
        VectorXf AngularVel();

        /**
         * @brief 
         * 
         */
        VectorXf Acceleration();

        /**
         * @brief 
         * 
         */
        VectorXf LinAcceleration();

        /**
         * @brief 
         * 
         */
        VectorXf GravityVec();

};
