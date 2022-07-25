/**
 *	\file include/actuator.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class actuator
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
         * @brief Assign new lower limit on control signals
         * 
         * @param[in] _lowerLimits      New lower limit on control signals
         */
        void setLowerControlLimit( const VectorXf& _lowerLimits );
        
        /**
         * @brief Assign new upper limit on control signals
         * 
         * @param[in] _upperLimits      New upper limit on control signals
         */
        void setUpperControlLimit( const VectorXf& _upperLimits );

        /**
         * @brief Assign new lower limit on certain control signal
         * 
         * @param[in] _idx              Index of control signal
         * @param[in] _lowerLimit       New lower limit on control signals
         */
        void setLowerControlLimit( int idx, const float _lowerLimit );
        
        /**
         * @brief Assign new upper limit on control signal
         * 
         * @param[in] _idx              Index of control signal
         * @param[in] _upperLimit       New upper limit on control signals
         */
        void setUpperControlLimit( int idx, const float _upperLimit );


        /**
         * @brief Assign new lower rate limit on control signals
         * 
         * @param[in] _lowerRateLimits      New lower rate limit on control signals
         */
        void setLowerRateLimit( const VectorXf& _lowerRateLimits );
        
        /**
         * @brief Assign new upper rate limit on control signals
         * 
         * @param[in] _upperRateLimits      New upper rate limit on control signals
         */
        void setUpperRateLimit( const VectorXf& _upperRateLimits );

        /**
         * @brief Assign new lower rate limit on certain control signal
         * 
         * @param[in] _idx                  Index of control signal
         * @param[in] _lowerRateLimit       New lower rate limit on control signals
         */
        void setLowerRateLimit( int idx, const float _lowerRateLimit );
        
        /**
         * @brief Assign new upper rate limit on control signal
         * 
         * @param[in] _idx                  Index of control signal
         * @param[in] _upperRateLimit       New upper rate limit on control signals
         */
        void setUpperRateLimit( int idx, const float _upperRateLimit );


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

        VectorXf lastU;                     // Previes control signals

        VectorXf lowerLimits;				// Lower limits on control signals
		VectorXf upperLimits;				// Upper limits on control signals

		VectorXf lowerRateLimits;			// Lower rate limits on control signals
		VectorXf upperRateLimits;			// Upper rate limits on control signals

};