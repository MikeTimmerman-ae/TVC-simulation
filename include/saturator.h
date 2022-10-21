/**
 *	\file include/saturator.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class saturator
{
    //
    // PUBLIC MEMBER FUNCTIONS
    //
    public:

        /** Default constructor
         */ 
        saturator();

        /** Constructor which takes dimensions of signal to be saturated
         *
         * @param[in] _nU                   Number of control signals to be saturated
         * @param[in] _samplingTime         Sampling time
         */
        saturator( unsigned int _nU, float _samplingTime );
        
        /** Copy constructor
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		saturator( const saturator& rhs );

		/** Destructor. 
		 */
		~saturator( );


        /** Assigns new lower limit on control signals
         * 
         * @param[in] _lowerLimit       New lower limit on control signal
         */
        void setLowerControlLimit( const VectorXf& _lowerLimit );

        /** Assigns new lower limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _lowerLimit       New lower limit
         */
        void setLowerControlLimit( int idx, float _lowerLimit );

        /** Assigns new upper limit on control signals
         * 
         * @param[in] _upperLimit       New upper limit on control signal
         */
        void setUpperControlLimit( const VectorXf& _upperLimit );

        /** Assigns new upper limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _upperLimit       New upper limit
         */
        void setUpperControlLimit( int idx, float _upperLimit );


        /** Assigns new lower rate limit on control signals
         * 
         * @param[in] _lowerRateLimit   New lower rate limit on control signal
         */
        void setLowerRateLimit( const VectorXf& _lowerRateLimit );

        /** Assigns new lower rate limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _lowerRateLimit   New lower rate limit
         */
        void setLowerRateLimit( int idx, float _lowerRateLimit );

        /** Assigns new upper rate limit on control signals
         * 
         * @param[in] _upperRateLimit   New upper limit on control signal
         */
        void setUpperRateLimit( const VectorXf& _upperRateLimit );

        /** Assigns new upper rate limit on given component of control signal
         * 
         * @param[in] idx               Index of control signal component
         * @param[in] _upperRateLimit   New upper limit
         */
        void setUpperRateLimit( int idx, float _upperRateLimit );


    //
    // PROTECTED MEMBER FUNCTIONS
    //
    protected:

        /** Saturate the control signal
         * 
         * @param[in] _u                Control signal
         */
        void saturate( VectorXf& _u );



    //
    // PROTECTED DATA MEMBERS
    //

        int nU;                            // Number of control variables
        float samplingTime;                         // Controller time step

        VectorXf lastU;				                // Previous control

		VectorXf lowerLimits;				// Lower limits on control signals
		VectorXf upperLimits;				// Upper limits on control signals

		VectorXf lowerRateLimits;			// Lower rate limits on control signals
		VectorXf upperRateLimits;			// Upper rate limits on control signals

};
