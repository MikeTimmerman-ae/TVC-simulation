/**
 *	\file include/filter.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module

class filter
{
    //
    // PUBLIC MEMBER FUNCTIONS
    //
    public:

        /** Default constructor
         */ 
        filter();

        /** Initialize filter without filtering signal
         *
         * @param[in] _nu                   Number of control inputs
         * @param[in] _samplingTime         Sampling time
         */
        filter( int _nu, float _samplingTime );

        /** Initialize low-pass filter with given cut-off frequency and sampling time
         *
         * @param[in] _omega_0              Cut-off frequency (low-pass filter)
         * @param[in] _nu                   Number of control inputs
         * @param[in] _samplingTime         Sampling time
         */
        filter( float _omega_0, int _nu, float _samplingTime );
        
        /** Copy constructor
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		filter( const filter& rhs );

		/** Destructor. 
		 */
		~filter( );



    //
    // PRIVATE MEMBER FUNCTIONS
    //
    protected:

        /** Filter signal
         * 
         * @param[in] _x            Input datasample
         * @param[in] _y            Filtered output datasample
         */ 
        void filterSignal( VectorXf _x, VectorXf& _y );


    //
    // PRIVATE DATA MEMBERS
    //
    private:
        float omega_0;
        float dt;

        int nu;

        VectorXf a_coeff;       // Output coefficients
        VectorXf b_coeff;       // Input coefficients

        MatrixXf prevX;         // Previous input samples
        MatrixXf prevY;         // Previous output samples
};