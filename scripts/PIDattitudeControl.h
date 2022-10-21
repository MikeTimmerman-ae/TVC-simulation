/**
 *	\file include/PIDattitudeControl.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>              // #include module
using namespace Eigen;              // using namespace of module


/** 
 * @brief Perform attitude control on the drone (pitch and roll control)
 * 
 * @param[in] DroneDynamics     Object containing the drone dynamics
 * @param[in] RefAttitude       Reference drone attitude
 * @param[in] finalTime         Simulation time
 */
void PIDattitudeControl( dynamics& dynamics, VectorXf& Reference, float finalTime );


/**
 * @brief Perform position control on the drone
 * 
 * @param[in] DroneDynamics     Object containing the drone dynamics
 * @param[in] RefPosition       Reference drone position
 * @param[in] finalTime         Simulation time
 */
void INDIpositionControl( dynamics& Drone, MatrixXf& Reference, float finalTime );
