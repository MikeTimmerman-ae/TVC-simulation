/**
 *	\file include/helpers.h
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <Eigen/Dense>            // #include module
using namespace Eigen;            // using namespace


/** Load grid data from csv file
 * 
 * @param[in] FileName      File name from which to load in the data
 * @param[in] row           Number of rows to read
 * @param[in] col           Number of colums to read
 * 
 * \returns Matrix with data from file of type MatrixXf(row,col)
 * 
 */
MatrixXf loadFromFile(std::string FileName, int row, int col);



/** Upload grid data to csv file
 * 
 * @param[in] data          Data name from which to upload data to
 * @param[in] row           Number of rows of data grid
 * @param[in] col           Number of colums of data grid
 * @param[in] FileName      File name to which data should be uploaded
 * 
 */
void saveToFile(MatrixXf &data, int rows, int cols, std::string FileName);


/** Convert from euler angles to quaternion attitude representation
 * 
 * @param[in] EulerAnlges   Vector containing the euler angles: roll, pitch, yaw
 */
VectorXf toQuaternion( VectorXf& EulerAngles );


/** Convert vector from body-fixed reference frame to earth-fixed reference frame
 * 
 * @param[in] EulerAnlges       Vector containing the euler angles: roll, pitch, yaw
 * @param[in] Vector            Vector to be transformed
 * 
 * \return transformed vector in earth-fixed reference frame (NED)
 */
VectorXf BFRtoNED( VectorXf& EulerAngles, VectorXf& Vector );
