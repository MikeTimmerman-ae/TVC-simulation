/**
 *	\file header.hpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#pragma once

#include <iostream>                 // #include directives
#include <fstream>
#include <math.h>
#include <string>

#include "include/saturator.h"       // include src code
#include "include/filter.h"
#include "include/estimator.h"
#include "include/controller.h"
#include "include/controller.ipp"
#include "include/dynamics.h"
#include "include/dynamics.ipp"
#include "include/helpers.h"
#include "include/PIDcontroller.h"
#include "include/INDIcontroller.h"
#include "include/actuator.h"
#include "include/sensor.h"

#include "scripts/PIDattitudeControl.h"     // include scripts


#include <Eigen/Dense>
#include <Eigen/SparseCore>

using namespace Eigen;