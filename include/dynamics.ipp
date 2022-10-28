/**
 *	\file include/dynamics.ipp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


inline float dynamics::getdt( )
{
    return samplingTime;
}