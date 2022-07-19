/**
 *	\file include/controller.ipp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


inline void controller::getU(   VectorXf& _u    )
{
    _u(0) = u(0);
    _u(1) = u(1);
    _u(2) = u(2)+2276.856764;
}