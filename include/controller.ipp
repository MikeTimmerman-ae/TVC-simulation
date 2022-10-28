/**
 *	\file include/controller.ipp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


inline void controller::getU(   VectorXf& _u    )
{
    for (unsigned int i=0; i<nOutputs; ++i)
        _u(i) = u(i);
}