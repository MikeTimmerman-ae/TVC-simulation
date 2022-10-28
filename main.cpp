#include "header.h"


int main(int argc, char const *argv[])
{
    /* Set-up dynamics model */

    VectorXf initState = VectorXf::Zero(12);
    initState(8) = -0.05;
    float initTime = 0.0; float finalTime = 35.0;
    float samplingTime = 0.01;

    dynamics Drone( initState, initTime, samplingTime );
    
    
    // Set reference using polynomial coefficients
    MatrixXf ref = loadFromFile("../guidance/trajectory.csv",3,(finalTime-initTime)/samplingTime);

    // Simulate rocket launch
    INDIpositionControl( Drone,ref,finalTime );

    return 0;
}