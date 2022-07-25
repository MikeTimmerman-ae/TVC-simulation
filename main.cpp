#include "header.h"


int main(int argc, char const *argv[])
{
    /* Set-up dynamics model */

    VectorXf initState = VectorXf::Zero(12);
    initState(0)=0.0872665;
    initState(1)=0.0872665;
    initState(6)=50;
    initState(7)=50;
    initState(8)=-50.5;
    float initTime = 0.0; float finalTime = 25.0;
    float samplingTime = 0.01;

    dynamics Drone( initState, initTime, samplingTime );
    

    /* Simulation loop */

    // Data points
    int Nsim = (int) (finalTime-initTime)/samplingTime;

    // Input, output signals and reference vector
    VectorXf u(3);
    VectorXf u_serv(2); u_serv << 0.0, 0.0;
    VectorXf u_prop(1); u_prop << 2276.856764;

    VectorXf ySystem(18);
    VectorXf y(3);
    VectorXf y_attitude(3); y_attitude << initState( seq( 0,2 ) );
    VectorXf y_position(3); y_position << initState( seq( 6,8 ) );

    VectorXf ref(3); ref << -0.0872665, -0.0872665, -50;
    
    // Data matrices
    MatrixXf X(12,Nsim+1); X(seq(0,11), 0)=initState;
    MatrixXf U(6,Nsim+1); U(seq(0,2), 0)=u; U(seq(3,5), 0) = VectorXf::Zero(3);
    MatrixXf T(1,Nsim+1); T(0,0) = initTime;

    // Initialize actuators
    actuator Servos( 2,u_serv, samplingTime );
    
    Servos.setLowerControlLimit( -1,-0.261799 );        // Max gimbal angle
    Servos.setUpperControlLimit( -1, 0.261799 );        // deflections: +-15 deg

    Servos.setLowerRateLimit( -1,-0.261799 );           // Max gimbal deflection
    Servos.setUpperRateLimit( -1, 0.261799 );           // rateL +-15 deg/s

    actuator Propellers( 1,u_prop,samplingTime );

    Propellers.setLowerControlLimit( 0,-3952.12 );      // Max propeller rotational
    Propellers.setUpperControlLimit( 0, 3952.12 );      // acceleration: +-3952.12 rad/s

    Propellers.setLowerRateLimit( 0,-100.0 );         // Max propeller rotational
    Propellers.setUpperRateLimit( 0, 100.0 );         // acceleration: +-1976.06 rad/s2

    // Initialize sensors
    IMUsensor BNO055;

    // Initialize controller
    PIDcontroller PID( 3,3,samplingTime );

    VectorXf pGains( 3 );
	pGains(0) = -0.5;
	pGains(1) = -0.6;
	pGains(2) = -110.0;

	VectorXf iGains( 3 );
	iGains(0) = -0.1;
	iGains(1) = -0.1;
	iGains(2) =  0.0;

	VectorXf dGains( 3 );
	dGains(0) = -0.15;
	dGains(1) = -0.2;
	dGains(2) = -50.0;

    PID.setProportionalGains( pGains );
    PID.setIntegralGains( iGains );
    PID.setDerivativeGains( dGains );

    PID.init( y,initTime );

    // Run closed-loop simulation
    for (int i=0; i<Nsim; ++i)
    {
        // Drone has not hit ground
        if (Drone.state[8] <= 0.0)
        {
            // Control logic
            y << y_attitude( seq( 0,1 ) ), y_position( seq( 2,2 ) );

            PID.step( Drone.time,y,ref );
            PID.getU( u );

            // Actuator
            u_serv = u( seq( 0,1 ) ); u_prop = u( seq( 2,2 ) );
            
            Servos.actuate( u_serv );
            Propellers.actuate( u_prop );
            
            u << u_serv, u_prop;

            // System
            Drone.step( u,ySystem );

            // Sensor
            BNO055.processOutput( ySystem );
            
            BNO055.EulerAngles( y_attitude );
            BNO055.PositionVec( y_position );   
        }

        // Save data
        X(seq(0, 11), i+1) = Drone.state;
        U(seq(0, 2), i+1) = u;
        U(seq(3,4), i+1) = Servos.controlRate;
        U(seq(5,5), i+1) = Propellers.controlRate;
        T(0, i+1) = (i+1)*samplingTime;

        // Print status
        if ((i+1)%50 == 0)
        {
            std::cout << "Atitude: " << Drone.state[8] << " Time: " << (i+1)*samplingTime << std::endl;
            std::cout << "Closed-Loop simulation: iteration " << i+1 << " out of " << Nsim << std::endl;
        }

    }

    // Export data
    saveToFile(X, X.rows(), X.cols(), "../data/state.csv");
    saveToFile(U, U.rows(), U.cols(), "../data/input.csv");
    saveToFile(T, T.rows(), T.cols(), "../data/time.csv");

    return 0;
}