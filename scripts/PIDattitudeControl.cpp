/**
 *	\file src/PIDattitudeControl.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header


void PIDattitudeControl( dynamics& Drone, VectorXf& Reference, float finalTime )
{
    /* Simulation loop */

    // Data points
    float samplingTime = Drone.getdt( );
    float initTime = Drone.time;
    int Nsim = (int) (finalTime-initTime)/samplingTime;

    // Input, output signals and reference vector
    VectorXf u = VectorXf::Zero(3);
    VectorXf u_serv(2); u_serv << 0.0, 0.0;
    VectorXf u_prop(1); u_prop << 2276.856764;
    VectorXf s_prop(1); s_prop << 2276.856764;

    VectorXf ySystem(18);
    VectorXf y_attitude(3); y_attitude << Drone.state( seq( 0,2 ) );
    VectorXf y_omega(3); y_omega << Drone.state( seq( 3,5 ) );
    VectorXf y_position(3); y_position << Drone.state( seq( 6,8 ) );
    VectorXf y(3); y << y_attitude( seq(0,1) ), y_position( seq(2,2) );

    VectorXf ref(3); ref << -0.0872665, -0.0872665, -50;
    
    // Data matrices
    MatrixXf X(12,Nsim+1); X( seq(0,11),0 )=Drone.state;
    MatrixXf U(6,Nsim+1); U( seq(0,1),0 )=u_serv; U( seq(2,2),0 )=u_prop; U(seq(3,5), 0) = VectorXf::Zero(3);
    MatrixXf T(1,Nsim+1); T( 0,0 ) = initTime;
    MatrixXf R(3,Nsim+1); R( seq(0,2),0 ) = ref;
    
    // Initialize actuators
    actuator Servos( 2,u_serv,samplingTime );
    
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
    PIDcontroller PIDinner( 2,2,samplingTime );

    VectorXf pGains( 3 );
	pGains(0) = 1.1;
	pGains(1) = 0.9;
	pGains(2) = -110.0;

	VectorXf iGains( 3 );
	iGains(0) = 0.0;
	iGains(1) = 0.0;
	iGains(2) =  0.0;

	VectorXf dGains( 3 );
	dGains(0) = 0.5;
	dGains(1) = 0.7;
	dGains(2) = -50.0;

    PID.setProportionalGains( pGains );
    PID.setIntegralGains( iGains );
    PID.setDerivativeGains( dGains );

    VectorXf pGainsInner( 2 );
	pGainsInner(0) = -1.0;
	pGainsInner(1) = -1.0;

	VectorXf iGainsInner( 2 );
	iGainsInner(0) = -0.3;
	iGainsInner(1) = -0.3;

	VectorXf dGainsInner( 2 );
	dGainsInner(0) = 0.0;
	dGainsInner(1) = 0.0;

    PIDinner.setProportionalGains( pGainsInner );
    PIDinner.setIntegralGains( iGainsInner );
    PIDinner.setDerivativeGains( dGainsInner );

    PID.init( y,ref,y_omega,initTime );
    PIDinner.init( y_omega(seq(0,1)),u,initTime );

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

            PIDinner.step( Drone.time,y_omega( seq(0,1) ),u( seq(0,1) ) );
            PIDinner.getU( u_serv );

            // Actuator
            u_prop = u( seq( 2,2 ) ) + s_prop;
            
            Servos.actuate( u_serv );
            Propellers.actuate( u_prop );
            
            u << u_serv, u_prop;

            // System
            Drone.step( u,ySystem );

            // Sensor
            BNO055.processOutput( ySystem );
            
            BNO055.EulerAngles( y_attitude );
            BNO055.PositionVec( y_position );   
            BNO055.AngularVel( y_omega );
        }

        // Save data
        X(seq(0, 11), i+1) = Drone.state;
        R(seq(0,2), i+1) = ref;
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
    saveToFile(R, R.rows(), R.cols(), "../data/ref.csv");
    saveToFile(U, U.rows(), U.cols(), "../data/input.csv");
    saveToFile(T, T.rows(), T.cols(), "../data/time.csv");
}


void INDIpositionControl( dynamics& Drone, MatrixXf& Reference, float finalTime )
{
    /* Simulation loop */

    // Data points
    float samplingTime = Drone.getdt( );
    float initTime = Drone.time;
    int Nsim = (int) (finalTime-initTime)/samplingTime;

    // Parameters, input, output and reference signals
    VectorXf p(2); p << 1.75, -2*0.00377;                                // Mass and twice the force constant of one propeller

    VectorXf u = VectorXf::Zero(3);
    VectorXf u_serv(2); u_serv << 0.0, 0.0;
    VectorXf u_prop(1); u_prop << 2276.856764;

    VectorXf ySystem(18);
    VectorXf y_position(3); y_position << Drone.state( seq( 6,8 ) );
    VectorXf y_vel = VectorXf::Zero(3);
    VectorXf y_acc = VectorXf::Zero(3);
    VectorXf y_attitude(3); y_attitude << Drone.state( seq( 0,2 ) );
    VectorXf y_omega(3); y_omega << Drone.state( seq( 3,5 ) );

    VectorXf ref_pos = VectorXf::Zero(3); // ref_pos << 1.0, 1.0, -1.0;
    VectorXf ref_vel = VectorXf::Zero(3);
    VectorXf ref_acc = VectorXf::Zero(3); 
    VectorXf ref_attitude = VectorXf::Zero(2);
    VectorXf ref_omega = VectorXf::Zero(2);

    // Data matrices
    MatrixXf X(18,Nsim+1); X( seq(0,11),0 )=Drone.state;
    MatrixXf U(6,Nsim+1); U( seq(0,1),0 )=u_serv; U( seq(2,2),0 )=u_prop; U(seq(3,5), 0) = VectorXf::Zero(3);
    MatrixXf T(1,Nsim+1); T( 0,0 ) = initTime;
    MatrixXf R(13,Nsim+1); R( seq(0,1),0 ) = ref_omega; R( seq(2,3),0 ) = ref_attitude; R( seq(4,6),0 ) = ref_acc; R( seq(7,9),0 ) = ref_vel; R( seq(10,12),0 ) = ref_pos; 
    
    // Define controller
    PIDcontroller PIDpos( 3,3,samplingTime, 5 );
    PIDcontroller PIDvel( 3,3,samplingTime, 10 );
    INDIcontroller INDI( 3,3,samplingTime );
    PIDcontroller PID( 2,2,samplingTime, 10 );
    PIDcontroller PIDinner( 2,2,samplingTime, 10 );

    VectorXf pGainsInner( 2 );
	pGainsInner(0) = -1.0;
	pGainsInner(1) = -1.0;

	VectorXf iGainsInner( 2 );
	iGainsInner(0) = -0.0;
	iGainsInner(1) = -0.0;

	VectorXf dGainsInner( 2 );
	dGainsInner(0) = -0.0;
	dGainsInner(1) = -0.0;

    PIDinner.setProportionalGains( pGainsInner );
    PIDinner.setIntegralGains( iGainsInner );
    PIDinner.setDerivativeGains( dGainsInner );

    VectorXf pGains( 2 );
	pGains(0) = 3.0;
	pGains(1) = 3.0;

	VectorXf iGains( 2 );
	iGains(0) = 10.0;
	iGains(1) = 10.0;

	VectorXf dGains( 2 );
	dGains(0) = 0.0;
	dGains(1) = 0.0;

    PID.setProportionalGains( pGains );
    PID.setIntegralGains( iGains );
    PID.setDerivativeGains( dGains );

    // PID.setLowerControlLimit( -1,-0.261799 );        // Max rotational velocity: +-15 deg/s
    // PID.setUpperControlLimit( -1, 0.261799 );

    // PID.setLowerRateLimit( -1,-0.0873 );       // Max rotational acceleration: +-5 deg/s2
    // PID.setUpperRateLimit( -1, 0.0873 );

    VectorXf pGainsVel( 3 );
	pGainsVel(0) = 2.0;
	pGainsVel(1) = 2.0;
	pGainsVel(2) = 2.0;

    VectorXf dGainsVel( 3 );
	dGainsVel(0) = 0.0;
	dGainsVel(1) = 0.0;
	dGainsVel(2) = 0.0;

    VectorXf iGainsVel( 3 );
	iGainsVel(0) = 10.0;
	iGainsVel(1) = 10.0;
	iGainsVel(2) = 10.0;

    PIDvel.setProportionalGains( pGainsVel );
    PIDvel.setDerivativeGains( dGainsVel );
    PIDvel.setIntegralGains( iGainsVel );

    // PIDvel.setLowerControlLimit( -1,-0.3 );        // Max acceleration: +-0.3 m/s
    // PIDvel.setUpperControlLimit( -1, 0.3 );

    // PIDvel.setLowerRateLimit( -1,-0.05 );          // Max time derivative of acceleration: +-0.05 m/s2
    // PIDvel.setUpperRateLimit( -1, 0.05 );

    VectorXf pGainsPos( 3 );
	pGainsPos(0) = 0.7;
	pGainsPos(1) = 0.7;
	pGainsPos(2) = 1.0;

    VectorXf dGainsPos( 3 );
	dGainsPos(0) = 0.0;
	dGainsPos(1) = 0.0;
	dGainsPos(2) = 0.0;

    VectorXf iGainsPos( 3 );
	iGainsPos(0) = 2.0;
	iGainsPos(1) = 2.0;
	iGainsPos(2) = 2.0;

    PIDpos.setProportionalGains( pGainsPos );
    PIDpos.setDerivativeGains( dGainsPos );
    PIDpos.setIntegralGains( iGainsPos );

    // PIDpos.setLowerControlLimit( -1,-1.0 );        // Max velocity: +-5 m/s
    // PIDpos.setUpperControlLimit( -1, 1.0);

    // PIDpos.setLowerRateLimit( -1,-1.0 );           // Max acceleration: +-0.1 m/s2
    // PIDpos.setUpperRateLimit( -1, 1.0 );

    // Define and initialize actuators
    actuator Servos( 2,u_serv,samplingTime );
    
    // Servos.setLowerControlLimit( -1,-0.261799 );        // Max gimbal angle
    // Servos.setUpperControlLimit( -1, 0.261799 );        // deflections: +-15 deg

    // Servos.setLowerRateLimit( -1,-0.261799 );           // Max gimbal deflection
    // Servos.setUpperRateLimit( -1, 0.261799 );           // rateL +-15 deg/s

    actuator Propellers( 1,u_prop,samplingTime );

    // Propellers.setLowerControlLimit( 0,-3952.12 );      // Max propeller rotational
    // Propellers.setUpperControlLimit( 0, 3952.12 );      // acceleration: +-3952.12 rad/s

    // Propellers.setLowerRateLimit( 0,-100.0 );         // Max propeller rotational
    // Propellers.setUpperRateLimit( 0, 100.0 );         // acceleration: +-1976.06 rad/s2

    actuator Attitude( 2,y_attitude(seq(0,1)),samplingTime );
    
    // Attitude.setLowerControlLimit( -1,-2.61799 );        // Max attitude angle: +-15 deg
    // Attitude.setUpperControlLimit( -1, 2.61799 );

    // Attitude.setLowerRateLimit( -1,-0.05 );       // Max rotational velocity: +-0.15 deg/s
    // Attitude.setUpperRateLimit( -1, 0.05 );        

    // Define sensors
    IMUsensor BNO055;


    // Initialize controllers
    PIDpos.init( y_position,y_vel,initTime );
    PIDvel.init( y_vel,y_acc,ref_vel,initTime );
    PID.init( y_attitude(seq(0,1)),y_omega,ref_attitude,initTime );
    PIDinner.init( y_omega(seq(0,1)),u,ref_omega,initTime );

    // Run closed-loop simulation
    for (int i=0; i<Nsim; ++i)
    {
        // Drone has not hit ground
        if (Drone.state[8] <= 0.0)
        {
            // Set position reference
            ref_pos = Reference.col(i);

            // Control logic
            PIDpos.step( Drone.time,y_position,ref_pos );
            PIDpos.getU( ref_vel );

            PIDvel.step( Drone.time,y_vel,ref_vel );
            PIDvel.getU( ref_acc );

            y_acc = BFRtoNED( y_attitude,y_acc );
            INDI.computeControlEffectiveness( y_attitude,u_serv,u_prop,p );
            INDI.step( Drone.time, y_acc, ref_acc );
            INDI.getU( u );

            ref_attitude << u( seq(0,1) );
            Attitude.actuate( ref_attitude );
            u_prop = u( seq(2,2) );

            PID.step( Drone.time,y_attitude( seq(0,1) ),ref_attitude );
            PID.getU( ref_omega );

            PIDinner.step( Drone.time,y_omega( seq(0,1) ),ref_omega );
            PIDinner.getU( u_serv );

            // Actuator            
            Servos.actuate( u_serv );
            Propellers.actuate( u_prop );

            u << u_serv, u_prop;

            // System
            Drone.step( u,ySystem );

            // Sensor
            BNO055.processOutput( ySystem );
            
            BNO055.AngularVel( y_omega );
            BNO055.EulerAngles( y_attitude );
            BNO055.Acceleration( y_acc );   
            BNO055.PositionVec( y_position );
            y_vel = Drone.earthVel;
        }

        // Save data
        X(seq(0, 11), i+1) = Drone.state;
        X(seq(12, 14), i+1) = y_vel;
        X(seq(15, 17), i+1) = y_acc;

        R(seq(0,1), i+1) = ref_omega;
        R(seq(2,3), i+1) = ref_attitude;
        R(seq(4,6), i+1) = ref_acc;
        R(seq(7,9), i+1) = ref_vel;
        R(seq(10,12), i+1) = PIDpos.yRef;

        U(seq(0, 2), i+1) = u;
        U(seq(3,4), i+1) = Servos.controlRate;
        U(seq(5,5), i+1) = Propellers.controlRate;

        T(0, i+1) = (i+1)*samplingTime;

        // Print status
        if ((i+1)%50 == 0)
        {
            std::cout << "Altitude: " << Drone.state[8] << " Time: " << (i+1)*samplingTime << std::endl;
            std::cout << "Closed-Loop simulation: iteration " << i+1 << " out of " << Nsim << std::endl;
        }

    }

    // Export data
    saveToFile(X, X.rows(), X.cols(), "../data/state.csv");
    saveToFile(R, R.rows(), R.cols(), "../data/ref.csv");
    saveToFile(U, U.rows(), U.cols(), "../data/input.csv");
    saveToFile(T, T.rows(), T.cols(), "../data/time.csv");
}