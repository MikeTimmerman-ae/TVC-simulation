clear;
close all;
clc;

%% Generate trajectory (constant)

t0 = 0.0; tf = 30.0;
dt = 0.01;

targetX = 1.0;
targetY = 1.0;
targetZ = -2.0;

t = t0:dt:tf;
rx = targetX*ones(tf/dt+1);
ry = targetY*ones(tf/dt+1);
rz = targetZ*ones(tf/dt+1);

r = [rx ;ry ;rz];

csvwrite("trajectory.csv", r);

figure(1);
plot(t,rx,t,ry,t,rz);


%% Generate trajectory (parabolic)

t0 = 0.0; tf = 30.0;
dt = 0.01;

targetX = 1;
targetY = 1;
targetZ = -2;

t = t0:dt:tf;
rx = -targetX/(tf/2)^2*t.*(t-tf);
ry = -targetY/(tf/2)^2*t.*(t-tf);
rz = -targetZ/(tf/2)^2*t.*(t-tf);

r = [rx ;ry ;rz];

csvwrite("trajectory.csv", r);

figure(2);
plot(t,rx,t,ry,t,rz);


%% Generate trajectory (cubic)

t0 = 0.0; tf = 30.0;
dt = 0.01;

targetX = -1;
targetY = 3;
targetZ = -5;

t = t0:dt:tf;
rx = -targetX/(tf/2)^3*t.^2.*(t-tf);
ry = -targetY/(tf/2)^3*t.^2.*(t-tf);
rz = -targetZ/(tf/2)^3*t.^2.*(t-tf)-0.05;

r = [rx ;ry ;rz];

csvwrite("trajectory.csv", r);

figure(3);
plot(t,rx,t,ry,t,rz);


%% Generate trajectory (tan)

t0 = 1.0; tf = 90.0;
dt = 0.01;

targetX = 1;
targetY = 1;
targetZ = -2;

scale = 10;

t = t0:dt:tf;
rx1 = [(0:dt:(t0-dt)).*((atan((t0-tf/2)/scale)*targetX)/pi+targetX/2)/(t0-dt) atan((t-tf/2)/scale)*targetX/pi+targetX/2];
ry1 = [(0:dt:(t0-dt)).*((atan((t0-tf/2)/scale)*targetY)/pi+targetY/2)/(t0-dt) atan((t-tf/2)/scale)*targetY/pi+targetY/2];
rz1 = [(0:dt:(t0-dt)).*(atan((t0-tf/2)/scale)*targetZ/pi+targetZ/2)/(t0-dt) atan((t-tf/2)/scale)*targetZ/pi+targetZ/2];
rx2 = [atan(-(t-tf/2)/scale)*targetX/pi+targetX/2 fliplr(dt:dt:(t0-dt)).*((atan(-(tf-tf/2)/scale)*targetX)/pi+targetX/2)/(t0-dt)];
ry2 = [atan(-(t-tf/2)/scale)*targetY/pi+targetY/2 fliplr(dt:dt:(t0-dt)).*((atan(-(tf-tf/2)/scale)*targetY)/pi+targetY/2)/(t0-dt)];
rz2 = [atan(-(t-tf/2)/scale)*targetZ/pi+targetZ/2 fliplr(dt:dt:(t0-dt)).*(atan(-(tf-tf/2)/scale)*targetZ/pi+targetZ/2)/(t0-dt)];

r = [rx1 rx2;ry1 ry2;rz1 rz2];
t = 0:dt:2*tf;

%csvwrite("trajectory.csv", r);

figure(4);
plot(t,[rx1 rx2],t,[ry1 ry2],t,[rz1 rz2]);

% %% Approximate trajectory by Nth-order polynomial
% 
% % Get trajectory data
% N = 5;
% 
% % Approximate x-trajectory
% p1 = polyfit(t,r,N);
% y1 = polyval(p1, t);
% 
% % Approximate y-trajectory
% p2 = polyfit(t,r,N);
% y2 = polyval(p2, t);
% 
% % Approximate z-trajectory
% p3 = polyfit(t,r,N);
% y3 = polyval(p3, t);
% 
% %% Plot result to compare
% figure(2);
% 
% subplot(3,1,1);
% plot(t,r,t,y1);
% title("x-position");
% ylim([0, -1.5]);
% grid on
% ylabel('Displacement [m]'); 
% xlabel('Time [s]');
% legend({'Actual Trajectory', 'Approx Trajectory'},'Location','southeast')
% 
% subplot(3,1,2);
% plot(t,r,t,y2);
% title("y-position");
% ylim([0, -1.5]);
% grid on
% ylabel('Displacement [m]'); 
% xlabel('Time [s]');
% legend({'Actual Trajectory', 'Approx Trajectory'},'Location','northeast')
% 
% subplot(3,1,3);
% plot(t,r,t,y3);
% title("z-position");
% ylim([0, -1.5]);
% grid on
% ylabel('Displacement [m]'); 
% xlabel('Time [s]');
% legend({'Actual Trajectory', 'Approx Trajectory'},'Location','northeast')