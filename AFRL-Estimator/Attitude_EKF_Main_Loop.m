clc
clear all
close all

%%Attitude Estimator EKF
%Estimates attitude recursively once started
%PX4 default firmware

%%INITIALIZATION

%State Vector (Not sure what the states are...)
%They are commented as (ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz)
%in the documentation
%UPDATE: ax, ay, and az are roll, pitch, and yaw speed
%UPDATE: mx, my, and mz are roll, pitch, and yaw acceleration
%UPDATE: wox,woy and woz are the components of the earth z vector 
%(in body frame?)
%UPDATE: wx, wy, and wz are components of the magnetic vector
x_aposteriori_k    =  zeros(12,1);
x_aposteriori      =  zeros(12,1);
%State Covariance Matrix
P_aposteriori_k    =  eye(12)*100.; %Initialized diag with big values
P_aposteriori      =  zeros(12,12);
%Measurement vector (Not sure what the measurements are...6 is g?)
%zk 1,2, and 3 are gyro readings
%zk 4,5, and 6 are accelerometer readings
%zk 7,8, and 9 are magnetometer readings
z_k    =  [0,0,0,0,0,9.81,0.2,-0.2,0.2];
%Output Euler Angles
euler       = [0,0,0];
%Rotation Matrix from body to inertial?
Rot_matrix  = eye(3);
%Current Velocity
vel = zeros(3,1);
%Previous Velocity
vel_prev = zeros(3,1);
%Actual Acceleration (from GPS)
acc = zeros(3,1);
%Rotation Matrix from body to inertial?
R = eye(3);
%Gyro offsets
gyro_offsets= [0,0,0];
%Rotation matrix for magnetic declination (?)
R_decl = eye(3);

%% MAIN LOOP

tf = 10;
dt = 0.05;
t = 0:dt:tf;

%Initialize with good values
x_aposteriori_k(1:3) = z_k(1:3);
x_aposteriori_k(4:6) = [0,0,0];
x_aposteriori_k(7:12)= z_k(4:9);

for i = 1 : length(t)-1
    %Take gyro reading
    gyro_rad_s = eye(3,1) + randn(3,1)*0.1;
    %Update gyro measurements
    z_k(1) = gyro_rad_s(1) - gyro_offsets(1);
    z_k(2) = gyro_rad_s(2) - gyro_offsets(2);
    z_k(3) = gyro_rad_s(3) - gyro_offsets(3);
    %Take velocity reading
    vel = eye(3,1)+ randn(3,1)*0.1;
    %Take acceleromter reading
    accelerometer_m_s2 = eye(3,1);
    %Update accelerometer measurements
    acc = R' * (vel - vel_prev)/dt;
    vel_prev = vel;
    z_k(4) = accelerometer_m_s2(1) - acc(1);
    z_k(5) = accelerometer_m_s2(2) - acc(2);
    z_k(6) = accelerometer_m_s2(3) - acc(3);
    %Take magnetometer measurements
    magnetometer_ga = [1;0;0]+ randn(3,1)*0.1;
    %Update magnetometer measurements
    z_k(7) = magnetometer_ga(1);
    z_k(8) = magnetometer_ga(2);
    z_k(9) = magnetometer_ga(3);
    %Run Filter
    [euler(i+1,:),Rot_matrix,x_aposteriori_k,P_aposteriori_k] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k);
    R = Rot_matrix;
     
end

figure
plot(t,euler(:,:))


