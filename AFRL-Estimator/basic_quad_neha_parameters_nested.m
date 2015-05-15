  %function error = basic_quad()
%SUMMER PROJECT VERSION
clear all
clc

% Define simulation duration (tf = n/100)
n = 1000;

% Define copter properties
mass = .36;%*(0.9 + rand()/5); %kg (with up to plus/minus 10% error)
mu = .05;%*(0.9 + rand()/5); %drag coefficient (up to plus/minus 10% error)

% Define Initial state
pos = [0;0;-5];
% or = [rand()-0.5;rand()-0.5;(rand()-0.5)]/1000000;
or = [0 0 pi/4]';
vel = [0;0;0];
ang = [0;0;0];
State = [pos;or;vel;ang];

% Define Initial forces and moments
force = [0;0;0];
moment = [0;0;0];

% Define initial moment of inertia matrix
Ib = [0.0241 0 0;0 0.0232 0;0 0 .0451]*10;

inner_loop_gains = [5,0,-3,5,0,-3,5,0,-3,-30,0,0];
% Instantiate object
copter = eulerRK4(State,Ib,mass,force,moment);
% Instantiate control system
cntrl = control(copter.m,copter.Ib,mu,inner_loop_gains);
cntrl.geometry();

% Initialize simulated INS and Flow sensors
accelerometer = zeros(3,n);
gyro = zeros(3,n);
flow = zeros(2,n);
altimeter = zeros(n);

% Characterize the sensor noise/error
std_dev_acc = 0.5;
% std_dev_acc = 0.1;
mean_err_acc = 0;
acc_drift_rate = [0;0;0];
std_dev_gyro = 0.02;
% std_dev_gyro = 0.015;
mean_err_gyro = 0;
gyro_drift_rate = [0;0;0];
std_dev_flow = .1;
mean_err_flow = 0;
flow_drift_rate = [0;0];
std_dev_altimeter = .05;
mean_err_altimeter = 0;
altimeter_drift_rate = 0;

% Instantiate estimator
x0 = [State(7:9); State(4:5); 0; State(3)];
ekf = estimator(mass, mu, std_dev_acc, std_dev_gyro, std_dev_altimeter, std_dev_flow, x0);

% Initialize some debugging variables for efficiency
time = 0.01:0.01:n/100;
tmp = zeros(3,n);
thrust = zeros(4,n);
position = zeros(4,n);
orientation = zeros(3,n);
vert_vel = zeros(n);
body_vel = zeros(3,n);
ang_vel = zeros(3,n);
world_vel = zeros(4,n);
xhat = zeros(7,n);
alt_time = [];
alt_log = [];
flow_time = [];
flow_log = zeros(2,0);

% Initialize PX4 EKF Variables
x_aposteriori_k    =  zeros(12,1);
x_aposteriori      =  zeros(12,1);
P_aposteriori_k    =  eye(12)*100.;
P_aposteriori      =  zeros(12,12);
z_k    =  [0,0,0,0,0,0,0,0,0];

euler       = [0,0,0];
Rot_matrix  = eye(3);
vel = zeros(3,1);
vel_prev = zeros(3,1);

% Start timer on simulation, for evaluation purposes
tic
set_points = 0;
% Velocity has +/- 1m/s error in u,v and +/- 0.25 m/s error in w
cnst_err = zeros(12,1);%[0;0;0;0;0;0;2*rand()-1;2*rand()-1;.5*rand-.25;0;0;0];
path = zeros(4,n);
for i =1:n/2
    path(1,i) = .1;
    path(1,i+n/2) = 0;
    path(2,i) = .1;
    path(2,i+n/2) = 0;
    %path(3,i) = 5;
    %path(3,i+n/2) = 0;
    %path(4,i) = 3;
    %path(4,i+n/2) = -10;
end
for i = 1:n
    inner_loop_set_points = cntrl.outer_loop(copter.State+cnst_err,path(:,i));
    %inner_loop_set_points = [0;0;0;0];
    thrust(:,i) = cntrl.inner_loop(inner_loop_set_points,copter.State+cnst_err);
    
    % Apply the commanded thrusts
    result = cntrl.A_actual*thrust(:,i);
    copter.Moment = result(1:3);
    copter.Force = [0;0;0];
    copter.Force(3) = result(4);
    
    % Correct for drag
    %%% Not 100% sure this is being done correctly
    copter.Force = copter.Force - (mu)*[copter.State(7:8);0];

    % Setting plotting/debugging variables
    tmp(:,i) = copter.State(4:6);
    position(1,i) = copter.State(1);
    position(2,i) = copter.State(2);
    position(3,i) = -copter.State(3);
    orientation(:,i) = copter.State(4:6);
    body_vel(:,i) = copter.State(7:9);
    ang_vel(:,i) = copter.State(10:12);
    vert_vel(i) = -copter.State(9);
    debug(:,i) = copter.Force;%(mu)*copter.State(7:9);
  
    % Update accelerometer reading
    accel_noise = std_dev_acc*randn(3,1) + mean_err_acc;
    accel_drift = acc_drift_rate*time(i);
    % Neglecting vertical drag in next line
    Or = copter.State(4:6);
    Rdd = transpose([cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
                       cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
                       sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))]);
    accelerometer(:,i) = -cross(copter.State(10:12),copter.State(7:9)) - [mu*copter.State(7:8)/mass;sum(thrust(:,i))/mass] + accel_noise + accel_drift;
    
    % Update gyroscope reading
    gyro_noise = std_dev_gyro*randn(3,1) + mean_err_gyro;
    gyro_drift = gyro_drift_rate*time(i);
    gyro(:,i) = copter.State(10:12) + gyro_noise + gyro_drift;
    
    % Update flow sensor reading
    flow_noise = std_dev_flow*randn(2,1) + mean_err_flow;
    flow_drift = flow_drift_rate*time(i);
    flow(:,i) = copter.State(7:8) + flow_noise + flow_drift;
    if (mod(i,15) == 0)
        ekf.set_flow(flow(:,i))
        flow_time(end+1) = time(i); %#ok<SAGROW>
        flow_log(:,end+1) = flow(:,i); %#ok<SAGROW>
    end
    
    % Update altimeter reading
    altimeter_noise = std_dev_altimeter*randn(1,1) + mean_err_altimeter;
    altimeter_drift = altimeter_drift_rate*time(i);
    altimeter(i) = -copter.State(3) + altimeter_noise +altimeter_drift;
    if (mod(i,10) == 0)
        ekf.set_altimeter(altimeter(i))
        alt_time(end+1) = time(i); %#ok<SAGROW>
        alt_log(end+1) = altimeter(i); %#ok<SAGROW>
    end
    
    % Estimator
    xhat(:,i) = ekf.run_estimator(accelerometer(:,i), gyro(:,i), 0.01);
    
    %PX4 Attitude Estimator
    dt = 1 / 100;
    acc = Rot_matrix' * (vel - vel_prev)/dt;
    vel_prev = vel;
    orientation(:,i);
    phi = orientation(1,i);
    theta = orientation(2,i);
    psi = orientation(3,i);
    mag_true = [1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)]'*[cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)]'*[cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1]' * [1;0;0];
    mag_meas = mag_true + randn(3,1)*0.01;
    z_k = [gyro(:,i);accelerometer(:,i)-acc;mag_meas;];
    [euler(i,:),Rot_matrix,x_aposteriori_k,P_aposteriori_k] = attitudeKalmanfilter(dt,z_k',x_aposteriori_k,P_aposteriori_k);
    
    % RK4 integrator
    copter.State = copter.homebrewRK4();
    %copter.State(4:6) = inner_loop_set_points(1:3);
end
toc
% Set output variable when used as a function for monte carlo simulation
error = sqrt(copter.State(4)^2 + copter.State(5)^2 + copter.State(6)^2);
%%
figure(1)
subplot(2,1,1)
plot(time,[euler(:,1)-orientation(1,:)',xhat(4,:)'-orientation(1,:)'])
title('Roll')
xlabel('Time')
legend('PX4 Estimate Error','BYU Estimate Error','Location','SouthEast')
%axis([0 n/100 -.2 .2])

subplot(2,1,2)
plot(time,[euler(:,2)-orientation(2,:)',xhat(5,:)'-orientation(2,:)'])
title('Pitch')
xlabel('Time')
legend('PX4 Estimate Error','BYU Estimate Error')

print -depsc errorPlot.eps

figure(2)
subplot(2,1,1)
plot(time,[euler(:,1),xhat(4,:)',orientation(1,:)'])
title('Roll')
xlabel('Time')
legend('PX4 Estimate','BYU Estimate','Truth')
%axis([0 n/100 -.2 .2])

subplot(2,1,2)
plot(time,[euler(:,2),xhat(5,:)',orientation(2,:)'])
title('Pitch')
xlabel('Time')
legend('PX4 Estimate','BYU Estimate','Truth')

% print -depsc EKFPlot.eps
%axis([0 n/100 -.2 .2])

% figure(2)
% subplot(2,2,1)
% plot(time,position(1,:))
% 
% subplot(2,2,2)
% plot(time,position(2,:))
% 
% subplot(2,2,3)
% plot(time,position(3,:))
%figure(3)
%plot_quad(time,tmp,position,vert_vel,thrust)
% %{
% plot(time,debug(1,:),'r');
% hold on
% plot(time,debug(2,:),'g');
% plot(time,debug(3,:),'b');
% %}
% world_vel = zeros(3,n);
% %
% for i = 2:n
%     %world_vel(:,i) = (position(:,i)-position(:,i-1))*100;
%     world_vel(1,i) = (position(1,i)-position(1,i-1))*100;
%     world_vel(2,i) = (position(2,i)-position(2,i-1))*100;
%     world_vel(3,i) = (position(3,i)-position(3,i-1))*100;
% end
% figure(3)
% plot(time,body_vel(1,:),'r');
% hold on
% plot(time,body_vel(2,:),'g');
% plot(time,body_vel(3,:),'b');
% title('Body Frame Velocity');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% 
% figure(4)
% plot(time,world_vel(1,:),'r.-');
% hold on
% plot(time,world_vel(2,:),'g');
% plot(time,world_vel(3,:),'b');
% title('World Frame Velocity');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');

% plot states
% figure(5), clf
% set(gcf, 'Name', 'States')
% 
% subplot(7,1,1)
% plot(time, body_vel(1,:), time, xhat(1,:))
% legend('truth','estimate')
% title('u')
% ylabel('m/s')
% % set(gca, 'YLim', [min([1.5*min(body_vel(1,:)),0]), max([1.5*max(body_vel(1,:)),0])])
% 
% subplot(7,1,2)
% plot(time, body_vel(2,:), time, xhat(2,:))
% legend('truth','estimate')
% title('v')
% ylabel('m/s')
% % set(gca, 'YLim', [min([1.5*min(body_vel(2,:)),0]), max([1.5*max(body_vel(2,:)),0])])
% 
% subplot(7,1,3)
% plot(time, body_vel(3,:), time, xhat(3,:))
% legend('truth','estimate')
% title('w')
% ylabel('m/s')
% % set(gca, 'YLim', [min([1.5*min(body_vel(3,:)),0]), max([1.5*max(body_vel(3,:)),0])])
% 
% subplot(7,1,4)
% plot(time, orientation(1,:), time, xhat(4,:))
% legend('truth','estimate')
% title('\phi')
% ylabel('rad')
% % set(gca, 'YLim', [min([1.5*min(orientation(1,:)),0]), max([1.5*max(orientation(1,:)),0])])
% 
% subplot(7,1,5)
% plot(time, orientation(2,:), time, xhat(5,:))
% legend('truth','estimate')
% title('\theta')
% ylabel('rad')
% % set(gca, 'YLim', [min([1.5*min(orientation(2,:)),0]), max([1.5*max(orientation(2,:)),0])])
% 
% subplot(7,1,6)
% plot(time, ang_vel(3,:), time, xhat(6,:))
% legend('truth','estimate')
% title('\dot{\psi}')
% ylabel('rad/s')
% % set(gca, 'YLim', [min([1.5*min(ang_vel(3,:)),0]), max([1.5*max(ang_vel(3,:)),0])])
% 
% subplot(7,1,7)
% plot(time, position(3,:), time, -xhat(7,:))
% legend('truth','estimate')
% title('h')
% ylabel('m')
% % set(gca, 'YLim', [min([1.5*min(position(3,:)),0]), max([1.5*max(position(3,:)),0])])
% 
% % plot measurements
% figure(6), clf
% set(gcf, 'Name', 'Measurements')
% 
% subplot(4,1,1)
% plot(time, accelerometer(1,:), '.',...
%      time, accelerometer(2,:), '.',...
%      time, accelerometer(3,:), '.')
% legend('a_i', 'a_j', 'a_k')
% title('accelerometers')
% ylabel('m/s^2')
% 
% subplot(4,1,2)
% plot(time, gyro(1,:), '.',...
%      time, gyro(2,:), '.',...
%      time, gyro(3,:), '.')
% legend('p', 'q', 'r')
% title('gyros')
% ylabel('rad/s')
% 
% subplot(4,1,3)
% plot(alt_time, alt_log, '.')
% title('altimeter')
% ylabel('m')
% 
% subplot(4,1,4)
% plot(flow_time, flow_log(1,:), '.',...
%      flow_time, flow_log(2,:), '.')
% legend('u', 'v')
% title('flow sensor')
% ylabel('m/s')
% 
% %}