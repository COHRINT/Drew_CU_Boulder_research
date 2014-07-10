
clc
clear
close all
clear sweepingSearch2

%Initialize grid
global  m;
global  n;
global Xv;
global Yv;
global Xg;
global Yg;
global A;
global obstacleGrid;
global alphaList;
global betaList;
global maxSteps;

alphaList = 0.2:0.2:0.8;
betaList = 0.2:0.2:0.8;

m = 15;                                                                    %grid y extent   
n = 15;                                                                    %grid x extent

Xv = -m:m;
Yv = -n:n;
[Xg ,Yg]=meshgrid(Xv,Yv);                                                  

%Set up obstacles 
obstacleGrid = ones((2*m+1),(2*n+1),5);
obstacleGrid(14:18,14:18,1) = 0;
obstacleGrid(12:19,19:23,1) = 0;
obstacleGrid(3:11,5:8,1) = 0;
obstacleGrid(22:25,2:6,1) = 0;
obstacleGrid(11:13,4:19,1) = 0;

%SET UP PRIORS
                                            
%%(truncated) Gaussian Prior
sigmax = sqrt(27);
sigmay = sqrt(15);
mux = 1;
muy = 3;
gaussPrior = exp(-((Xg-mux).^2/(2*sigmax^2)+(Yg-muy).^2/(2*sigmay)^2));
A(:,:,1) = gaussPrior;                              
A(:,:,1) = A(:,:,1)./(sum(sum(A(:,:,1))));

%%Uniform Prior
A(:,:,2) = zeros(size(Xg));
A(:,:,2) = 1/(numel(A));  

%Gaussian Superposition

sigmax1 = sqrt(27);
sigmay1 = sqrt(15);
mux1 = 1;
muy1 = 3;
sigmax2 = sqrt(5);
sigmay2 = sqrt(13);
mux2 = 10;
muy2 = 12;
sigmax3 = sqrt(5);
sigmay3 = sqrt(13);
mux3 = -10;
muy3 = -12;
sigmax4 = sqrt(5);
sigmay4 = sqrt(13);
mux4 = 10;
muy4 = -12;
amp1 = 1;
amp2 = 1;
amp3 = 0;
amp4 = 0;

gaussSuperposePrior = amp1*exp(-((Xg-mux1).^2/(2*sigmax1^2)+(Yg-muy1).^2/(2*sigmay1)^2))+amp2*exp(-((Xg-mux2).^2/(2*sigmax2^2)+(Yg-muy2).^2/(2*sigmay2)^2))+amp3*exp(-((Xg-mux3).^2/(2*sigmax3^2)+(Yg-muy3).^2/(2*sigmay3)^2))+amp4*exp(-((Xg-mux4).^2/(2*sigmax4^2)+(Yg-muy4).^2/(2*sigmay4)^2)); 
A(:,:,3) = gaussSuperposePrior;                              
A(:,:,3) = A(:,:,3)./(sum(sum(A(:,:,3))));

amp3 = 1;

gaussSuperposePrior = amp1*exp(-((Xg-mux1).^2/(2*sigmax1^2)+(Yg-muy1).^2/(2*sigmay1)^2))+amp2*exp(-((Xg-mux2).^2/(2*sigmax2^2)+(Yg-muy2).^2/(2*sigmay2)^2))+amp3*exp(-((Xg-mux3).^2/(2*sigmax3^2)+(Yg-muy3).^2/(2*sigmay3)^2))+amp4*exp(-((Xg-mux4).^2/(2*sigmax4^2)+(Yg-muy4).^2/(2*sigmay4)^2)); 
A(:,:,4) = gaussSuperposePrior;                              
A(:,:,4) = A(:,:,4)./(sum(sum(A(:,:,4))));

amp4 = 1;

gaussSuperposePrior = amp1*exp(-((Xg-mux1).^2/(2*sigmax1^2)+(Yg-muy1).^2/(2*sigmay1)^2))+amp2*exp(-((Xg-mux2).^2/(2*sigmax2^2)+(Yg-muy2).^2/(2*sigmay2)^2))+amp3*exp(-((Xg-mux3).^2/(2*sigmax3^2)+(Yg-muy3).^2/(2*sigmay3)^2))+amp4*exp(-((Xg-mux4).^2/(2*sigmax4^2)+(Yg-muy4).^2/(2*sigmay4)^2)); 
A(:,:,5) = gaussSuperposePrior;                              
A(:,:,5) = A(:,:,5)./(sum(sum(A(:,:,5))));

%Highly Non-Uniform
A(:,:,6) = zeros(size(Xg));
A(:,:,6) = rand(2*m+1,2*n+1);
A(:,:,6) = A(:,:,4)./(sum(sum(A(:,:,4))));

%Begin Search Simulation
%Experiment Types
% 1 : Different Obstacle Configurations
% 2 : Different Prior Distributions
% 3 : Variations in Alpha (Beta = 0)
% 4 : Variations in Beta (Alpha = 0)
% 4 : Lookahead window (2 - 5)

maxSteps = 5000;

[time_to_detection_array,correct_identification_list]= experiment_type(100,3);
42






