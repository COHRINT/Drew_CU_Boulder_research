
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
global stepsSinceLastExplore;

stepsSinceLastExplore = [];

alphaList = 0.2:0.2:0.8;
betaList = 0.2:0.2:0.8;

m = 15;                                                                    %grid y extent   
n = 15;                                                                    %grid x extent

Xv = -m:m;
Yv = -n:n;
[Xg ,Yg]=meshgrid(Xv,Yv);                                                  

%Set up obstacles 
obstacleGrid = ones((2*m+1),(2*n+1),1);
obstacleGrid(7:7,1:10,1) = 0;
obstacleGrid(3:7,10:10,1) = 0;
obstacleGrid(10:10,1:10,1) = 0;
obstacleGrid(10:17,10:10,1) = 0;
obstacleGrid(17:17,5:10,1) = 0;
obstacleGrid(14:17,5:5,1) = 0;
obstacleGrid(20:23,5:5,1) = 0;
obstacleGrid(23:23,3:5,1) = 0;
obstacleGrid(20:20,5:10,1) = 0;
obstacleGrid(20:31,10:10,1) = 0;
obstacleGrid(28:31,13:13,1) = 0;
obstacleGrid(20:25,13:13,1) = 0;
obstacleGrid(20:20,13:18,1) = 0;
obstacleGrid(20:20,21:31,1) = 0;
obstacleGrid(28:31,25:25,1) = 0;
obstacleGrid(23:25,25:25,1) = 0;
obstacleGrid(23:23,25:31,1) = 0;
obstacleGrid(15:15,15:31,1) = 0;
obstacleGrid(5:15,15:15,1) = 0;
obstacleGrid(11:15,22:22,1) = 0;
obstacleGrid(11:11,22:24,1) = 0;
obstacleGrid(11:11,27:31,1) = 0;
obstacleGrid(3:5,24:26,1) = 0;

%SET UP PRIORS
%                                             
% %%(truncated) Gaussian Prior
% sigmax = sqrt(27);
% sigmay = sqrt(15);
% mux = 1;
% muy = 3;
% gaussPrior = exp(-((Xg-mux).^2/(2*sigmax^2)+(Yg-muy).^2/(2*sigmay)^2));
% A(:,:,1) = gaussPrior;                              
% A(:,:,1) = A(:,:,1)./(sum(sum(A(:,:,1))));
% 
% %%Uniform Prior
% A(:,:,2) = zeros(size(Xg));
% A(:,:,2) = 1/(numel(A));  
% 
% %Gaussian Superposition
% 
% sigmax1 = sqrt(27);
% sigmay1 = sqrt(15);
% mux1 = 1;
% muy1 = 3;
% sigmax2 = sqrt(5);
% sigmay2 = sqrt(13);
% mux2 = 10;
% muy2 = 12;
% sigmax3 = sqrt(5);
% sigmay3 = sqrt(13);
% mux3 = -10;
% muy3 = -12;
% sigmax4 = sqrt(5);
% sigmay4 = sqrt(13);
% mux4 = 10;
% muy4 = -12;
% amp1 = 1;
% amp2 = 1;
% amp3 = 0;
% amp4 = 0;
% 
% gaussSuperposePrior = amp1*exp(-((Xg-mux1).^2/(2*sigmax1^2)+(Yg-muy1).^2/(2*sigmay1)^2))+amp2*exp(-((Xg-mux2).^2/(2*sigmax2^2)+(Yg-muy2).^2/(2*sigmay2)^2))+amp3*exp(-((Xg-mux3).^2/(2*sigmax3^2)+(Yg-muy3).^2/(2*sigmay3)^2))+amp4*exp(-((Xg-mux4).^2/(2*sigmax4^2)+(Yg-muy4).^2/(2*sigmay4)^2)); 
% A(:,:,3) = gaussSuperposePrior;                              
% A(:,:,3) = A(:,:,3)./(sum(sum(A(:,:,3))));
% 
% amp3 = 1;
% 
% gaussSuperposePrior = amp1*exp(-((Xg-mux1).^2/(2*sigmax1^2)+(Yg-muy1).^2/(2*sigmay1)^2))+amp2*exp(-((Xg-mux2).^2/(2*sigmax2^2)+(Yg-muy2).^2/(2*sigmay2)^2))+amp3*exp(-((Xg-mux3).^2/(2*sigmax3^2)+(Yg-muy3).^2/(2*sigmay3)^2))+amp4*exp(-((Xg-mux4).^2/(2*sigmax4^2)+(Yg-muy4).^2/(2*sigmay4)^2)); 
% A(:,:,4) = gaussSuperposePrior;                              
% A(:,:,4) = A(:,:,4)./(sum(sum(A(:,:,4))));
% 
% amp4 = 1;
% 
% gaussSuperposePrior = amp1*exp(-((Xg-mux1).^2/(2*sigmax1^2)+(Yg-muy1).^2/(2*sigmay1)^2))+amp2*exp(-((Xg-mux2).^2/(2*sigmax2^2)+(Yg-muy2).^2/(2*sigmay2)^2))+amp3*exp(-((Xg-mux3).^2/(2*sigmax3^2)+(Yg-muy3).^2/(2*sigmay3)^2))+amp4*exp(-((Xg-mux4).^2/(2*sigmax4^2)+(Yg-muy4).^2/(2*sigmay4)^2)); 
% A(:,:,5) = gaussSuperposePrior;                              
% A(:,:,5) = A(:,:,5)./(sum(sum(A(:,:,5))));
% 
% %Highly Non-Uniform
% A(:,:,6) = zeros(size(Xg));
% A(:,:,6) = rand(2*m+1,2*n+1);
% A(:,:,6) = A(:,:,4)./(sum(sum(A(:,:,4))));

sigmax1 = 1.5;
sigmay1 = 1;
muy1 = 7;
mux1 = -8;
sigmax2 = 1.5;
sigmay2 = 1;
muy2 = -3;
mux2 = -9;
sigmax3 = 3;
sigmay3 = 1;
muy3 = -12;
mux3 = -12;
sigmax4 = 2;
sigmay4 = 2;
muy4 = -12;
mux4 = 13;
sigmax5 = 1;
sigmay5= 1;
muy5 = -3;
mux5 = 16;
sigmax6 = 3;
sigmay6 = 3;
muy6 = 13;
mux6 = 1;
sigmax7 = 2;
sigmay7 = 2;
muy7 = 13;
mux7 = 13;
amp1 = 0;
amp2 = 1;
amp3 = 1;
amp4 = 0;
amp5 = 0;
amp6 = 0;
amp7 = 0;

gaussSuperposePrior = amp1*exp(-((Xg-mux1).^2/(2*sigmax1^2)+(Yg-muy1).^2/(2*sigmay1)^2))+amp2*exp(-((Xg-mux2).^2/(2*sigmax2^2)+(Yg-muy2).^2/(2*sigmay2)^2))+amp3*exp(-((Xg-mux3).^2/(2*sigmax3^2)+(Yg-muy3).^2/(2*sigmay3)^2))+amp4*exp(-((Xg-mux4).^2/(2*sigmax4^2)+(Yg-muy4).^2/(2*sigmay4)^2))+amp5*exp(-((Xg-mux5).^2/(2*sigmax5^2)+(Yg-muy5).^2/(2*sigmay5)^2))+amp6*exp(-((Xg-mux6).^2/(2*sigmax6^2)+(Yg-muy6).^2/(2*sigmay6)^2))+amp7*exp(-((Xg-mux7).^2/(2*sigmax7^2)+(Yg-muy7).^2/(2*sigmay7)^2)); 
A(:,:,1) = gaussSuperposePrior;                              
A(:,:,1) = A(:,:,1)./(sum(sum(A(:,:,1))));
%Begin Search Simulation
%Experiment Types
% 1 : Different Obstacle Configurations
% 2 : Different Prior Distributions
% 3 : Variations in Alpha (Beta = 0)
% 4 : Variations in Beta (Alpha = 0)
% 4 : Lookahead window (2 - 5)

maxSteps = 2000;

[time_to_detection_array,correct_identification_list]= experiment_type(100,1);
42






