
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
obstacleGrid = ones((2*m+1),(2*n+1),2);
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

maxSteps = 2000;
[time_to_detection_array,correct_identification_list]= experiment(1);
42






