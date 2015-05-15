%   Author: Drew Ellison
%   Email: dme722@gmail.com
%   File Description:
%   The experiment_start program initializes global variables including
%   grid size, obstacles, maximum number of allowed steps in a given
%   simulations, and the lookahead window used by the optimal lookahead
%   function. 
clc
clear
close all
%Clear functions with persistent variables
clear sweepingSearch
clear drosophLookaheadHybrid

%Initialize grid
global  m;  %Grid y extent
global  n;  %Grid x extent
global Xv;  %Grid x list
global Yv;  %Grid y list
global Xg;  %Mesh grid variables
global Yg;
global obstacleGrid;   %Obstacle grid setup
global maxSteps;    %Maximum number of steps in simulation allowed
global lookahead;   %Lookahead window used in optimal lookahead search



m = 15;                                                                     
n = 15;                                                                   

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

maxSteps = 20;
lookahead = 3;

%Begin experiment
[time_to_detection_array,correct_identification_list]= experiment(1);
%When experiment is complete, display the answer to life, the universe and
%everything. 
42






