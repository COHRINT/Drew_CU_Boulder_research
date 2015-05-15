%% Gaussian Process Estimation of Control Inputs (Velocity, Turnrate)
% Author: Drew Ellison
% Date created: February 19, 2015
% Data modified: February 19, 2015

% Housekeeping
clear all
close all
clc

% Generate test data points for GP
x = 0:.25:10;
y = 0:.25:10;

% Define GP parameters
sigma_f = 1;
sigma_n = 1;
l = diag([.1,.1]);

% Run GP on test data
[x_data,y_data,GP_data] = GP(x,y,l,sigma_f,sigma_n);

surf(x_data,y_data,GP_data)