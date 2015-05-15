clear all
close all
clc
clear target

global dt
global search_agn_pose

dt = 0.1;
search_agn_pose = [40,30];
map = csvread('map1.csv');
goal_list = [10,32;10,70;32,90;95,10;72,90];

goal_HMM= [0 0.4 0.1 0.3 0.2
           0.1 0.2 0.4 0.1 0.2
           0.1 0.3 0 0.4 0.2
           0.3 0.2 0.1 0 0.4
           0.4 0.2 0.1 0.3 0];
 obj_dropoff_hmm = [0 1 0 0 0
           0 0 1 0 0
           0 0 0 1 0
           0 0 0 0 1
           1 0 0 0 0];
 obj_have = [0 0 0 0 0];

target = target([40,10],[0,0],2,map,goal_list,goal_HMM);
pose_history = [];
goal_history = [];

pcolor(~map)
hold on
colormap('gray')
shading('interp')
i = 1;
%pause(1)
while i < 100000
    %i = i + 1;
    clf
    target.move();
    target.pose;
    
    pcolor(~map)
    hold on
    colormap('gray')
    shading('interp')
    axis([0 101 0 101])
    hold on
    plot3(target.pose(1),target.pose(2),2,'mx','MarkerSize',12,'LineWidth',2)
    hold on
    plot3(search_agn_pose(1),search_agn_pose(2),2,'ro','MarkerSize',12,'LineWidth',2)
    
    for i = 1:size(target.goal_list,1)
        plot3(target.goal_list(i,1),target.goal_list(i,2),0.02,'rx','MarkerSize',12,'LineWidth',2);
        hold on
    end
    pose_history = [pose_history;target.pose(1),target.pose(2)];
    pause(.0001)
end 

csvwrite('pose_history.csv',pose_history)
csvwrite('goal_history.csv',target.goal_history)