clear
clear plotGrid
clear quad
clear space
clear search2
clear optimal_lookahead
clear update_truth
clear drosoph_search
clear propagateBeliefHMM
clc
close all
global dt;


dt = 0.01;
time = 0;
maxTime = 1000;

lookahead_window = 1;
paths_per_node = 10 ;

%=======================INITIALIZE CLASSES================================

agent_properties = agent_truth(1,1,0,0,0,1,2*pi,.2,0.1);
target_truth = target_truth(9,5,0,0);
space = space([0,10],[0,10],2000);
space.add_obstacle([2,4],[3,4.75]);
space.add_obstacle([6,8],[3,4.75]);
space.add_obstacle([2,4],[5.75,8]);
space.add_obstacle([6,8],[5.75,8]);
search = search3(space);
A = mvnpdf([space.grid_x(:) space.grid_y(:)],[5 5],[1 .5;.5 1]);
A = reshape(A,length(space.grid_y),length(space.grid_x));
%A = ones(size(space.grid_x));
search.assign_prior(A,space);
initial_state = [agent_properties.x_pos;agent_properties.y_pos;agent_properties.u;agent_properties.v;agent_properties.udot;agent_properties.vdot;agent_properties.heading];
pos_estimate = pos_estimate(initial_state);
count = 1;

%=======================INITIALIZE QUAD===================================

plot_counter = 50;
plot_period = 50;

% filename = 'continuous_search.avi';
% aviobj = VideoWriter(filename);
% open(aviobj);
% hFig = figure('Visible','Off');

accel_readings = [];
vel_est = [];
vel_truth = [];
time_list = [];
x_covar_list = [];

quad = quad(agent_properties,space,search,pos_estimate,target_truth);
while ~quad.crashed
    quad.go(lookahead_window,paths_per_node,target_truth);
    time = time+dt;
    time_list = [time_list,time];
    accel_readings = [accel_readings;quad.pos_estimate.state(6:7)];
    vel_est = [vel_est;quad.pos_estimate.state(3:4)];
    vel_truth = [vel_truth;quad.agent_truth.u,quad.agent_truth.v];
    x_covar_list = [x_covar_list,quad.pos_estimate.P(1)];
    %plot(time_list,x_covar_list)
    
    if plot_counter == plot_period
        plotGrid(target_truth,quad.pos_estimate,quad.agent_truth,quad.space,quad.search2,time_list,accel_readings,vel_est,vel_truth);
        pause(0.01)
        plot_counter = 0;
        
%         img = hardcopy(hFig,'-dzbuffer','r0');
%         writeVideo(aviobj,im2frame(img));
    end
    plot_counter = plot_counter + 1;
    %sum(sum(quad.search2.A(~isnan(quad.search2.A))))
end
% close(aviobj);

disp('Crashed!!!!!!!')