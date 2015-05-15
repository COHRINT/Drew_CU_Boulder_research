classdef quad < handle
    properties
        agent_truth
        pos_estimate
        space
        search2
        accel_noise
        accel_meas_noise
        accel_process_noise
        planned_path
        crashed
        target_truth
        hmm
        edges
        points
    end
    methods
        function obj = quad(agent_truth,space,search2,pos_estimate,target_truth)
            obj.agent_truth = agent_truth;
            obj.space = space;
            obj.search2 = search2;
            obj.pos_estimate = pos_estimate;
            obj.accel_noise = 0.1;
            obj.accel_process_noise = 0.05;
            obj.planned_path = [];
            obj.crashed = false;
            obj.target_truth = target_truth;
            obj.hmm = eye(size(obj.space.grid_x,1)*size(obj.space.grid_x,2));
        end
        
        function plan_path(obj,window,n)
            if isempty(obj.planned_path)
                current_pos = obj.pos_estimate.state(1:2);
                %obj.planned_path = optimal_lookahead(current_pos,obj.space,obj.search2,obj.agent_truth.radius_of_detection,window,obj.agent_truth.radius_of_detection+obj.space.dist_bn_cells,n);
                [obj.edges,obj.points,obj.planned_path] = drosoph_search(current_pos,obj.space,obj.search2);
            end
        end
        
        function [udot_commaned,vdot_commanded] = run_controller(obj)
            if ~isempty(obj.planned_path)
                [udot_commaned,vdot_commanded] = controller(obj.planned_path(1,:),obj.pos_estimate,obj.space);
            else
                [udot_commaned,vdot_commanded] = controller(obj.pos_estimate.state(1:2),obj.pos_estimate,obj.space);
            end
        end
        
        function update_truth(obj)
            persistent t;
            if isempty(t)
                t = 0;
            end
            if t==1
                [udot_temp,vdot_temp] = obj.run_controller;
                t = 0;
                obj.agent_truth.udot = udot_temp + obj.accel_process_noise * randn;
                obj.agent_truth.vdot = vdot_temp + obj.accel_process_noise * randn;
            end
            
            t = t + 1;
            
            global dt;
            
            u = obj.agent_truth.u + obj.agent_truth.udot * dt;
            v = obj.agent_truth.v + obj.agent_truth.vdot * dt;
            
            obj.agent_truth.u = u;
            obj.agent_truth.v = v;
            
            psi = obj.agent_truth.heading;
            
            vx = u*cos(psi) - v*sin(psi);
            vy = -u*sin(psi) - v*cos(psi);
            
            obj.agent_truth.x_pos = obj.agent_truth.x_pos + vx*dt;
            obj.agent_truth.y_pos = obj.agent_truth.y_pos + vy*dt;
        end
        
        function accel_reading = take_imu_meas(obj)
            accel_reading = [obj.agent_truth.udot,obj.agent_truth.vdot] + obj.accel_noise * randn;
        end
        
        function estimator(obj)
            accel = obj.take_imu_meas;
            udot = accel(1);
            vdot = accel(2);
            
            global dt
            persistent time;
            if isempty(time)
                time = 0;
            end
%             u = obj.pos_estimate.state(3) + udot*dt;
%             v = obj.pos_estimate.state(4) + vdot*dt;
%             
%             psi = obj.pos_estimate.state(5);
%             
%             vx = u*cos(psi) - v*sin(psi);
%             vy = -u*sin(psi) - v*cos(psi);
%             
%             x = obj.pos_estimate.state(1) + vx*dt;
%             y = obj.pos_estimate.state(2) + vy*dt;
%             
%             psi = psi; %Not needed right now, but may in the future
%             new_state = [x,y,u,v,psi,udot,vdot];
%             
%             obj.pos_estimate.state = new_state;
            if time > 5
                gps = [obj.agent_truth.x_pos,obj.agent_truth.y_pos,obj.agent_truth.u,obj.agent_truth.v];
                time = 0;
            else
                gps = [];
            end
            time = time + dt;
            obj.pos_estimate.run_ekf(accel,gps)
        end
        
        function update_path(obj)
            
            r_thresh = obj.agent_truth.radius_of_detection;
            v_thresh = 0.2;
            
            x = obj.pos_estimate.state(1);
            y = obj.pos_estimate.state(2);
            x_command = obj.planned_path(1,1);
            y_command = obj.planned_path(1,2);
            
            dist_from_target = norm([x-x_command,y-y_command]);
            
            u = obj.pos_estimate.state(3);
            v = obj.pos_estimate.state(4);
            
            v_tot = norm([u,v]);
            
            if dist_from_target < r_thresh && v_tot < v_thresh
                obj.planned_path(1,:) = [];
            end
        end
        
        function look_for_target(obj,target_truth)
            global dt;
            persistent time;
            if isempty(time)
                time = 0;
            end
            if time > 1
                obj.planned_path = obj.search2.update_prior(obj.space,obj.agent_truth,target_truth,obj.pos_estimate,obj.planned_path);
                time = 0;
            end
            time = time + dt;
        end
        
        function propagateBelief(obj,targets)
            persistent time;
            global dt;
            if isempty(time)
                time = 0;
            end
            if time > 4
                [obj.search2.A,obj.hmm] = propagateBeliefHMM(obj.space,obj.search2,targets);
                time = 0;
            end
            time = time + dt;
        end
        function moveTarget(obj)
            persistent time;
            global dt;
            if isempty(time)
                time = 0;
            end
            if time > 5
                obj.target_truth.update_pos(obj.edges,obj.points,[])
                time = 0;
            end
            time = time + dt;
        end
        
        function go(obj,window,n,target_truth)
            
            persistent obstacle_grid;
            if isempty(obstacle_grid)
                obstacle_grid = zeros(size(obj.space.grid_x));
                for i = 1:size(obj.space.obstacle_pos,1)
                    [~,~,obs_x,obs_y] = obj.space.find_cell(obj.space.obstacle_pos(i,1),obj.space.obstacle_pos(i,2));
                    obstacle_grid(obs_y,obs_x) = 1;
                end
            end
            
            obj.plan_path(window,n);
            obj.update_truth; %Controller ran in here
            obj.estimator;
            obj.update_path;
            [~,~,xind,yind] = obj.space.find_cell(obj.agent_truth.x_pos,obj.agent_truth.y_pos);
            obj.crashed = obstacle_grid(yind,xind);
            obj.propagateBelief([]);
            obj.moveTarget;
            if ~obj.crashed
                obj.look_for_target(target_truth);
            end
        end
    end
end
        
       