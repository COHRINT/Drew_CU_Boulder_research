classdef target < handle
    properties
        pose
        velocity
        type
        map
        map_graph
        waypoint_list
        edge_list
        goal_list
        goal
        goal_HMM
        goal_history
    end
    methods
        function obj = target(pose,velocity,type,map,goal_list,goal_HMM)
            obj.pose = pose;
            obj.velocity = velocity;
            obj.type = type;
            obj.map = map;
            [obj.edge_list,obj.map_graph] = obj.discretize_map(map,1000);
            obj.waypoint_list = [];
            obj.goal_list = goal_list;
            obj.goal_HMM = goal_HMM;
            obj.goal = obj.choose_goal();
            obj.goal_history = [];
        end
        function [edge_list ,discretized_map] = discretize_map(obj,map,n)
            prm_points = [];
            
            x_bounds = [0,size(map,2)-1];
            y_bounds = [0,size(map,1)-1];
            
            count = 0;
            while count < n
                x = rand*(x_bounds(2) - x_bounds(1)) + x_bounds(1);
                y = rand*(y_bounds(2) - y_bounds(1)) + y_bounds(1);
                if ~map(ceil(y),ceil(x))
                    prm_points = [prm_points;x y];
                    count = count + 1;
                end
            end
            obstacle_pos = [];
            for i = 1:size(map,1)
                for j = 1:size(map,2)
                    if obj.map(j,i) == 1
                        obstacle_pos = [obstacle_pos;i-1,j-1];
                    end
                end
            end
            
            prm_points = [prm_points;obstacle_pos];
            
            tri = delaunayTriangulation(prm_points(:,1),prm_points(:,2));
            %triplot(tri)
            %hold on
            %scatter(prm_points(:,1),prm_points(:,2),'ro')
            %hold on
            
            e = edges(tri);
            
            e = [e;e(:,2),e(:,1)];
            
            e_new = [];
            
            for i = 1:size(e,1)
                if e(i,1) <= n && e(i,2) <= n
                    point1 = prm_points(e(i,1),:);
                    point2 = prm_points(e(i,2),:);
                    
                    dist = sqrt((point1(1)-point2(1))^2+(point1(2)-point2(2))^2);
                    e_new = [e_new;e(i,:),dist];
                end
            end
            discretized_map = prm_points;
            edge_list = e_new;
        end
        function goal = choose_goal(obj)
            if isempty(obj.goal)
                goal = randi(size(obj.goal_list,1));
                return
            end
            trans_probs = obj.goal_HMM(obj.goal,:);
            cum_prob = cumsum(trans_probs);
            r = rand;
            
            for i = 1:size(cum_prob,2)
                if r < cum_prob(i)
                    goal = i;
                    break
                end
            end
        end
        function ind = find_nearest_pt(obj,x,y)
            dist = inf;
            for i = 1:size(obj.map_graph,1)
                dist_temp = (obj.map_graph(i,1) - x)^2+(obj.map_graph(i,2) - y)^2;
                if dist_temp<dist
                    dist = dist_temp;
                    ind = i;
                end
            end
        end
        function move(obj)
            global dt
            global search_agn_pose
            
            if obj.type == 1
                if norm([obj.pose(1) - obj.goal_list(obj.goal,1),obj.pose(2) -obj.goal_list(obj.goal,2)]) < 2 || isempty(obj.waypoint_list)
                    obj.goal = obj.choose_goal();
                    obj.goal_history = [obj.goal_history;obj.goal];
                    cur_pose_ind = obj.find_nearest_pt(obj.pose(1),obj.pose(2));
                    cur_goal_ind = obj.find_nearest_pt(obj.goal_list(obj.goal,1),obj.goal_list(obj.goal,2));
                    
                    obj.waypoint_list = A_star(1000,cur_pose_ind,cur_goal_ind,obj.edge_list,obj.map_graph);
                end
                
                if (obj.pose(1) - obj.map_graph(obj.waypoint_list(1),1))^2 + (obj.pose(2) -obj.map_graph(obj.waypoint_list(1),2))^2 < 2
                    obj.waypoint_list = obj.waypoint_list(2:end,:);
                end
                
                if norm([obj.pose(1) - obj.goal_list(obj.goal,1),obj.pose(2) -obj.goal_list(obj.goal,2)]) < 2 || isempty(obj.waypoint_list)
                    obj.goal = obj.choose_goal();
                    obj.goal_history = [obj.goal_history;obj.goal];
                    cur_pose_ind = obj.find_nearest_pt(obj.pose(1),obj.pose(2));
                    cur_goal_ind = obj.find_nearest_pt(obj.goal_list(obj.goal,1),obj.goal_list(obj.goal,2));
                    
                    obj.waypoint_list = A_star(1000,cur_pose_ind,cur_goal_ind,obj.edge_list,obj.map_graph);
                end
            end
            if obj.type == 2
                persistent count
                if isempty(count)
                    count = 0;
                end
                if norm([obj.pose(1) - obj.goal_list(obj.goal,1),obj.pose(2) -obj.goal_list(obj.goal,2)]) < 2 || isempty(obj.waypoint_list)
                    if count < 3
                        count = count + 1;
                        obj.goal = obj.choose_goal();
                        obj.goal_history = [obj.goal_history;obj.goal];
                        cur_pose_ind = obj.find_nearest_pt(obj.pose(1),obj.pose(2));
                        cur_goal_ind = obj.find_nearest_pt(obj.goal_list(obj.goal,1),obj.goal_list(obj.goal,2));
                        
                        obj.waypoint_list = A_star(1000,cur_pose_ind,cur_goal_ind,obj.edge_list,obj.map_graph);
%                     else
%                         count = count + 1;
%                         if count == 4
%                             cur_pose_ind = obj.find_nearest_pt(obj.pose(1),obj.pose(2));
%                             possible_goal_ind = [];
%                             for i = 1:size(obj.map_graph,1)
%                                 if norm(obj.map_graph(i,:) - obj.pose) < 15 && norm(obj.map_graph(i,:) - obj.pose) < 10
%                                     possible_goal_ind = [possible_goal_ind,i];
%                                 end
%                             end
%                             
%                             cur_goal_ind = possible_goal_ind(randi(size(possible_goal_ind,1)));
%                             obj.waypoint_list = A_star(1000,cur_pose_ind,cur_goal_ind,obj.edge_list,obj.map_graph);
%                             count = count + 1;
%                         elseif count == 5
%                             count = 0;
%                         end
                    end
                end
                
                if (obj.pose(1) - obj.map_graph(obj.waypoint_list(1),1))^2 + (obj.pose(2) -obj.map_graph(obj.waypoint_list(1),2))^2 < 2
                    obj.waypoint_list = obj.waypoint_list(2:end,:);
                end
                
                if norm([obj.pose(1) - obj.goal_list(obj.goal,1),obj.pose(2) -obj.goal_list(obj.goal,2)]) < 2 || isempty(obj.waypoint_list)
                   if count < 3
                        count = count + 1;
                        obj.goal = obj.choose_goal();
                        obj.goal_history = [obj.goal_history;obj.goal];
                        cur_pose_ind = obj.find_nearest_pt(obj.pose(1),obj.pose(2));
                        cur_goal_ind = obj.find_nearest_pt(obj.goal_list(obj.goal,1),obj.goal_list(obj.goal,2));
                        
                        obj.waypoint_list = A_star(1000,cur_pose_ind,cur_goal_ind,obj.edge_list,obj.map_graph);
                    else
                        count = count + 1;
                        if count == 4
                            cur_pose_ind = obj.find_nearest_pt(obj.pose(1),obj.pose(2));
                            dist_min = inf;
                            for i = 1:size(obj.map_graph,1)
                                if norm(obj.map_graph(i,:) - search_agn_pose) < 25 && norm(obj.map_graph(i,:) - search_agn_pose) > 10 && mod(obj.map_graph(i,1),1) ~=0
                                    if norm(obj.map_graph(i,:) - obj.pose) < dist_min
                                        dist_min = norm(obj.map_graph(i,:) - obj.pose);
                                        cur_goal_ind = obj.find_nearest_pt(obj.map_graph(i,1),obj.map_graph(i,2));
                                    end
                                end
                            end
                            obj.waypoint_list = A_star(1000,cur_pose_ind,cur_goal_ind,obj.edge_list,obj.map_graph);
                        elseif count == 5
                            cur_pose_ind = obj.find_nearest_pt(obj.pose(1),obj.pose(2));
                            cur_goal_ind = obj.find_nearest_pt(search_agn_pose(1),search_agn_pose(2));
                            obj.waypoint_list = A_star(1000,cur_pose_ind,cur_goal_ind,obj.edge_list,obj.map_graph);
                            count = 0;
                        end
                    end
                end
                count
            end
            accel_max = 4;
            v_max = 8;
            k_accel = 1;
            k_vel = 1;
            vel_des = k_vel*[obj.map_graph(obj.waypoint_list(1),1) - obj.pose(1),obj.map_graph(obj.waypoint_list(1),2) - obj.pose(2)];
            accel = k_accel*[vel_des(1) - obj.velocity(1),vel_des(2) - obj.velocity(2)];
            if norm(accel) > accel_max
                accel = accel_max*accel/norm(accel);
            end
            obj.velocity = obj.velocity + accel*dt;
            if norm(obj.velocity) > v_max
                obj.velocity = v_max * obj.velocity/norm(obj.velocity);
            end
            obj.pose(:) = obj.pose(:) + obj.velocity(:)*dt;
        end
    end
end