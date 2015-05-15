classdef target_truth < handle
    properties
        x_pos
        y_pos
        u
        v
        planned_path
    end
    methods
        function obj = target_truth(x,y,u,v)
            if nargin ~= 4
                disp(['target: expected [' num2str(4)...
                        '] arguments, got [' num2str(nargin) ']. Initializing to default.'])
                    obj.x_pos = 0;
                    obj.y_pos = 0;
                    obj.u = 0;
                    obj.v = 0;
                    obj.planned_path = [];
            else
                obj.x_pos = x;
                obj.y_pos = y;
                obj.u = u;
                obj.v = v;
                obj.planned_path = [];
            end
        end
        
        function plan_path(obj,edges,points,goal)
            if isempty(goal)
                return;
            end
            
            if isempty(obj.planned_path)
                start_id = find_nearest_graph_cell(obj.x_pos,obj.y_pos,points);
                goal_id = find_nearest_graph_cell(goal(1),goal(2),points);
                obj.planned_path = A_star(1000,start_id,goal_id,edges,points);
            end
        end
        
        function update_pos(obj,edges,points,goal)
            if isempty(obj.planned_path)
                obj.plan_path(edges,points,goal);
            end
            if ~isempty(obj.planned_path)
                obj.x_pos = points(obj.planned_path(1),1);
                obj.y_pos = points(obj.planned_path(1),2);
                obj.planned_path(1,:) = [];
            end
        end
    end
end