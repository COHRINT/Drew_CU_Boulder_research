classdef space < handle
    properties
        x_extent
        y_extent
        obstacle_pos
        feature_pos
        grid_x
        grid_y
        dist_bn_cells
    end
    methods
        function obj = space(x_extent,y_extent,num_cells)
            obj.obstacle_pos = [];
            obj.x_extent = x_extent;
            obj.y_extent = y_extent;
            obj.set_cell_list(x_extent,y_extent,num_cells);
            obj.feature_pos = [];
        end
        
        function set_cell_list(obj,x_extent,y_extent,num_cells)
            a_cell = abs((x_extent(2)-x_extent(1))*(y_extent(2)-y_extent(1)))/num_cells;
            obj.dist_bn_cells = sqrt(a_cell);
            xlist = x_extent(1):obj.dist_bn_cells:x_extent(2);
            ylist = y_extent(1):obj.dist_bn_cells:y_extent(2);
            x_left_over = x_extent(2) - xlist(size(xlist,2));
            y_left_over = y_extent(2) - ylist(size(ylist,2));
            xlist = xlist + x_left_over/2;
            ylist = ylist + y_left_over/2;
            [obj.grid_x,obj.grid_y] = meshgrid(xlist,ylist);
            obj.obstacle_pos = [obj.obstacle_pos;[xlist',ones(size(xlist',1),1)*y_extent(1)];[xlist',ones(size(xlist',1),1)*y_extent(2)];[ones(size(ylist',1),1)*x_extent(1),ylist'];[ones(size(ylist',1),1)*x_extent(2),ylist']];
        end
        
        function add_obstacle(obj,obs_x_extent,obs_y_extent)
            [xmin,ymin,~,~] = obj.find_cell(obs_x_extent(1),obs_y_extent(1));
            [xmax,ymax,~,~] = obj.find_cell(obs_x_extent(2),obs_y_extent(2));
            
            xlist=xmin:obj.dist_bn_cells:xmax;
            ylist=ymin:obj.dist_bn_cells:ymax;
            
            for i = 1:size(xlist,2)
                for j = 1:size(ylist,2)
                    obj.obstacle_pos = [obj.obstacle_pos; [xlist(i),ylist(j)]];
                end
            end
        end
        
        function [x,y,xind,yind]=find_cell(obj,x,y)
            [~,x] = min(abs(obj.grid_x(1,:)-x));
            [~,y] = min(abs(obj.grid_y(:,1)-y));
            xind = x;
            yind = y;
            x = obj.grid_x(1,x);
            y = obj.grid_y(y,1);
        end
        
        function add_features(obj,features)
            obj.feature_pos = features;
        end
        
        function [cell_list,index_list] = observable_cells(obj,x,y,r)
            cell_list = [];
            index_list = [];
            persistent obstacle_grid;
            if isempty(obstacle_grid)
                obstacle_grid = zeros(size(obj.grid_x));
                for i = 1:size(obj.obstacle_pos,1)
                    [~,~,obs_x,obs_y] = obj.find_cell(obj.obstacle_pos(i,1),obj.obstacle_pos(i,2));
                    obstacle_grid(obs_y,obs_x) = 1;
                end
            end
                
            
            [~,~,xind,yind]=obj.find_cell(x,y);
            n = floor(r/obj.dist_bn_cells);
            persistent grid_x_size;
            persistent grid_y_size;
            
            if isempty(grid_x_size)
                grid_x_size = size(obj.grid_x,2);
                grid_y_size = size(obj.grid_x,1);
            end
            
            persistent gridx;
            persistent gridy;
            
            if isempty(gridx)
                gridx = obj.grid_x;
                gridy = obj.grid_y;
            end
            
            for i = -n:n
                for j = -n:n
                    if xind + j > 0
                        if xind + j <= grid_x_size
                            if yind + i > 0
                                if yind + i <= grid_y_size
                                    if (obj.grid_x(1,xind + j)-x)^2 + (obj.grid_y(yind+i,1)-y)^2 <= r^2
                                        if ~obstacle_grid(yind + i,xind + j)
                                            cell_list = [cell_list;gridx(1,xind + j),gridy(yind + i,1)];
                                            index_list = [index_list;xind+j,yind+i];
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
        end

        function isobs = is_obstacle(obj,x,y)
            persistent obstacle_grid;
            if isempty(obstacle_grid)
                obstacle_grid = zeros(size(obj.grid_x));
                for i = 1:size(obj.obstacle_pos,1)
                    [~,~,obs_x,obs_y] = obj.find_cell(obj.obstacle_pos(i,1),obj.obstacle_pos(i,2));
                    obstacle_grid(obs_y,obs_x) = 1;
                end
            end
            
            if x < obj.x_extent(1) || x > obj.x_extent(2) || y < obj.y_extent(1) || y > obj.y_extent(2)
                isobs = true;
                return;
            else
                [~,~,xind,yind] = obj.find_cell(x,y);
                if size(obj.obstacle_pos,1) > 0
                    isobs = obstacle_grid(yind,xind);
                else
                    isobs = false;
                end
            end
        end
        
%         function nearest_not_obs(obj,x,y)
%             
%         end
    end
end