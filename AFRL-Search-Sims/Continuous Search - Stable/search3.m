classdef search3 < handle
    properties
        A
    end
    methods
        function obj = search3(space)
            obj.A = zeros(size(space.grid_x));
        end
        
        function assign_prior(obj,prior,space)
            obj.A = prior;
            obstacles = ones(size(space.grid_x));
            
            if ~isempty(space.obstacle_pos)
                for i = 1:size(space.obstacle_pos,1)
                    [~,~,xind,yind] = space.find_cell(space.obstacle_pos(i,1),space.obstacle_pos(i,2));
                    obstacles(yind,xind)=0;
                end
            end
            
            obj.A = obj.A .* obstacles;
            obj.A = obj.A/sum(sum(obj.A));
            obj.A(obj.A==0) = nan;
            
        end
        
        function prob_obs = obs_prob(obj,space,r)
            A = obj.A;
            A(isnan(A))= 0;
            persistent obs_matrix;
            
            if isempty(obs_matrix)
                obs_matrix = zeros(size(space.grid_x,1)*size(space.grid_x,2));
                for i = 1:size(space.grid_x,2)
                    for j = 1:size(space.grid_y,1)
                        [~,index_list] = space.observable_cells(space.grid_x(1,i),space.grid_y(j,1),r);
                        for k = 1:size(index_list,1)
                            obs_matrix(sub2ind(size(space.grid_x),index_list(k,1),index_list(k,2)),sub2ind(size(space.grid_x),i,j)) = 1;
                        end
                    end
                end
            end
            
            prob_obs = reshape(obs_matrix * reshape(A,size(space.grid_x,1)*size(space.grid_x,2),1),size(space.grid_x,1),size(space.grid_x,2));
        end
        
        function d = takeMeasurement(obj,agent_truth,target_truth,pos_estimate,space)
            xtx = target_truth.x_pos;
            xty = target_truth.y_pos;
            xt = agent_truth.x_pos;
            yt = agent_truth.y_pos;
            r = agent_truth.radius_of_detection;
            alpha = agent_truth.alpha;
            beta = agent_truth.beta;
            
            %===================SIMULATE TAKING A MEASUREMENT=============
            [observed_cells,index_list] = space.observable_cells(xt,yt,r);
            
            [xtx,xty,~,~] = space.find_cell(xtx,xty);
            target_in_zone = ismember(round([xtx,xty]*1000)/1000,round(observed_cells*1000)/1000,'rows');
            rd = rand;
            if ~target_in_zone
                if (rd<alpha)
                    d = 1;
                else
                    d = 0;
                end
            else
                if (rd<beta)
                    d = 0;
                else
                    d = 1;
                end
            end

        end
        
        function planned_path = update_prior(obj,space,agent_truth,target_truth,pos_estimate,planned_path)
            mu = pos_estimate.state(1:2)';
            covar = pos_estimate.P(1:2,1:2);
            alpha = agent_truth.alpha;
            beta = agent_truth.beta;
            r = agent_truth.radius_of_detection;
            d = obj.takeMeasurement(agent_truth,target_truth,pos_estimate,space);
            A_temp = obj.A;
            A_new = zeros(size(space.grid_x));
            p_in_sight = [];
            p_pos = [];
            prob_of_pos = [];
            [mu(1),mu(2)] = space.find_cell(mu(1),mu(2));
            x_pos_max = (mu(1) + 3 * sqrt(covar(1)));
            x_pos_min = (mu(1) - 3 * sqrt(covar(1)));
            y_pos_max = (mu(2) + 3 * sqrt(covar(4)));
            y_pos_min = (mu(2) - 3 * sqrt(covar(4)));
            
            if d == 0
                term3 = beta;
                term5 = 1 - alpha;
                term1observed = beta;
                term1notobserved = 1 - alpha;
            elseif d == 1
                term3 = 1 - beta;
                term5 = alpha;
                term1observed = 1 - beta;
                term1notobserved = alpha;
                planned_path = [];
            else
                term3 = beta;
                term5 = 1 - alpha;
                term1observed = beta;
                term1notobserved = 1 - alpha;
            end
            
            obs_prob_matrix = obj.obs_prob(space,r);
            
            [~,~,x_ind,y_ind] = space.find_cell(mu(1),mu(2));
            
            for i = 1:size(space.grid_x,2)
                for j = 1:size(space.grid_x,1)
                    x = space.grid_x(1,i);
                    y = space.grid_y(j,1);
                    
                    if x <= x_pos_max && x >= x_pos_min && y >= y_pos_min && y <= y_pos_max
                        p_pos = [p_pos;i,j];
                        prob_of_pos = [prob_of_pos;mvnpdf([x,y],mu,covar)];
                    end
                    
                    if x < x_pos_max + r && x > x_pos_min - r && y > y_pos_min - r && y < y_pos_max + r
                        p_in_sight = [p_in_sight;i,j];
                    else
                        p_current = obs_prob_matrix(y_ind,x_ind);
                        A_new(j,i) = term1notobserved * A_temp(j,i)/(term3 * p_current+term5*(1-p_current));
                    end
                end
            end
            
            prob_of_pos = prob_of_pos./sum(prob_of_pos);
            term1_matrix = term1notobserved * ones(size(space.grid_x,1)*size(space.grid_x,2),size(p_pos,1));

            for i = 1:size(p_pos,1)
                [~,cell_list] = space.observable_cells(space.grid_x(1,p_pos(i,1)),space.grid_y(p_pos(i,2),1),r);

                for j = 1:size(cell_list,1)
                    term1_matrix(sub2ind(size(space.grid_x),cell_list(j,1),cell_list(j,2)),i) = term1observed;
                end
            end
            
            for i = 1:size(p_in_sight,1)
                for j =1:size(p_pos,1)
                    A_new(p_in_sight(i,2),p_in_sight(i,1)) = A_new(p_in_sight(i,2),p_in_sight(i,1)) + prob_of_pos(j)*term1_matrix(sub2ind(size(space.grid_x),p_in_sight(i,1),p_in_sight(i,2)),j) * A_temp(p_in_sight(i,2),p_in_sight(i,1))/(term3*obs_prob_matrix(p_pos(j,2),p_pos(j,1)) + term5*(1-obs_prob_matrix(p_pos(j,2),p_pos(j,1))));
                end
            end   
            obj.A = A_new./sum(sum(A_new(~isnan(A_new))));
        end
    end
end