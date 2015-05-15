function path = A_star(n,start,goal,edge_list,prm_points)
% Initialize set values
closedset = zeros(n,1);
openset = nan*ones(n,1);
came_from = nan*ones(n,1);

openset(start) = 1;
g_score = nan*ones(n,1);
f_score = nan*ones(n,1);

g_score(start) = 0;
f_score(start) = g_score(start) + heuristic_cost_estimate(start,goal,prm_points);

% Iterate through list of map points
while sum(openset(~isnan(openset))) ~= 0
    % Expand search based on which map point has currently lowest f score
    current = find(f_score == min(f_score.*openset));
    for i = 1:size(current,1)
        % If goal has been reached, construct the path taken
        if current(i,1) == goal
            %disp('here!')
            path = reconstruct_path(came_from,current(i,1));
            return;
        end
    end
    current = current(1);
    %prm_points(current,:) %Show current cell here
    
    %scatter(prm_points(current,1),prm_points(current,2),50,'y','fill'),hold on
    
    % Remove currently examined point from list of unexamined points
    openset(current) = nan;
    % Add to list of examined points
    closedset(current) = 1;
    
    for i = 1:size(edge_list,1)
        % Find other points connected to current point
        if edge_list(i,1) == current
            %disp('looking at neighbor')
            neighbor = edge_list(i,2);
            % Only expand if point hasn't been examined yet
            if ~closedset(neighbor)
                % Calculate tentative g score
                tentative_g_score = g_score(current) + edge_list(i,3);
                % If neighbor has been examined or g score less than
                % current value, re-evaluate g score and f score
                if isnan(openset(neighbor)) || tentative_g_score <= g_score(neighbor)
                    
                    came_from(neighbor) = current;
                    g_score(neighbor) = tentative_g_score;
                    f_score(neighbor) = g_score(neighbor) + heuristic_cost_estimate(neighbor,goal,prm_points);
                    %if isnan(openset(neighbor))
                        openset(neighbor) = 1;
                    %end
                else
                    
                end
            end
        end
    end
end
% If no path found, return NaN
path = [nan,nan];
end

function path = reconstruct_path(came_from, current)
if ~isnan(came_from(current))
    p = reconstruct_path(came_from,came_from(current));
    path = [p;current];
    return
else
    path = current;
    return
end
end

function h = heuristic_cost_estimate(start,goal,prm_points)
goal = prm_points(goal,:);
start = prm_points(start,:);
h = sqrt((goal(1)-start(1))^2 + (goal(2)-start(2))^2);
end