function path = A_star(A,start,goal)

[m,n] = size(A);     

startx = start(1);
starty = start(2);

closedset = zeros(m,n);
openset = nan(m,n);
came_fromx = nan(m,n);
came_fromy = nan(m,n);

openset(starty,startx) = 1;

g_score = nan(m,n);
f_score = nan(m,n);

g_score(starty,startx) = 0;
f_score(starty,startx) = g_score(starty,startx) + heuristic_cost_estimate(start,goal); 

while sum(sum(openset)) ~= 0
    [currenty , currentx] = find(f_score.*openset == min(min(f_score.*openset)));
    currentx = currentx(1);
    currenty = currenty(1);
    current = [currentx,currenty];
    
    if current == goal
        path = reconstruct_path(came_fromx, came_fromy, currentx,currenty);
        return;
    end
    
    openset(currenty,currentx) = nan;
    closedset(currenty,currentx) = 1;
    
    for i = -1:1
        for j = -1:1
            if ~(i == 0 && j == 0) &&  currentx + j > 0 && currenty + i > 0 && currentx + j <= n && currenty + i <= m && ~isnan(A(currenty + i,currentx + j))
                neighbory = currenty + i;
                neighborx = currentx + j;
                neighbor = [neighborx,neighbory];
                
                if closedset(neighbory, neighborx) == 1
                    continue
                end
                
                tentative_g_score = g_score(currenty,currentx) + 1;
                
                if isnan(openset(currenty,currentx)) || tentative_g_score <  g_score(neighbory, neighborx)
                    came_fromx(neighbory,neighborx) = currentx;
                    came_fromy(neighbory,neighborx) = currenty;
                    
                    g_score(neighbory,neighborx) = tentative_g_score;
                    f_score(neighbory,neighborx) = g_score(neighbory,neighborx) + heuristic_cost_estimate(neighbor,goal);
                    if isnan(openset(neighbory,neighborx));
                        openset(neighbory,neighborx) = 1;
                    end
                end
            end
        end
    end
end
    path = [nan,nan];
end

function path = reconstruct_path(came_fromx, came_fromy, currentx,currenty)
if ~isnan(came_fromx(currenty,currentx)) && ~isnan(came_fromy(currenty,currentx))
    p = reconstruct_path(came_fromx, came_fromy,came_fromx(currenty,currentx),came_fromy(currenty,currentx));
    path = [p;[currentx,currenty]];
    return
else
    path = [currentx,currenty];
    return
end
end
    
function h = heuristic_cost_estimate(start,goal)
h = sqrt((goal(1)-start(1))^2 + (goal(2)-start(2))^2);    
end