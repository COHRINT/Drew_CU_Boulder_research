function [edges,points,path_pos] = drosoph_search(current_pos,space,search)

persistent edge_list;
persistent prm_points;
n = 500;
if isempty(edge_list)
    [edge_list ,prm_points]= generate_graph(space,n);
end

goal_pos = find_max(current_pos,space,search);
goal_id = find_nearest_graph_cell(goal_pos(2),goal_pos(1),prm_points(1:n,:));
start_id = find_nearest_graph_cell(current_pos(1),current_pos(2),prm_points(1:n,:));

path = A_star(n,start_id,goal_id,edge_list,prm_points);

for i = 1:size(path,1)
    path_pos(i,:) = prm_points(path(i),:);
end
edges = edge_list;
points = prm_points;
end

function max_pos = find_max(current_pos,space,search)
A = search.A;
% [max_posx,max_posy] = find(A == max(A(:)));
% max_pos = [max_posx(1),max_posy(1)];

x_diff = space.grid_x - current_pos(1)*ones(size(space.grid_x));
y_diff = space.grid_y - current_pos(2)*ones(size(space.grid_x));

dist_matrix = (x_diff.^2 + y_diff.^2).^.5;

util = 100.*A - 0.23 * dist_matrix;
[max_posx,max_posy] = find(util == max(util(:)));
max_pos = [max_posx(1),max_posy(1)];

max_pos = [space.grid_x(1,max_pos(1)),space.grid_y(max_pos(2),1)];
end

% function cell_id = find_nearest_graph_cell(x,y,prm_points)
% min_dist = 1000;
% for i = 1:size(prm_points,1)
%     dist = (prm_points(i,1) - x)^2 + (prm_points(i,2) - y)^2;
%     if  dist < min_dist
%         cell_id = i;
%         min_dist = dist;
%     end
% end
% end

function [e_new ,prm_points] = generate_graph(space,n)
prm_points = [];

x_bounds = space.x_extent;
y_bounds = space.y_extent;

count = 0;
while count < n
    x = rand*(x_bounds(2) - x_bounds(1)) + x_bounds(1);
    y = rand*(y_bounds(2) - y_bounds(1)) + y_bounds(1);
    if ~space.is_obstacle(x,y)
        prm_points = [prm_points;x y];
        count = count + 1;
    end
end


prm_points = [prm_points;space.obstacle_pos];

tri = delaunayTriangulation(prm_points(:,1),prm_points(:,2));
%triplot(tri)
%scatter(prm_points(:,1),prm_points(:,2),'ro')

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
end