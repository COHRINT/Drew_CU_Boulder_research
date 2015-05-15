function optimal_path = optimal_lookahead(paths,space,search,radius_of_detection,window,r,n)

persistent obstacle_grid;
if isempty(obstacle_grid)
    obstacle_grid = zeros(size(space.grid_x));
    for i = 1:size(space.obstacle_pos,1)
        [~,~,obs_x,obs_y] = space.find_cell(space.obstacle_pos(i,1),space.obstacle_pos(i,2));
        obstacle_grid(obs_y,obs_x) = 1;
    end
end

for i = 1:window
    paths = expand(paths,r,n);
end

utility_list = zeros(size(paths,3),1);
prob_obs = search.obs_prob(space,radius_of_detection);

for i = 1:size(paths,3)
    for j = 2:size(paths,1)
        [~,~,xind,yind] = space.find_cell(paths(j,1,i),paths(j,2,i));
        if ~obstacle_grid(yind,xind)
            utility_list(i) = utility_list(i) + prob_obs(yind,xind);
        else
            utility_list(i) = 0;
            break;
        end
    end
end

max_ind = find(utility_list == max(utility_list(:)));
r = randperm(size(max_ind,1),1);
max_ind = max_ind(r);

optimal_path = paths(:,:,max_ind);
%optimal_path(1,:,:) = [];
optimal_path = optimal_path(2,:,:);
end

function theta_list = theta_gen(n)
dtheta = 2*pi / n;
theta_list =0:dtheta:2*pi;
theta_list(n+1) = [];
end

function paths = expand(paths,r,n)
new_nodes = [];
new_paths = [];
for i = 1:size(paths,3)
    x = paths(size(paths,1),1,i);
    y = paths(size(paths,1),2,i);
    theta_list = theta_gen(n);
    for j = 1:n
        new_nodes = [new_nodes;x + r*cos(theta_list(j)),y  + r*sin(theta_list(j))];
    end
    new_paths = cat(3,new_paths,generate_path(paths(:,:,i),new_nodes));
    new_nodes = [];
end

paths = new_paths;
end

function new_path = generate_path(path,expansion_nodes)
for i = 1:size(expansion_nodes)
    temp_path = [path;expansion_nodes(i,:)];
    new_path(:,:,i) = temp_path;
end
end