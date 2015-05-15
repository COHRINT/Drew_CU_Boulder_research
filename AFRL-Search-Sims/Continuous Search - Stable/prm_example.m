clear
clc
clf
%%
space = space([0,10],[0,10],2000);
space.add_obstacle([2,4],[3,4.75]);
space.add_obstacle([6,8],[3,4.75]);
space.add_obstacle([2,4],[5.75,8]);
space.add_obstacle([6,8],[5.75,8]);

x_bounds = space.x_extent;
y_bounds = space.y_extent;
prm_points = [];
n = 1000;
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
%triplot(tri), hold on
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
x = 1;
y = 1;
x_goal = 9;
y_goal = 9;
start = find_nearest_graph_cell(x,y,prm_points)
goal = find_nearest_graph_cell(x_goal,y_goal,prm_points)
%start = 900;
%goal = 52;
triplot(tri),hold on

for i = 1:size(e_new,1)
    plot([prm_points(e_new(i,1),1),prm_points(e_new(i,2),1)],[prm_points(e_new(i,1),2),prm_points(e_new(i,2),2)]), hold on
end

scatter([prm_points(start,1),prm_points(goal,1)],[prm_points(start,2),prm_points(goal,2)],100,'r','fill')
%%
path = A_star(size(prm_points,1)-size(space.obstacle_pos,1),start,goal,e_new,prm_points)
%%




scatter(prm_points(n+1:size(prm_points,1),1),prm_points(n+1:size(prm_points,1),2),'k','fill'),hold on



for i = 1:size(path,1)
    scatter(prm_points(path(i),1),prm_points(path(i),2),100,'g','fill'),hold on
end


scatter([prm_points(start,1),prm_points(goal,1)],[prm_points(start,2),prm_points(goal,2)],100,'r','fill'),hold on

