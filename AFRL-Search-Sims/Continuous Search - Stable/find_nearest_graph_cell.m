function cell_id = find_nearest_graph_cell(x,y,prm_points)
min_dist = 100000;
for i = 1:size(prm_points,1)
    dist = (prm_points(i,1) - x)^2 + (prm_points(i,2) - y)^2;
    if  dist < min_dist
        cell_id = i;
        min_dist = dist;
    end
end
end