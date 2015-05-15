function create_map(x_size,y_size,filename)

x = 0:x_size;
y = 0:y_size;

[X,Y] = meshgrid(x,y);

occupancy_grid = zeros(size(X));

figure
imagesc(~occupancy_grid)
axis([0 x_size 0 y_size])
colormap('gray')

obj_point_temp = [-10 -10];
obj_points = [-10 -10];
new_obj_flag = 1;
while ~isempty(obj_points)
    while ~isempty(obj_point_temp)
        if new_obj_flag == 1
            obj_points = [];
            new_obj_flag = 0;
        end
        obj_point_temp = ginput(1);
        obj_points = [obj_points;obj_point_temp];
        hold on
        if ~isempty(obj_points)
            plot(obj_points(:,1),obj_points(:,2),'r')
        end
    end
    if ~isempty(obj_points)
        for i = 1:size(x,2)
            for j = 1:size(y,2)
                if inpolygon(x(i),y(j),obj_points(:,1),obj_points(:,2))
                    occupancy_grid(j,i) = 1;
                    clf
                    imagesc(~occupancy_grid),hold on
                    axis([0 x_size 0 y_size])
                    colormap('gray')
                end
            end
        end
    end
    new_obj_flag = 1;
    obj_point_temp = [-10,-10];
end
close all

csvwrite(filename,occupancy_grid)

end