function [command_udot,command_vdot] = controller(command_pos,pos_estimate,space)

x_command = command_pos(1);
y_command = command_pos(2);

x = pos_estimate.state(1);
y = pos_estimate.state(2);
u = pos_estimate.state(3);
v = pos_estimate.state(4);
psi = pos_estimate.state(5);

vx = u*cos(psi) - v*sin(psi);
vy = -u*sin(psi) - v*cos(psi);


v_max = 1.0;
a_max = 2.0;
pos_gain = .5;
vel_gain = 1;

del_x = x_command - x;
del_y = y_command - y;

command_vx = pos_gain*del_x;
command_vy = pos_gain*del_y;

const = v_max/sqrt(command_vx^2+command_vy^2);
if sqrt(command_vx^2+command_vy^2) > v_max
    command_vx = const*command_vx;
    command_vy = const*command_vy;
end

del_vx = command_vx - vx;
del_vy = command_vy - vy;

command_ax = vel_gain*del_vx;
command_ay = vel_gain*del_vy;


[del_ax,del_ay] = repulsion_field(space,x,y);
command_ax = command_ax + del_ax;
command_ay = command_ay + del_ay;


const = a_max/sqrt(command_ax^2+command_ay^2);
if sqrt(command_ax^2+command_ay^2) > a_max
    command_ax = const*command_ax;
    command_ay = const*command_ay;
end

command_udot = command_ax *cos(psi) + command_ay * sin(psi);
command_vdot = command_ax *sin(psi) - command_ay * cos(psi);

end

function [del_ax,del_ay] = repulsion_field(space,x,y)
R = .3;
C = 8;
k= 30;
persistent obstacles;
if isempty(obstacles)
    obstacles = space.obstacle_pos;
end

del_ax = 0;
del_ay = 0;
for i = 1:size(obstacles,1)
    x_obs = obstacles(i,1);
    if abs(x_obs - x) < R
        y_obs = obstacles(i,2);
        if abs(y_obs - y) < R
            r = (x_obs-x)^2 + (y_obs - y)^2;
            if r < R^2
                del_ax = del_ax + C*exp(-k*r) * (x - x_obs)/r;
                del_ay = del_ay + C*exp(-k*r) * (y - y_obs)/r;
            end
        end
    end
end
end


