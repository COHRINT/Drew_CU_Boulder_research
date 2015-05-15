clear all
clc
close all

x = [10,10,0,0,0,0]';

F = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0; 0 0 0 0 0 0];

u = [5,.1];
x_list = [];
y_list = [];
dt = .1;
for i = 1:10000
    x_list = [x_list;x'];
    y = x + randn(6,1).*[.5;.5;.05;.1;.1;.05];
    y_list = [y_list;y'];
    x = F*x + [u(1)*cos(x(3))*dt;u(1)*sin(x(3))*dt;u(2)*dt;u(1)*cos(x(3));...
        u(1)*cos(x(3));u(2)];%+randn(6,1).*[.01;.01;.005;.01;.01;.005];
    
end


plot(x_list(:,1),x_list(:,2)),hold on
plot(y_list(:,1),y_list(:,2))
