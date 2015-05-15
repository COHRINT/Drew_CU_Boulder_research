clear
clc
figure(1)
clf
figure(2)
clf

space = space([0,10],[0,10],2000);
A = mvnpdf([space.grid_x(:) space.grid_y(:)],[5 5],[2 0;0 2]);
A = reshape(A,length(space.grid_y),length(space.grid_x));
xt = 5;
yt = 5;

figure(1)
pcolor(space.grid_x,space.grid_y,A);
set(gca,'color',[0 0 0]),hold on;
hag = plot3(xt,yt,0.02,'wo','MarkerSize',12,'LineWidth',2);  

w = [-2;-1.5;-.75];
b = [5, 4.5, 0];
range_grid = sqrt((space.grid_x - xt).^2 + (space.grid_y - yt).^2);

g1 = exp(w(1)*range_grid + b(1));
g2 = exp(w(2)*range_grid + b(2));
g3 = exp(w(3)*range_grid + b(3));

gsum = g1 + g2 + g3;
g1 = g1./gsum;
g2 = g2./gsum;
g3 = g3./gsum;

figure(2)
pcolor(g1);colorbar

figure(3)
pcolor(g2);colorbar

figure(4)
pcolor(g3);colorbar

% surf(g1), hold on
% surf(g2), hold on
% surf(g3)
