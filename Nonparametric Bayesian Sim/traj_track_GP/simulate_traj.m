function traj = simulate_traj()

umap = markov_control();

x = [5,5,0]';
dt = .1;

t = 1:dt:100;

traj = [];
u_hist = [];

for i = 1:length(t)
    map_pos = round(x);
    u = umap(map_pos(1),map_pos(2),:);
    vx = u(2)*cos(x(3));
    vy = u(2)*sin(x(3));
    x = x + [vx;vy;u(1)]*dt;
    traj = [traj;x'+randn(size(x')).*[.04,.04,.01]];
    u_hist = [u_hist;u];
end


end
