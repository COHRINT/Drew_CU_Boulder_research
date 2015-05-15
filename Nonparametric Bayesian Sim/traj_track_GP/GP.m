function [x_data,y_data,GP_data] = GP(x_list,y_list,l,sigma_f,sigma_n)

dt = .1; 

traj = simulate_traj();
diff_pos = diff([traj(:,1),traj(:,2)]);

vel_est = sqrt(diff_pos(:,1).^2 + diff_pos(:,2).^2)./dt;

data = [traj(2:end,1:2),vel_est];

K = K_mat(data(:,1:2),sigma_f,l,sigma_n);

[X,Y] = meshgrid(x_list,y_list);

est_mean=zeros(size(X));
est_cov=zeros(size(X));
for i = 1:size(X,1)
    for j = 1:size(X,2)
        x_star = [X(i,j),Y(i,j)];
        K_star = K_star_calc(data(:,1:2),x_star,sigma_f,l,sigma_n);
        est_mean(i,j) = K_star*inv(K)*data(:,3);
%         est_cov = [est_cov;covar_func(x_truth(i),x_truth(i),sigma_f,l,sigma_n,n)-...
%             K_star*inv(K)*K_star'];
    end
end

x_data = X;
y_data = Y;
GP_data = est_mean;

end