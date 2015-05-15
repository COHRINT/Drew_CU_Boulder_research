%%
clear all
clc
close all

% The function we'll use is complicated enough that we may not be able to
% model it. We'll choose f(x) = 2*sin(3*x) + x^2


x1 = 0:.5:7;
x2 = 9:.3:14.;

x = [x1';x2'];

x_truth = 0:.03789:20;
truth = 2.*sin(3.*x_truth) + .1*x_truth.^2;

sigma_noise = .5;
data = 2.*sin(3.*x) + .1*x.^2 + sigma_noise*randn(length(x),1);

% figure
% scatter(x,data),hold on
% 
% p1 = polyfit(x,data,1);
% plot(x,p1(1)*x+p1(2))
% 
% p2 = polyfit(x,data,2);
% plot(x,p2(1)*x.^2 + p2(2).*x+p2(3))
% 
% p3 = polyfit(x,data,3);
% plot(x,p3(1)*x.^3 + p3(2).*x.^2+p3(3).*x + p3(4))
% 
% plot(x_truth,truth)
%%
%clf
plot(x_truth,truth),hold on
scatter(x,data)
%%
n = 2;

sigma_f = 10;
sigma_n = 1;
l = 1;
K = K_mat(x,sigma_f,l,sigma_n,n);

est_mean=[];
est_cov=[];
x_truth = x_truth';

for i = 1:size(x_truth,1)
    K_star = K_star_calc(x,x_truth(i),sigma_f,l,sigma_n,n);
    est_mean = [est_mean;K_star*inv(K)*data];
    est_cov = [est_cov;covar_func(x_truth(i),x_truth(i),sigma_f,l,sigma_n,n)-...
        K_star*inv(K)*K_star'];
end
figure
plot(x_truth,[est_mean,est_mean+sqrt(est_cov),est_mean-sqrt(est_cov)]),hold on
scatter(x,data)
%plot(x_truth,truth)

figure
plot(x_truth,[est_mean - truth',sqrt(est_cov),-sqrt(est_cov)])

% %%
% 
% data =  csvread('apple_data.csv');
% x = 1:size(data,1);
% x_truth = x';
% 
% x_truth = x_truth(end-1000:end,1);
% data = data(end-1000:end,1);
% 
% n = 1;
% 
% sigma_f = 100;
% sigma_n = 1;
% l = 5;
% K = K_mat(x_truth,sigma_f,l,sigma_n,n);
% 
% est_mean=[];
% est_cov=[];
% 
% K_inv = inv(K);
% 
% for i = 1:size(x_truth,1)
%     K_star = K_star_calc(x_truth,x_truth(i)+.00001,sigma_f,l,sigma_n,n);
%     est_mean = [est_mean;K_star*K_inv*data];
%     est_cov = [est_cov;covar_func(x_truth(i)+.00001,x_truth(i)+.00001,sigma_f,l,sigma_n,n)-...
%         K_star*inv(K)*K_star'];
% end
% figure
% plot(x_truth,[est_mean,est_mean+1.96*sqrt(est_cov),est_mean-1.96*sqrt(est_cov)]),hold on
% scatter(x_truth,data)
% %plot(x_truth,truth)
% 

