clear all
clc
close all

goal_history = csvread('goal_history.csv');

goal_max = max(goal_history);
goal_min = min(goal_history);

goals = goal_min:goal_max;

goal_transition_count = zeros(goal_max);

for i = 1:size(goal_history,1)-1
    goal_transition_count(goal_history(i),goal_history(i+1)) = goal_transition_count(goal_history(i),goal_history(i+1)) + 1;
end

mu_list = 0:.1:1;
p_max = 0;

alpha_temp = ones(size(goal_transition_count))*1;

alpha_list = goal_transition_count + alpha_temp;
mu_E = zeros(size(alpha_list));
for i = 1:size(alpha_list,1)
    for j = 1:size(alpha_list,2)
        mu_E(i,j) = alpha_list(i,j)/sum(alpha_list(i,:));
    end
end
mu_E

mu_mode = (goal_transition_count);

for i = 1:size(mu_mode,1)
    mu_mode(i,:) = mu_mode(i,:)/(sum(alpha_list(i,:)) - size(alpha_list,1));
end
mu_mode

alpha_hat = sum(alpha_list')';

var = zeros(size(alpha_list));

for i = 1:size(alpha_list,1)
    for j = 1:size(alpha_list,2)
        var(i,j) = alpha_list(i,j)*(alpha_hat(i) - alpha_list(i,j))/(alpha_hat(i)^2*(alpha_hat(i)+1));
    end
end

var.^.5