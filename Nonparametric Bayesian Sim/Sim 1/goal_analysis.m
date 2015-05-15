goal_history = csvread('goal_history.csv');

goal_max = max(goal_history);
goal_min = min(goal_history);

goals = goal_min:goal_max;

goal_transition_count = zeros(goal_max);

for i = 1:size(goal_history,1)-1
    goal_transition_count(goal_history(i),goal_history(i+1)) = goal_transition_count(goal_history(i),goal_history(i+1)) + 1;
end

for i = 1:size(goal_transition_count,1)
    sum_temp = sum(goal_transition_count(i,:));
    goal_transition_count(i,:) = goal_transition_count(i,:)./sum_temp;
end



goal_HMM_est = goal_transition_count