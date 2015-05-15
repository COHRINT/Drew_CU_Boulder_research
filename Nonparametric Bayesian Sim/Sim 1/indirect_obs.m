data = csvread('goal_history.csv');

detector_HMM = rand(5,10);
k = 0;
detector_hist = zeros(size(data,1),10);
for i = 1:size(data,1)
    for j = 1:10
        r = rand;
        if r < detector_HMM(data(i),j)
            detector_hist(i,j) = 1;
        end
    end
end

state_count = zeros(5,1);
for i = 1:size(data,1)
    state_count(data(i)) = state_count(data(i)) + 1;
end

detector_count = zeros(5,10);
for i = 1:size(detector_hist,1)
    for j = 1:10
        detector_count(data(i),j) = detector_count(data(i),j) + detector_hist(i,j);
    end
end

for i = 1:size(detector_count,1)
    detector_count(i,:) = detector_count(i,:)/state_count(i);
end

detector_HMM
detector_HMM_est = detector_count
