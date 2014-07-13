function [time_to_detection_list,num_successful_searches_list] = experiment_type(numRuns,type)
    global A;
    global obstacleGrid;
    global alphaList;
    global betaList;
    global m;
    global n;
    global lookahead;

%Experiment Types
% 1 : Different Obstacle Configurations
% 2 : Different Prior Distributions
% 3 : Variations in Alpha (Beta = 0)
% 4 : Variations in Beta (Alpha = 0)
% 4 : Lookahead window (2 - 5)

% Search ID's
% 1 : Optimal Lookahead
% 2 : Random
% 3 : Sweeping
% 4 : Drosophila Inspired
% 5 : Saccadic
    
heads = {'Lookahead','Sweeping','Drosoph','Saccadic'};

if type == 1
    lookahead = 5;
    Aobstacle = zeros(2*m+1,2*n+1,1);
    time_to_detection_list = zeros(numRuns,4*size(obstacleGrid,3));
    num_successful_searches_list = zeros(1,4*size(obstacleGrid,3));
    for i = 1:size(obstacleGrid,3)
        Aobstacle(:,:,i) = A(:,:,1).*obstacleGrid(:,:,i);
        Aobstacle(:,:,i) = Aobstacle(:,:,i)./(sum(sum(Aobstacle(:,:,i))));
    end
    for i = 1:size(obstacleGrid,3)         %For each obstacle configuration
        for j = 1:4                        %For each SearchID
            for k = 1:numRuns
                [time_to_detection,correct_identification] = experiment_sim(0.2,0.2,Aobstacle(:,:,i),j);
                time_to_detection_list(k,4*(i-1)+j)=time_to_detection;
                num_successful_searches_list(4*(i-1)+j) = num_successful_searches_list(i*j) + correct_identification;
                headers{4*(i-1)+j} = strcat(heads{j},num2str(i));
            end
            4*(i-1)+j
        end
    end
    csvwrite_with_headers('obstacle_test.csv',time_to_detection_list,headers);

end

if type == 2
    lookahead = 3;
    time_to_detection_list = zeros(numRuns,4*size(A,3));
    num_successful_searches_list = zeros(1,4*size(A,3));
    for i = 1:size(A,3)     %For each prior
        for j = 1:4         %For each SearchID
            for k = 1:numRuns
                [time_to_detection,correct_identification] = experiment_sim(0.2,0.2,A(:,:,i),j);
                time_to_detection_list(k,4*(i-1)+j)=time_to_detection;
                num_successful_searches_list(4*(i-1)+j) = num_successful_searches_list(4*(i-1)+j) + correct_identification;
                headers{4*(i-1)+j} = strcat(heads{j},num2str(i));
            end
            4*(i-1)+j
        end
    end
    csvwrite_with_headers('prior_test.csv',time_to_detection_list,headers);
end

if type == 3
    lookahead = 3;
    time_to_detection_list = zeros(numRuns,4*size(alphaList,2));
    num_successful_searches_list = zeros(1,4*size(alphaList,2));
    for i = 1:size(alphaList,2)     %For each alpha
        for j = 1:4                 %For each SearchID
            for k = 1:numRuns
                [time_to_detection,correct_identification] = experiment_sim(alphaList(i),0,A(:,:,1),j);
                time_to_detection_list(k,4*(i-1)+j)=time_to_detection;
                num_successful_searches_list(4*(i-1)+j) = num_successful_searches_list(4*(i-1)+j) + correct_identification;
                headers{4*(i-1)+j} = strcat(heads{j},num2str(i));
            end
            4*(i-1)+j
        end
    end
    csvwrite_with_headers('alpha_test.csv',time_to_detection_list,headers);
end

if type == 4
    lookahead = 3;
    time_to_detection_list = zeros(numRuns,4*size(betaList,2));
    num_successful_searches_list = zeros(1,4*size(betaList,2));
    for i = 1:size(betaList,2)     %For each beta
        for j = 1:4 %For each SearchID
            for k = 1:numRuns
                [time_to_detection,correct_identification] = experiment_sim(0,betaList(i),A(:,:,1),j);
                time_to_detection_list(k,4*(i-1)+j)=time_to_detection;
                num_successful_searches_list(4*(i-1)+j) = num_successful_searches_list(4*(i-1)+j) + correct_identification;
                headers{4*(i-1)+j} = strcat(heads{j},num2str(i));
            end
            4*(i-1)+j
        end
    end
    csvwrite_with_headers('beta_test.csv',time_to_detection_list,headers);
end

if type == 5
    time_to_detection_list = zeros(numRuns,4);
    num_successful_searches_list = zeros(1,4);
    for i = 2:5
        lookahead = i;
        j = 1;
        for k = 1:numRuns
            [time_to_detection,correct_identification] = experiment_sim(0.2,0.2,A(:,:,1),j);
            time_to_detection_list(k,i-1) = time_to_detection;
            num_successful_searches_list(i-1) = num_successful_searches_list(i-1) + correct_identification;
            headers{i-1} = strcat('Window-',num2str(i));
        end
        i
    end
    csvwrite_with_headers('lookahead_window_test.csv',time_to_detection_list,headers);
end
end