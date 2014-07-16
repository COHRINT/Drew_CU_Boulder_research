%   Author: Drew Ellison
%   Email: dme722@gmail.com
%   File Description:
%   The dependence of each of the search algorithms present in the
%   simulate_search function on obstacles, prior distribution, and
%   variations in alpha and beta (detector characteristics) is tested in
%   the experiment set up by this function. For now, only 2 detector
%   characterizations are considered. alpha = 0.3/beta = 0.0 and
%   alpha =0.0/beta =0.3.
function [time_to_detection_list,num_successful_searches_list] = experiment(numRuns)

%Subscribe to global values
global Xg;
global Yg;
global m;
global n;
global obstacleGrid;

%Set up alpha and beta configurations to be used
alpha_beta = [0.3 0.0;0.0 0.3];

%Gaussian Superposition Setup (Will be used for obstacle setup)
amp = [0 0 0 0 0 0 0];
sigmax = [1.5 1.5 3 2 1 3 2];
sigmay = [1 1 1 2 .5 3 2];
mux = [-8 -9 -12 13 13 1 13];
muy = [7 -3 -12 -12 -3 13 13];
digits(1000);

%Open Map Setup
mu = [-3 4];
sigma = [12 12;5 5];
hedge = .1;
Aopen(:,:,1) = exp(-((Xg-mu(1)).^2/(2*sigma(1,1)^2)+(Yg-mu(2)).^2/(2*sigma(1,2))^2));
Aopen(:,:,2) = exp(-((Xg-mu(1)).^2/(2*sigma(2,1)^2)+(Yg-mu(2)).^2/(2*sigma(2,2))^2));
Aopen(:,:,3) = (heaviside(Xg -(mu(1)-sigma(1,1))) - heaviside(Xg +(mu(1)-sigma(1,1))))*(heaviside(Xg -(mu(2)-sigma(1,2))) - heaviside(Xg +(mu(2)-sigma(1,2)))) + hedge*ones(2*m+1,2*n+1);
Aopen(:,:,4) = (heaviside(Xg -(mu(1)-sigma(2,1))) - heaviside(Xg +(mu(1)-sigma(2,1))))*(heaviside(Xg -(mu(2)-sigma(2,2))) - heaviside(Xg +(mu(2)-sigma(2,2)))) + hedge*ones(2*m+1,2*n+1);

%Initialize succesful searches list
num_successful_searches_list = zeros(1,4*2*2*4);

%Initialize header strings for CSV file
searchHeads = {'Lookahead','Sweeping','Drosoph','Saccadic'};
obstacleHeads = {'Obstacles','Open'};
alpha_betaHeads = {'a=.3_b=0','a=0_b=.3'};
priorHeads = {'Prior1','Prior2','Prior3','Prior4'};

%Initialize count for diagnostic of current experiment status
count = 0;

for search = 1:4                                                            %For each search ID
    for i = 1:2                                                             %For each obstacle config (obstacles(1) and no obstacles (2))
        for j = 1:2                                                         %For each alpha/beta config (alpha = 0.3 beta = 0.0 and alpha = 0.0 beta = 0.3)
            for k = 1:4                                                     %For each type of prior
                for h = 1:numRuns
                    if(i == 1)                                              %Set up prior
                        x = randperm(7);                                    %Randomly choose where the target is and choose how many peaks are present based on current simulation
                        for g = 1:k
                            amp(x(g)) = 1;
                        end
                        %Assign prior to grid
                        Acurrent = double(amp(1)*exp(-((Xg-mux(1)).^2/(2*sigmax(1)^2)+(Yg-muy(1)).^2/(2*sigmay(1))^2))+amp(2)*exp(-((Xg-mux(2)).^2/(2*sigmax(2)^2)+(Yg-muy(2)).^2/(2*sigmay(2))^2))+amp(3)*exp(-((Xg-mux(3)).^2/(2*sigmax(3)^2)+(Yg-muy(3)).^2/(2*sigmay(3))^2))+amp(4)*exp(-((Xg-mux(4)).^2/(2*sigmax(4)^2)+(Yg-muy(4)).^2/(2*sigmay(4))^2))+amp(5)*exp(-((Xg-mux(5)).^2/(2*sigmax(5)^2)+(Yg-muy(5)).^2/(2*sigmay(5))^2))+amp(6)*exp(-((Xg-mux(6)).^2/(2*sigmax(6)^2)+(Yg-muy(6)).^2/(2*sigmay(6))^2))+amp(7)*exp(-((Xg-mux(7)).^2/(2*sigmax(7)^2)+(Yg-muy(7)).^2/(2*sigmay(7))^2)));
                    else
                        Acurrent = Aopen(:,:,k);
                    end
                    Acurrent = (Acurrent.*obstacleGrid(:,:,i))./(sum(sum(Acurrent)));  %Renormalize
                    [xtx,xty] = deployTarget(Acurrent);                     %Deploy target
                    [xt,yt]   = deployAgent(Acurrent);                      %Deploy agent
                    [time_to_detection,correct_identification] = simulate_search(alpha_beta(j,1),alpha_beta(j,2),Acurrent,search,xtx,xty,xt,yt);%Run simulation
                    time_to_detection_list(h,sub2ind([4,2,2,4],search,i,j,k))=time_to_detection;%Update matrix for CSV file
                    num_successful_searches_list(sub2ind([4,2,2,4],search,i,j,k)) = num_successful_searches_list(sub2ind([4,2,2,4],search,i,j,k)) + correct_identification;
                    headers{sub2ind([4,2,2,4],search,i,j,k)} = strcat(searchHeads{search},obstacleHeads{i},alpha_betaHeads{j},priorHeads{k});%Update header list for CSV file
                    count = count + 1; %Update count
                    currentPercentage = num2str(100*count/(4*2*2*4*numRuns),3);
                    currentDisplayMessage = strcat('Progress:\t',currentPercentage,'%%');
                    disp(sprintf(currentDisplayMessage));                   %Display percentage of experiment finished
                end
                amp = [0 0 0 0 0 0 0];                                      %Reset amplitudes for next simulation
            end
        end
    end
end
csvwrite_with_headers('experiment_test.csv',time_to_detection_list,headers);
end

function [xtx,xty] = deployTarget(A)
global m;
global n;
global Xv;
global Yv;

xtx = n+10;
xty = m+10;

targetDeployed = 0;

while ~targetDeployed
    while (xtx < -n || xty < -m || xtx > n ||xty > m || ~targetDeployed)
        targetPos = gendist(A(:)',1,1);
        [xty,xtx] = ind2sub(size(A),targetPos);
        xtx = Xv(xtx);
        xty = Yv(xty);
        targetDeployed = 1;
    end
    
    if A(find(Yv==xty),find(Xv==xtx)) == 0
        targetDeployed = 0;
    else
        targetDeployed = 1;
    end
end
end

function [xt,yt] = deployAgent(A)
global m;
global n;
global Xv;
global Yv;

xt = n+10;
yt = m+10;

agentDeployed = 0;

while ~agentDeployed
    while (xt < -n || yt < -m || xt > n ||yt > m || ~agentDeployed)
        xt = randi(2*m,1)-m;
        yt = randi(2*n,1)-n;
        agentDeployed = 1;
    end
    if A(find(Yv==yt),find(Xv==xt)) == 0
        agentDeployed = 0;
    else
        agentDeployed = 1;
    end
end
end