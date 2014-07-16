%   Author: Drew Ellison
%   Email: dme722@gmail.com
%   File Description:
%   The simulate_search function receives the necessary parameters
%   (alpha,beta, the grid, the searchID, the target location, and the agent
%   start location) in order to perform an appropriate simulation, upon
%   termination (either by detection or by the maximum time being reached)
%   the function returns the number of time steps used for the simulation
%   and a binary value of whether a correct identification was made.

function [time_to_detection,correct_identification]= simulate_search(alpha,beta,A,searchID,xtx,xty,xt,yt)
%Subscribe to necessary global variables
global Xv;
global Yv;
global Xg;
global Yg;
global maxSteps;

%Start the target for A* as not available
target = [nan,nan];
%Initialize the A* generated path as empty
patha = [];
%Replace 0 probability cells with nan to indicate that it is an obstacle
A(A==0)=nan;
%Initialize time step counter
count = 0;

% Uncomment to show visualization of search
% % %Begin Search Simulation
% figure(),hold on, grid
% set(gca,'color',[0 0 0]),hold on;
% plot3(xtx,xty,0.02,'mx','MarkerSize',12,'LineWidth',3)                     %%nra: plotting target
% hag = plot3(xt,yt,0.02,'co','MarkerSize',12,'LineWidth',3);                %%nra: plotting agent, creating handle for loop updates
% hplot = pcolor(Xg,Yg,A); colorbar;                                         % bar3(A);%  %%nra: use pcolor to show overhead "surface" plot with colors (although target/robot points will be on grid, not in center of the cells)
% view(2)

% Search ID's
% 1 : Optimal Lookahead
% 2 : Sweeping
% 3 : Drosophila Inspired
% 4 : Saccadic

% Publish number of search IDs being used
global numSearchIDs;
numSearchIDs = 4;

clear newLookahead;
while count <= maxSteps                                                     %Only run simulation for alloted time
    dkt = takeMeasurement(alpha,beta,xtx,xty,xt,yt);                        %Generate a pseudo-random measurement from the sensor
    A = bayesUpdate(A,alpha,beta,xt,yt,dkt,Xg,Yg);                          %Update the grid recursively
    if max(A(:)) > 0.95                                                     %If the grid has any cell with >95% probability of containing the target, consider the target found
        break
    end
    ixt = find(Xv==xt);                                               
    iyt = find(Yv==yt);
    if isempty(patha)
        if searchID == 1
            [ixt,iyt,target] = drosophLookaheadHybrid(A,ixt,iyt,count);
            %[ixt,iyt] = newLookahead(A,ixt,iyt);
        %elseif searchID == 2
            %[ixt,iyt] = randomSearch(A,ixt,iyt,1,m,n,dkt);
        elseif searchID == 2
            if dkt == 0
                [ixt,iyt,target] = sweepingSearch(A,ixt,iyt);
            end
        elseif searchID == 3
            [ixt,iyt,target] = drosophilaSearch(A,ixt,iyt);
        elseif searchID == 4
            [ixt,iyt] = saccadicSearch(A,ixt,iyt);
        end
        %If a target for A* has been generated, run A*
        if (~isnan(target(1)));
            patha = A_star(A,[ixt,iyt],target);
            patha(1,:) = [];
            ixt = patha(1,1);
            iyt = patha(1,2);
            patha(1,:) = [];
            %disp('Using A*')
        end
    %If a path exists from A*, use the next step instead of running a 
    %search algorithm again. Remove movement from path. 
    else
        ixt = patha(1,1);
        iyt = patha(1,2);
        patha(1,:) = [];
    end
    count = count + 1;
    
    % Uncomment to show visualization of search
    %          xt = Xv(ixt);                                                     %%nra: push index back into new physical position
    %          yt = Yv(iyt);
    %          delete(hplot)                                                     %%get rid of previous bar plot
    %          hplot = pcolor(Xg,Yg,A); colorbar;                                %  bar3(A);%  %%nra: put this inside the loop to see evolution
    %          set(hag,'XData',xt,'YData',yt)                                    %%nra: update the agent's pos'n
    %          title(['d_k^t = ',num2str(dkt),', time step: ',num2str(count)],'FontSize',14) %%nra: printing time step
    %          zlim([0 0.25])
    %          pause(0.00001)                                                       %%nra: slow down the loop to see the plot
end

time_to_detection = count;
if time_to_detection == maxSteps
    correct_identification = 0;
else
    correct_identification = xtx == Xv(ixt) && xty == Yv(iyt);
end

clear sweepingSearch
clear drosophLookaheadHybrid
end


function newBelief = bayesUpdate(A,alpha,beta,xk,yk,dkt,Xgrid,Ygrid)
newBelief = A;
if dkt == 0
    term3 = beta;
    term5 = 1 - alpha;
else
    term3 = 1 - beta;
    term5 = alpha;
end
Xv = Xgrid(1,:); %%all rows the same in Xgrid
Yv = Ygrid(:,1); %%all cols the same in Ygrid
ixk = find(Xv==xk);
iyk = find(Yv==yk);
for i = 1:size(A,2) %1:n,
    for j = 1:size(A,1) %1:m,
        if dkt == 0
            if xk == Xgrid(j,i) && yk == Ygrid(j,i)
                term1 = beta;
            else
                term1 = 1 - alpha;
            end
        else
            if xk == Xgrid(j,i) && yk == Ygrid(j,i)
                term1 = 1-beta;
            else
                term1 = alpha;
            end
        end
        newBelief(j,i) =  term1*A(j,i)/(term3*A(iyk,ixk)+term5*(1-A(iyk,ixk)));
    end
end
end

function detection = takeMeasurement(alpha,beta,xtx,xty,x,y)
r = rand;
if (xtx ~= x || xty ~= y)
    if (r<alpha)
        detection = 1;
    else
        detection = 0;
    end
else
    if (r<beta)
        detection = 0;
    else
        detection = 1;
    end
end
end