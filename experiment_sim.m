
function [time_to_detection,correct_identification]= experiment_sim(alpha,beta,A,searchID,xtx,xty,xt,yt)
global Xv;
global Yv;
global Xg;
global Yg; 
global maxSteps;

target = [nan,nan];
A(A==0)=nan;
patha = [];
count = 0;

% % %Begin Search Simulation
figure(),hold on, grid
set(gca,'color',[0 0 0]),hold on;
plot3(xtx,xty,0.02,'mx','MarkerSize',12,'LineWidth',3)                     %%nra: plotting target
hag = plot3(xt,yt,0.02,'co','MarkerSize',12,'LineWidth',3);                %%nra: plotting agent, creating handle for loop updates
hplot = pcolor(Xg,Yg,A); colorbar;                                         % bar3(A);%  %%nra: use pcolor to show overhead "surface" plot with colors (although target/robot points will be on grid, not in center of the cells)

view(2)

% Search ID's
% 1 : Optimal Lookahead
% 2 : Sweeping
% 3 : Drosophila Inspired
% 4 : Saccadic
global numSearchIDs;
numSearchIDs = 4;
global stepsSinceLastExplore;
clear newLookahead;
    while count <= maxSteps                                                           
         dkt = takeMeasurement(alpha,beta,xtx,xty,xt,yt);                  %Generate a pseudo-random measurement from the sensor
         A = bayesUpdate(A,alpha,beta,xt,yt,dkt,Xg,Yg);                    %Update the grid recursively 
         if max(A(:)) > 0.95                                                                  %nra: modified input to A' to fix flipping problem (hack?)
             stepsSinceLastExplore = [];
             break
         end
         ixt = find(Xv==xt);                                               %%nra: pull out the physical position index
         iyt = find(Yv==yt);
         if isempty(patha)
             
             if searchID == 1
                 
               [ixt,iyt,target] = drosophLookaheadHybrid(A,ixt,iyt,count);
%               [ixt,iyt] = newLookahead(A,ixt,iyt);            
%              elseif searchID == 2
%                 [ixt,iyt] = randomSearch(A,ixt,iyt,1,m,n,dkt);
             elseif searchID == 2
                if dkt == 0
                    [ixt,iyt,target] = sweepingSearch2(A,ixt,iyt);
                end
             elseif searchID == 3
                [ixt,iyt,target] = drosophilaSearch(A,ixt,iyt);
             elseif searchID == 4
                [ixt,iyt] = saccadicSearch(A,ixt,iyt);
             end
             
             if (~isnan(target(1)));
                 patha = A_star(A,[ixt,iyt],target);
                 patha(1,:) = [];
                 ixt = patha(1,1);
                 iyt = patha(1,2);
                 patha(1,:) = [];
                 disp('Using A*')
             end
         else
             ixt = patha(1,1);
             iyt = patha(1,2);
             patha(1,:) = [];
         end
         
         xt = Xv(ixt);                                                     %%nra: push index back into new physical position
         yt = Yv(iyt);          
         delete(hplot)                                                     %%get rid of previous bar plot
         hplot = pcolor(Xg,Yg,A); colorbar;                                %  bar3(A);%  %%nra: put this inside the loop to see evolution
         set(hag,'XData',xt,'YData',yt)                                    %%nra: update the agent's pos'n
         title(['d_k^t = ',num2str(dkt),', time step: ',num2str(count)],'FontSize',14) %%nra: printing time step
         zlim([0 0.25])
         pause(0.00001)                                                       %%nra: slow down the loop to see the plot
         
         count = count + 1;
    end
   
    time_to_detection = count;
    if time_to_detection == maxSteps
        correct_identification = 0;
    else
    correct_identification = xtx == Xv(ixt) && xty == Yv(iyt);

    end