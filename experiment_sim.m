
function [time_to_detection,correct_identification]= experiment_sim(alpha,beta,A,searchID)


                                                         
global m;
global n;
global Xv;
global Yv;
global Xg;
global Yg; 
global maxSteps;
global lookahead;

%Real Location of target
targetDeployed = 0;
int32 xtx;
int32 xty;
int32 targetPos;

xtx = n+10;
xty = m+10;

while ~targetDeployed
    while (xtx < -n || xty < -m || xtx > n ||xty > m || ~targetDeployed)
        targetPos = gendist(A(:)',1,1);
        xty = Yv(floor(targetPos/(2*n+1))+1);
        xtx = Xv(targetPos - floor(targetPos/(2*n+1))*(2*n+1)+1);
        targetDeployed = 1;
    end
    
    if A(find(Yv==xty),find(Xv==xtx)) == 0
        targetDeployed = 0;
    else
        targetDeployed = 1;
    end
end


%Start Location of agent
agentDeployed = 0;
xt=n+10;
yt=m+10;
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

target = [nan,nan];
A(A==0)=nan;
patha = [];
count = 0;

% % %Begin Search Simulation
% figure(),hold on, grid
% set(gca,'color',[0 0 0]),hold on;
% plot3(xtx,xty,0.02,'mx','MarkerSize',12,'LineWidth',3)                     %%nra: plotting target
% hag = plot3(xt,yt,0.02,'co','MarkerSize',12,'LineWidth',3);                %%nra: plotting agent, creating handle for loop updates
% hplot = pcolor(Xg,Yg,A); colorbar;                                         % bar3(A);%  %%nra: use pcolor to show overhead "surface" plot with colors (although target/robot points will be on grid, not in center of the cells)
% 
% view(2)

% Search ID's
% 1 : Optimal Lookahead
% 2 : Random
% 3 : Sweeping
% 4 : Drosophila Inspired
% 5 : Saccadic

    while count <= maxSteps                                                           
         dkt = takeMeasurement(alpha,beta,xtx,xty,xt,yt);                  %Generate a pseudo-random measurement from the sensor
         A = bayesUpdate(A,alpha,beta,xt,yt,dkt,Xg,Yg);                    %Update the grid recursively 
         if max(A(:)) > 0.95                                                                  %nra: modified input to A' to fix flipping problem (hack?)
            break
         end
         ixt = find(Xv==xt);                                               %%nra: pull out the physical position index
         iyt = find(Yv==yt);
         if isempty(patha)
             
             if searchID == 1
                [util,ixt,iyt] = Lookahead(A,ixt,iyt,1,lookahead,1);              %Use lookahead algorithm to decide where to go next. The 3rd argument changes the distance the agent can move in one time step and the 4th argument changes the number of lookahead steps
             elseif searchID == 2
                [ixt,iyt] = randomSearch(A,ixt,iyt,1,m,n,dkt);
             elseif searchID == 3
                if dkt == 0
                    [ixt,iyt,target] = sweepingSearch2(A,ixt,iyt);
                end
             elseif searchID == 4
                [ixt,iyt,target] = drosophilaSearch(A,m,n,ixt,iyt,dkt);
             elseif searchID == 5
                [ixt,iyt] = saccadicSearch(A,ixt,iyt);
             end
             
             if (~isnan(target(1)) && dkt == 0);
                 patha = A_star(A,[ixt,iyt],target);
                 patha(1,:) = [];
                 ixt = patha(1,1);
                 iyt = patha(1,2);
                 patha(1,:) = [];
             end
         else
             ixt = patha(1,1);
             iyt = patha(1,2);
             patha(1,:) = [];
         end
         
         xt = Xv(ixt);                                                     %%nra: push index back into new physical position
         yt = Yv(iyt);
%          
%          delete(hplot)                                                     %%get rid of previous bar plot
%          hplot = pcolor(Xg,Yg,A); colorbar;                                %  bar3(A);%  %%nra: put this inside the loop to see evolution
%          set(hag,'XData',xt,'YData',yt)                                    %%nra: update the agent's pos'n
%          title(['d_k^t = ',num2str(dkt),', time step: ',num2str(count)],'FontSize',14) %%nra: printing time step
%          zlim([0 0.25])
%          pause(0.00001)                                                       %%nra: slow down the loop to see the plot
         
         count = count + 1;
    end
   
    time_to_detection = count;
    if time_to_detection == maxSteps
        correct_identification = 0;
    else
    correct_identification = xtx == Xv(ixt) && xty == Yv(iyt);
    end