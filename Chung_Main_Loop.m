
clc
clear
close all

%%GLOBAL PARAMETERS
delta = 1;                                                                 %The probability that the target is in the search space
alpha = 0.2;                                                               %Probability of a false alarm
beta = 0.2;                                                                %Probability of a missed detection

%Initialize grid
m = 15;                                                                    %grid y extent   
n = 15;                                                                    %grid x extent

Xv = -m:m;
Yv = -n:n;
[Xg ,Yg]=meshgrid(Xv,Yv);                                                  %%nra: modified to -m:m and -n:n in (X,Y) to get "meat" of Gaussian prior

%Set up obstacles 
obstacleGrid = ones((2*m+1),(2*n+1));
obstacleGrid(14:18,14:18) = zeros(5);

%%Uniform Prior
% A = zeros(size(Xg));
% A(:) = 1/(numel(A));                                                     %%nra: faster method for setting uniform distribution

%%(truncated) Gaussian Prior
sigmax = sqrt(27);
sigmay = sqrt(15);
mux = 1;
muy = 3;
gaussPrior = exp(-((Xg-mux).^2/(2*sigmax^2)+(Yg-muy).^2/(2*sigmay)^2)); %%nra: ok, can use mvnpdf cmd in stats toolbox
A = delta*gaussPrior.*obstacleGrid;                                %nra: faster way to set up A (no need to loop through)
A = A./(sum(A(:)));
%%nra: sanity check: does everything add up to delta?
sum(sum(A));
A(A==0)=nan;

%Real Location of target
sx = 5;
sy = 3;
targetDeployed = 0;
xtx=n+10;
xty=m+10;
while ~targetDeployed
    while (xtx < -n || xty < -m || xtx > n ||xty > m || ~targetDeployed)
        xtx = floor(randn*sigmax+mux);
        xty = floor(randn*sigmay+muy);
        targetDeployed = 1;
    end
    
    if isnan(A(find(Yv==xty),find(Xv==xtx)))
        targetDeployed = 0;
    else
        targetDeployed = 1;
    end
end

%xtx = randi(2*sx,1)-sx;
%xty = randi(2*sy,1)-sy;

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
    if isnan(A(find(Yv==yt),find(Xv==xt)))
        agentDeployed = 0;
    else
        agentDeployed = 1;
    end
end

%Begin Search Simulation
figure(),hold on, grid
set(gca,'color',[0 0 0]),hold on;
plot3(xtx,xty,0.02,'mx','MarkerSize',12,'LineWidth',3)                     %%nra: plotting target
hag = plot3(xt,yt,0.02,'co','MarkerSize',12,'LineWidth',3);                %%nra: plotting agent, creating handle for loop updates
hplot = pcolor(Xg,Yg,A); colorbar;                                         % bar3(A);%  %%nra: use pcolor to show overhead "surface" plot with colors (although target/robot points will be on grid, not in center of the cells)

view(2)
steps = 10000;                                                               %Change the number of time steps in simulation
dist = 1;
target = [nan,nan];
patha = [];

%Lookahead Parameter
lookahead = 5; % nra: try varying # of look ahead steps
    for v = 1:steps,                                                            
         dkt = takeMeasurement(alpha,beta,xtx,xty,xt,yt);                  %Generate a pseudo-random measurement from the sensor
         A = bayesUpdate(A,alpha,beta,xt,yt,dkt,Xg,Yg);                    %Update the grid recursively 
         if max(A(:)) > 0.95                                                                  %nra: modified input to A' to fix flipping problem (hack?)
            break
         end
         ixt = find(Xv==xt);                                               %%nra: pull out the physical position index
         iyt = find(Yv==yt);
         if isempty(patha)
             utility = 0;
             %[util,ixt,iyt] = Lookahead(A,ixt,iyt,dist,lookahead,1);              %Use lookahead algorithm to decide where to go next. The 3rd argument changes the distance the agent can move in one time step and the 4th argument changes the number of lookahead steps
             %[ixt,iyt] = randomSearch(A,ixt,iyt,dist,m,n,dkt);
             [ixt,iyt,target] = sweepingSearch2(A,ixt,iyt);
             %[ixt,iyt,target] = drosophilaSearch(A,m,n,ixt,iyt,dkt);
             %[ixt,iyt] = saccadicSearch(A,ixt,iyt);
             
             if ~isnan(target);
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
         delete(hplot)                                                     %%get rid of previous bar plot
         hplot = pcolor(Xg,Yg,A); colorbar;                                %  bar3(A);%  %%nra: put this inside the loop to see evolution
         set(hag,'XData',xt,'YData',yt)                                    %%nra: update the agent's pos'n
         title(['d_k^t = ',num2str(dkt),', time step: ',num2str(v)],'FontSize',14) %%nra: printing time step
         %zlim([0 0.25])
         pause(0.00001)                                                       %%nra: slow down the loop to see the plot
    end
   
    
   