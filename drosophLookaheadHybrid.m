function [ixt,iyt,target] = drosophLookaheadHybrid(A,x,y,count)
global stepsSinceLastExplore;
global maxSteps;
global lookahead;

if count == 0
    stepsSinceLastExplore = 0;
    [ixt,iyt,target] = drosophilaSearch(A,x,y)
    disp('Initializing to Drosophila')
    return
end

if stepsSinceLastExplore / maxSteps >= 0.08
    [ixt,iyt,target] = drosophilaSearch(A,x,y);
    stepsSinceLastExplore = 0;
else
    [~,ixt,iyt] = Lookahead(A,x,y,1,lookahead,1);
    target = [nan,nan];
    stepsSinceLastExplore = stepsSinceLastExplore + 1;
end

end

%Target returns nan for some reason on initialization