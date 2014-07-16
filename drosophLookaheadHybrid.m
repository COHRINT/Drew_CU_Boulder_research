function [ixt,iyt,target] = drosophLookaheadHybrid(A,x,y,count)
global stepsSinceLastExplore;
global maxSteps;

if count == 0
    stepsSinceLastExplore = 0;
    [ixt,iyt,target] = drosophilaSearch(A,x,y)
    disp('Initializing to Drosophila')
    return
end

if stepsSinceLastExplore / maxSteps >= 0.08
    [ixt,iyt,target] = drosophilaSearch(A,x,y);
    stepsSinceLastExplore = 0;
    clear newLookahead;
else
    [ixt,iyt] = newLookahead(A,x,y);
    target = [nan,nan];
    stepsSinceLastExplore = stepsSinceLastExplore + 1;
end

end

%Target returns nan for some reason on initialization