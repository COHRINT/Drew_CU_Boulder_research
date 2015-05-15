%   Author: Drew Ellison
%   Email: dme722@gmail.com
%   File Description:
%   This hybrid search uses the drosophila inspired search at
%   initialization and after a predetermined percentage of the maximum time
%   to generate a path to the maximum belief cell, then switching to the
%   optimal lookahead strategy. This search presents the benefits of a
%   global exploration search and a local exploration search. 

function [ixt,iyt,target] = drosophLookaheadHybrid(A,x,y,count)
persistent stepsSinceLastExplore;
global maxSteps;

if count == 0
    stepsSinceLastExplore = 0;
    [ixt,iyt,target] = drosophilaSearch(A,x,y);
    return
end

if stepsSinceLastExplore / maxSteps >= 0.08 % Currently uses 8% of max time
    [ixt,iyt,target] = drosophilaSearch(A,x,y);
    stepsSinceLastExplore = 0;
    clear newLookahead;
else
    [ixt,iyt] = newLookahead(A,x,y);
    target = [nan,nan];
    stepsSinceLastExplore = stepsSinceLastExplore + 1;
end

end