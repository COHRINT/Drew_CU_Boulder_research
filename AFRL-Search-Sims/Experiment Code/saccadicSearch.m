%   Author: Drew Ellison
%   Email: dme722@gmail.com
%   File Description:
%   Saccadic search presents a nonphysically realizable simulation in which
%   the agent is allowed to jump from cell to cell. It always chooses to
%   move to the cell that contains the current highest belief. 

function [pix,piy] = saccadicSearch(A,x,y)
    [ymaxlist,xmaxlist]=find(A==max(A(:)));
    s = size(ymaxlist);
    sgrid = size(A);
    d = sgrid(1)^2*10000;
    for i =1:s
        if (ymaxlist(i) - y)^2 + (xmaxlist(i) - x)^2 < d
            piy = ymaxlist(i);
            pix = xmaxlist(i);
            d = (ymaxlist(i) - y)^2 + (xmaxlist(i) - x)^2;
        end
    end