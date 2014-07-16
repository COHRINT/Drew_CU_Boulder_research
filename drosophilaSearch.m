%   Author: Drew Ellison
%   Email: dme722@gmail.com
%   File Description:
%   The drosophila search finds the cell with the current maximum belief,
%   and then designates that cell as the current target cell. The A* algorithm
%   then takes over and generates a path to that cell. 

function [pix,piy,target] = drosophilaSearch(A,x,y)
    [ymaxlist,xmaxlist]=find(A==max(A(:)));
    s = size(ymaxlist);
    sgrid = size(A);
    d = sgrid(1)^2*10000;
    for i =1:s
        if (ymaxlist(i) - y)^2 + (xmaxlist(i) - x)^2 < d
            ymax = ymaxlist(i);
            xmax = xmaxlist(i);
            d = (ymaxlist(i) - y)^2 + (xmaxlist(i) - x)^2;
        end
    end
    
    target = [xmax , ymax];
    
    if target == [x , y]
        pix = x;
        piy = y;
        target = [nan,nan];
    else
        pix = x;
        piy = y;
    end