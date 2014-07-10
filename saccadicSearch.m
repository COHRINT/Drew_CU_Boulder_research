function [pix,piy] = saccadicSearch(A,x,y)
    [ymaxlist,xmaxlist]=find(A==max(A(:)));
    s = size(ymaxlist);
    sgrid = size(A);
    d = sgrid(1)^2;
    for i =1:s
        if (ymaxlist(i) - y)^2 + (xmaxlist(i) - x)^2 < d
            piy = ymaxlist(i);
            pix = xmaxlist(i);
        end
    end