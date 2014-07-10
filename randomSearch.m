function [pix,piy] = randomSearch(A,x,y,dist,m,n,dkt)
possibleCells = [];
if dkt == 0
    for i=-dist:dist
        for j = -dist:dist
            if (x+j >= 1 && y+i >=1 && x+j <= 2*n+1 && y+i <= 2*m+1 && ~isnan(A(y+i,x+j)))
                possibleCells = [possibleCells;[x+j,y+i]];
            end
        end
    end
msize = size(possibleCells);

cell = possibleCells(randperm(msize(1),1),:);

pix = cell(1);
piy = cell(2);

else
    pix = x;
    piy = y;
end

