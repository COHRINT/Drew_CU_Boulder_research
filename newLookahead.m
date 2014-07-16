function [pix,piy] = newLookahead(A,x,y)
global lookahead;
global m;
global n;

persistent currentPath;

if ~isempty(currentPath)
    pix = currentPath(1,1);
    piy = currentPath(1,2);
    currentPath(1,:) = [];
    return;
end

pix = [];
piy =[];

digits(100000);

moves = [ones(1,lookahead),zeros(1,lookahead),-1*ones(1,lookahead)];
permutations = npermutek(moves,lookahead);
permutations = unique(permutations,'rows');

if lookahead > 1
    for i = 2:size(permutations,2)
        permutations(:,i) = permutations(:,i) + permutations(:,i-1);
    end
end

xpaths = permutations + x;
ypaths = permutations + y;

utilityList = zeros(size(xpaths,1)*size(ypaths,1),1);

for i = 1:size(xpaths,1)
    for j = 1:size(ypaths,1)
        for k = 1:size(xpaths,2)
            paths(k,1:2,sub2ind([size(ypaths,1),size(xpaths,1)],i,j)) = [xpaths(i,k),ypaths(j,k)];
            if k>1
                for v = 1:k-1
                    if xpaths(i,k) == xpaths(i,v) && ypaths(j,k) == ypaths(j,v)
                        moved = 0;
                        break;
                    end
                    moved = 1;
                end
            else
                if xpaths(i,1)==x && ypaths(j,1)==y
                    moved = 0;
                else
                    moved = 1;
                end
            end
            if xpaths(i,k) > 0 && xpaths(i,k) <=2*n+1 && ypaths(j,k) > 0 && ypaths(j,k) <=2*m+1 && ~isnan(A(ypaths(j,k),xpaths(i,k))) && moved
                utilityList(sub2ind([size(xpaths,1),size(ypaths,1)],i,j)) = utilityList(sub2ind([size(xpaths,1),size(ypaths,1)],i,j)) + A(ypaths(j,k),xpaths(i,k));
            else
                utilityList(sub2ind([size(xpaths,1),size(ypaths,1)],i,j)) = -1;
                break;
            end
        end
    end
end

kstar = find(max(utilityList) == utilityList);
kstar = kstar(randi(size(kstar,1),1));
%kstar = kstar(1);

pix = paths(1,1,kstar);
piy = paths(1,2,kstar);
currentPath = paths(:,:,kstar);
currentPath(1,:) = [];

end