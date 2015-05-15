function [newBelief,hmm] = propagateBeliefHMM(space,search,targets)
persistent hmmModel;

A = search.A;


if isempty(hmmModel)
    if isempty(targets)
        hmmModel = eye(size(A,1)*size(A,2));
        hmm = hmmModel;
        newBelief = A;
        return
    else
        hmmModel = createMultipleTargetHMM(A,space,targets);
    end
end

A(isnan(A)) = 0;

A = reshape(A,size(space.grid_x,1)*size(space.grid_y,2),1);
A = reshape(hmmModel * A,size(space.grid_x,1),size(space.grid_x,2));
A(A==0) = nan;
newBelief = A;
hmm = hmmModel;
end

function hmap = createHeuristicMap(space,targetpos)
[m,n] = size(space.grid_x);
hmap = zeros(size(space.grid_x));

for i = 1:n
    for j = 1:m
        hmap(j,i) = sqrt((space.grid_x(1,i)-targetpos(1))^2+(space.grid_y(j,1)-targetpos(2))^2);
    end
end
end

function hmm = createHMM(A,space,targetpos)
hmap = createHeuristicMap(space,targetpos);

hmm = zeros(size(hmap,1)*size(hmap,2));

[m,n] = size(hmap);

for i = 1:size(hmap,2)
    for j = 1:size(hmap,1)
        min = 10000;
        k_ =i;
        l_ = j;
        for k = -1:1
            for l = -1:1
                if i + k >= 1 && i + k <= n && j + l >= 1 && j + l <= m %&& ~(k == 0 && l == 0)
                    if(~isnan(A(j+l,i+k)))
                        hmm(sub2ind([m n],j+l,i+k),sub2ind([m n],j,i)) = 1/(hmap(j+l,i+k))^2;
                        if hmap(j+l,i+k) < min
                            k_ = i+k;
                            l_ = j+l;
                            min = hmap(j+l,i+k);
                        end
                    end
                end
            end
        end
       hmm(sub2ind([m n],l_,k_),sub2ind([m n],j,i)) = hmm(sub2ind([m n],l_,k_),sub2ind([m n],j,i))  + 10;
    end
end

for i =1:size(hmm,2)
    if sum(hmm(:,i)) ~=0
        hmm(:,i) = hmm(:,i)/sum(hmm(:,i));
    end
end


end

function hmm = createMultipleTargetHMM(A,space,targetposes)

hmm_sum = zeros(size(space.grid_x,1)*size(space.grid_x,2));

for i = 1:size(targetposes)
    hmm_sum = hmm_sum+createHMM(A,space,targetposes(i,:));
end

hmm = hmm_sum/size(targetposes,1);
end