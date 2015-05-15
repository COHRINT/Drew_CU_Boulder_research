function hmm_test_3
clear
clc
space1 = space([0,10],[0,10],1000);

A = (mvnpdf([space1.grid_x(:) space1.grid_y(:)],[1 1],[1 0;0 1])+mvnpdf([space1.grid_x(:) space1.grid_y(:)],[9 1],[1 0;0 1]))/2;
A = reshape(A,length(space1.grid_y),length(space1.grid_x));

z_lim = .2;
pcolor(space1.grid_x,space1.grid_y,A)
%axis([0 10 0 10 0 z_lim])
pause(.03)


exit_pos = [1,9];
createHeuristicMap(space1,[3,3]);
hmm = createMultipleTargetHMM(space1,exit_pos);

for i = 1:10000
    sum(sum(A))
    A = reshape(A,size(space1.grid_x,1)*size(space1.grid_y,2),1);
    A = reshape(hmm * A,size(space1.grid_x,1),size(space1.grid_x,2));
    pcolor(space1.grid_x,space1.grid_y,A)
    pause(.01)
    i
end

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

function hmm = createHMM(space,targetpos)
hmap = createHeuristicMap(space,targetpos);

hmm = zeros(size(hmap,1)*size(hmap,2));

[m,n] = size(hmap);

for i = 1:size(hmap,2)
    for j = 1:size(hmap,1)
        for k = -1:1
            for l = -1:1
                if i + k >= 1 && i + k <= n && j + l >= 1 && j + l <= m
                    hmm(sub2ind([m n],j+l,i+k),sub2ind([m n],j,i)) = 1/hmap(j+l,i+k);
                end
            end
        end

    end
end

for i =1:size(hmm,2)
    hmm(:,i) = hmm(:,i)/sum(hmm(:,i));
end


end

function hmm = createMultipleTargetHMM(space,targetposes)

hmm_sum = zeros(size(space.grid_x,1)*size(space.grid_x,2));

for i = 1:size(targetposes)
    hmm_sum = hmm_sum+createHMM(space,targetposes(i,:));
end

hmm = hmm_sum/size(targetposes,1);
end
