function newBelief = bayesUpdate(A,alpha,beta,xk,yk,dkt,Xgrid,Ygrid)
%[m,n] = size(A);
newBelief = A;

if dkt == 0
    term3 = beta;
    term5 = 1 - alpha;
else
    term3 = 1 - beta;
    term5 = alpha;
end

%%nra: find out which indices in Xgrid and Ygrid vectors xk and yk correspond to
Xv = Xgrid(1,:); %%all rows the same in Xgrid
Yv = Ygrid(:,1); %%all cols the same in Ygrid
ixk = find(Xv==xk);
iyk = find(Yv==yk);
for i = 1:size(A,2) %1:n,
    for j = 1:size(A,1) %1:m,
        if dkt == 0
            if xk == Xgrid(j,i) && yk == Ygrid(j,i)
                term1 = beta;
            else
                term1 = 1 - alpha;
            end
        else
            if xk == Xgrid(j,i) && yk == Ygrid(j,i)
                term1 = 1-beta;
            else
                term1 = alpha;
            end
        end
        newBelief(j,i) =  term1*A(j,i)/(term3*A(iyk,ixk)+term5*(1-A(iyk,ixk))); %%nra: I think there was a transpose error here? should be A(j,i), not A(i,j)
    end
end
%%newBelief = newBelief ./sum(newBelief(:));

            
        

