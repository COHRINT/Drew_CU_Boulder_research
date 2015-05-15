function K = K_mat(data,sigma_f,l,sigma_n,n)

K = zeros(size(data,1));

for i = 1:size(data,1)
    for j = 1:size(data,1)
        K(i,j) = covar_func(data(i,1),data(j,1),sigma_f,l,sigma_n,n);
    end
end
end