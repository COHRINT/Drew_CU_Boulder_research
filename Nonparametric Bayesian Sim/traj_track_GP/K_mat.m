function K = K_mat(data,sigma_f,l,sigma_n)

K = zeros(size(data,1));

for i = 1:size(data,1)
    for j = 1:size(data,1)
        K(i,j) = k(data(i,:),data(j,:),l,sigma_f,sigma_n);
    end
end

end