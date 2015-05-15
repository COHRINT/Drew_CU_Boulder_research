function K_star = K_star_calc(data,x_star,sigma_f,l,sigma_n,n)

K_star = zeros(1,size(data,1));

for i = 1:size(data,1)
    K_star(i) = covar_func(x_star,data(i,1),sigma_f,l,sigma_n,n);
end