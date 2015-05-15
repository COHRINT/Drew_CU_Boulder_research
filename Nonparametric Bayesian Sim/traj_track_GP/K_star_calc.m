function K_star = K_star_calc(data,x_star,sigma_f,l,sigma_n)

K_star = zeros(1,size(data,1));

for i = 1:size(data,1)
    K_star(i) = k(x_star,data(i,:),l,sigma_f,sigma_n);
end