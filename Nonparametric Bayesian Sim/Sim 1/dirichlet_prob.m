function p = dirichlet_prob(mu_list,alpha_list,data)

N = sum(data);

d_terms = [];


for i = 1:size(mu_list)
    d_terms = [d_terms,mu_list(i)^(alpha_list(i) + data(i) - 1)];
end

denom_terms = [];
for i = 1:size(mu_list)
    denom_terms = [denom_terms,gamma(alpha_list(i) + data(i))];
end

p = gamma(sum(alpha_list) + N)/(prod(denom_terms)) * prod(d_terms);

end