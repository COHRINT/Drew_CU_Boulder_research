function covar = k(x,x_prime,l,sigma_f,sigma_n)

if x == x_prime
    delta = 1;
else
    delta = 0;
end

covar = sigma_f^2 * exp(-1/2 * (x-x_prime)*inv(l)*(x-x_prime)') + sigma_n^2 * ...
        delta;
end