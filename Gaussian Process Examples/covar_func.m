function k = covar_func(x,x_prime,sigma_f,l,sigma_n,n)
if n ==1
    if x == x_prime
        delta = 1;
    else
        delta = 0;
    end
    
    k = sigma_f^2 * exp(-(x-x_prime)^2/(2*l^2)) + sigma_n^2 * ...
        delta;
elseif n==2
    k = exp(-.0001*abs(x-x_prime));
elseif n == 3
    if x == x_prime
        delta = 1;
    else
        delta = 0;
    end
    
    k = sigma_f^2 * exp(-(x-x_prime)^2/(2*l^2)) + sigma_n^2 * ...
        delta + exp(-2*sin(1/pi*pi*(x-x_prime))^2);
end
end