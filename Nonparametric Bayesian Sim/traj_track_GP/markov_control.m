function map = markov_control()

x = 1:10;
y = 1:10;

[X,Y] = meshgrid(x,y);

turnrate = 1;
velocity = 1;

map = zeros(10,10,2);

for i = 1:size(X,1)
    for j = 1:size(X,2)
        map(i,j,:) = [turnrate,velocity] + [randn*.0,randn*0];
    end
end
end