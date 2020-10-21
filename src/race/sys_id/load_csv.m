function [xp,up,tp] = load_csv(csvFile)
T = readtable(csvFile);
n = 15; % number of data samples to remove from beginning and end
x = reshape(T{n:end-n,2},1,[]);
y = reshape(T{n:end-n,3},1,[]);
speed = reshape(T{n:end-n,4},1,[]);
theta = reshape(T{n:end-n,5},1,[]);
u = reshape(T{n:end-n,6},1,[]);
delta = reshape(T{n:end-n,7},1,[]);

xp = vertcat(x,y,speed,theta);
up = vertcat(u, delta);
end

