function data = load_experiment_data(csvFile,heading)
% load the data
T = readtable(csvFile);
n = 15; % number of data samples to remove from beginning and end

% The order of the data (time,x,y,speed,theta,u,delta)
x = T{n:end-n,2};
y = T{n:end-n,3};
speed = T{n:end-n,4};
theta = T{n:end-n,5};
u = T{n:end-n,6};
delta = T{n:end-n,7};

% create the input and output matrices
if heading == 0
    output = {x,y,speed};
elseif heading == 1
    output = {x,y,speed,theta};
else
    error('heading value must be 0 or 1')
end
output = cell2mat(output);
input = {u,delta};
input = cell2mat(input);

% Create iddata
data = iddata(output,input,0.05);
if heading == 0
    data.OutputName ={'x';'y';'speed'};
elseif heading == 1
    data.OutputName ={'x';'y';'speed';'theta'};
else
    error('heading value must be 0 or 1')
end
data.InputName ={'u';'delta'};
end

