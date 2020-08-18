% load the data
T = readtable('data_27180.csv');


% The order of the data (time,x,y,speed,theta,u,delta)
time = T{:,1};
x = T{:,2};
y = T{:,3};
speed = T{:,4};
theta = T{:,5};
u = T{:,6};
delta = T{:,7};

figure();
plot(time,theta);
xlabel('t') 
ylabel('theta') 
title("vehicle heading barca")


% load the data
T = readtable('racecar_walker_cclockwise_1.5.csv');


% The order of the data (time,x,y,speed,theta,u,delta)
time = T{:,1};
x = T{:,2};
y = T{:,3};
speed = T{:,4};
theta = T{:,5};
u = T{:,6};
delta = T{:,7};

figure();
plot(time,theta);
xlabel('t') 
ylabel('theta') 
title("vehicle heading racecar_walker_cc")

% load the data
T = readtable('racecar_walker_clockwise_1.5.csv');


% The order of the data (time,x,y,speed,theta,u,delta)
time = T{:,1};
x = T{:,2};
y = T{:,3};
speed = T{:,4};
theta = T{:,5};
u = T{:,6};
delta = T{:,7};

figure();
plot(time(157:400),theta(157:400));
xlabel('t') 
ylabel('theta') 
title("vehicle heading racecar_walker clockwise")


% load the data
T = readtable('track_barca.csv');


% The order of the data (time,x,y,speed,theta,u,delta)
time = T{:,1};
x = T{:,2};
y = T{:,3};
speed = T{:,4};
theta = T{:,5};
u = T{:,6};
delta = T{:,7};

figure();
plot(time,theta);
xlabel('t') 
ylabel('theta') 
title("vehicle heading track barca")