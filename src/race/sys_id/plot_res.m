function plot_res(data,ground_truth)
%PLOT_RES Summary of this function goes here
%   Detailed explanation goes here
time = data.SamplingInstants;

figure();
plot(data.y(:,1),data.y(:,2));
hold on;
plot(ground_truth.y(:,1),ground_truth.y(:,2));
xlabel('x (m)') 
ylabel('y (m)') 
title("Position of Vehicle")

% Let's look at the speed and throttle data we have 
figure();
plot(time,data.y(:,3));
hold on
plot(time,ground_truth.y(:,3));
xlabel('Time (s)') 
ylabel('Speed (m/s)') 
title("Speed input of the car")

% Let's look at the angle and input data we have 
figure();
plot(time, data.y(:,4));
hold on
plot(time,ground_truth.y(:,4));
xlabel('Time (s)') 
ylabel('theta (rad)') 
title("Heading of car")
hold off
end

