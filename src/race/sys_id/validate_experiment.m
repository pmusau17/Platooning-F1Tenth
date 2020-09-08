function validate_experiment(csv_filename)
% load the data from the csv 
[x,u] = load_csv(csv_filename);

% Initialize the time stp
timeStep = 0.05;

% create an array of zeros to store time steps and state vector
t=zeros(length(x),1);
xe = {};
for i=1:length(x)
    t(i) = (i-1)*timeStep;
    xe{i} = simulate_bicycle_euler(x(:,i),u(:,i),timeStep);
end
xe =cell2mat(xe);
% plot the results

figure()
subplot(2,2,1);
plot(t,x(3,:),'DisplayName','ground-truth,gazebo')
hold on;
plot(t,xe(3,:),'DisplayName','sys-id model')
xlabel('t') 
ylabel('linear speed') 
title("Experiment 1: Linear Speed of Vehicle")
legend

subplot(2,2,2);
plot(t,x(4,:),'DisplayName','ground-truth,gazebo')
hold on;
plot(t,xe(4,:),'DisplayName','sys-id model')
xlabel('t') 
ylabel('Vehicle heading (radians)') 
title("Experiment 1: Vehicle heading")
legend


subplot(2,2,[3,4]);
plot(x(1,:),x(2,:),'DisplayName','ground-truth,gazebo')
hold on;
plot(xe(1,:),xe(2,:),'DisplayName','sys-id model')
hold off;
xlabel('x') 
ylabel('y') 
title("Experiment 1: Position of Vehicle")
legend
end

