file1 = 'csv/data_15870_0.csv';
file2 = 'csv/data_15870_2.csv';
file3 = 'csv/data_15870_3.csv';
file4 = 'csv/data_15870_5.csv';
file5 = 'csv/data_15870_6.csv';
file6 = 'csv/data_15870_8.csv';



[x,u] = load_csv(file3);


xe = {};
timeStep = 1.0;
% 
sim_time = 0.5; 
step_size = 0.05;
simrange=0:step_size:sim_time;

xj = {};


incr = 10;
for j=1:incr:length(u)
    xe = {x(:,j)};
    for i=1:length(simrange)
        xe{i+1} = simulate_bicycle_euler(x(:,j),u(:,j),timeStep);
    end
    xj{j,1} = xe; 
end

plot(x(1,:),x(2,:),'b','DisplayName','ground-truth,gazebo','LineWidth',2)
hold on 


 for k=1:incr:length(u)
    plot(x(1,k),x(2,k),'bo','LineWidth',1); 
    x_sim = cell2mat(xj{k,:})';
    plot(x_sim(:,1),x_sim(:,2),'--','LineWidth',2)
end 

xlabel('x (meters)') 
ylabel('y (meters)') 
title("Vehicle Position (map frame)")
