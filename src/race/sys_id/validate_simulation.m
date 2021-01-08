file1 = 'csv/data_15870_0.csv';
file2 = 'csv/data_15870_2.csv';
file3 = 'csv/data_15870_3.csv';
file4 = 'csv/data_15870_5.csv';
file5 = 'csv/data_15870_6.csv';
file6 = 'csv/data_15870_8.csv';

set(gcf,'color','w');
set(gcf, 'Position',  [100, 100, 1200, 900])

[x,u] = load_csv(file1);


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

plot(x(1,1),x(2,1),'*','Color', [38, 38, 38]/255,'DisplayName','ground-truth,gazebo','LineWidth',2)
hold on 
plot(x(1,:),x(2,:),'-','Color', [157, 158, 157]/255,'DisplayName','ground-truth,gazebo','LineWidth',2)


 for k=1:incr:length(u)
    %plot(x(1,k),x(2,k),'bo','LineWidth',1); 
    x_sim = cell2mat(xj{k,:})';
    plot(x_sim(:,1),x_sim(:,2),'--','Color', [70, 143, 199]/255,'LineWidth',2)
 end 


ax = gca; % Get handle to current axes.
ax.XColor = [87, 93, 97]/255; % Red
ax.YColor = [87, 93, 97]/255; % Blue
xlabel('x (meters)') 
ylabel('y (meters)') 
t = title("Vehicle Position (map frame)",'Color',[87, 93, 97]/255);

set(t, 'horizontalAlignment', 'left')
set(t, 'units', 'normalized')
%h1 = get(t, 'position');
set(t, 'position', [0.01 1.01 0]);

legend('init','ground-truth,gazebo','prediction','Location','northwest')
legend boxoff 

set(gca,'box','off')
