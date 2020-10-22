function err= validate_experiment(csv_filename)
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


err = immse(x,xe);



%change to the desired value     
fig = figure();
set(gcf, 'Position',  [100, 100, 900, 900]);



subplot(2,2,[1,2]);
plot(t,x(4,:),'DisplayName','ground-truth,gazebo','LineWidth',2)
hold on;
plot(t,xe(4,:),'DisplayName','sys-id model','LineWidth',2)
xlabel('t (seconds)') 
ylabel('Vehicle heading (radians)') 
title("Vehicle heading")
legend


subplot(2,2,[3,4]);
plot(x(1,:),x(2,:),'DisplayName','ground-truth,gazebo','LineWidth',2)
hold on;
plot(xe(1,:),xe(2,:),'DisplayName','sys-id model','LineWidth',2)
plot(x(1,1),x(2,1),'kx','DisplayName','init','LineWidth',3)
hold off;
xlabel('x (meters)') 
ylabel('y (meters)') 
title("Vehicle Position (map frame)")
legend
set(findobj(gcf,'type','axes'),'FontName','Calibri','FontSize',11,'FontWeight','Bold', 'LineWidth', 2,'layer','top');

sgt =sgtitle(strcat('Validation MSE=',string(err)));
sgt.FontSize = 20;
figname = split(strrep(csv_filename,'csv/',''),".");
savename = strcat("plots/",figname(1),".png");
saveas(fig,savename);
end

