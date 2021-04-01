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
set(gcf,'color','w');
set(gcf, 'Position',  [100, 100, 900, 900]);



subplot(2,2,[1,2]);
plot(t,x(4,:),'-','Color', [157, 158, 157]/255,'DisplayName','ground-truth,gazebo','LineWidth',2)
hold on;
plot(t,xe(4,:),'--','Color', [70, 143, 199]/255,'DisplayName','sys-id','LineWidth',2)
xlabel('Time (s)') 
ylabel('Vehicle heading (radians)') 
t = title("Vehicle heading (radians)",'Color',[87, 93, 97]/255);
set(t, 'horizontalAlignment', 'left')
set(t, 'units', 'normalized')
set(t, 'position', [0.01 1.01 0]);
legend
ax = gca; % Get handle to current axes.
ax.XColor = [87, 93, 97]/255; % Red
ax.YColor = [87, 93, 97]/255; % Blue
set(gca,'box','off')
legend boxoff 
xlabel('x (meters)') 
ylabel('y (meters)') 
t = title("Vehicle Heading",'Color',[87, 93, 97]/255);

subplot(2,2,[3,4]);
plot(x(1,:),x(2,:),'-','Color', [157, 158, 157]/255,'DisplayName','ground-truth,gazebo','LineWidth',2)
hold on;
plot(xe(1,:),xe(2,:),'--','Color', [70, 143, 199]/255,'DisplayName','sys-id','LineWidth',2)
plot(x(1,1),x(2,1),'*','Color', [38, 38, 38]/255,'DisplayName','init','LineWidth',2)
hold off;
xlabel('x (meters)') 
ylabel('y (meters)') 
ax = gca; % Get handle to current axes.
ax.XColor = [87, 93, 97]/255; % Red
ax.YColor = [87, 93, 97]/255; % Blue
set(gca,'box','off')
t = title(strcat("Vehicle Position (map frame) MSE=",string(round(err,4))),'Color',[87, 93, 97]/255);
set(t, 'horizontalAlignment', 'left')
set(t, 'units', 'normalized')
set(t, 'position', [0.01 1.01 0]);
legend
legend boxoff 
set(findobj(gcf,'type','axes'),'FontName','Calibri','FontSize',11,'FontWeight','Bold', 'LineWidth', 1,'layer','top');

%sgt =sgtitle(strcat('Validation MSE=',string(err)));
%sgt.FontSize = 20;
figname = split(strrep(csv_filename,'csv/',''),".");
savename = strcat("plots/",figname(1),".png");
saveas(fig,savename);
end

