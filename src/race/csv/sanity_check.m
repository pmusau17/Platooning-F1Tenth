tspan = [0 10];
y0 = [0;0.0;0;0];
[t,y] = ode45(@(t,y) bicycle_model_fixed(t,y), tspan, y0);

figure();
plot(y(:,1),y(:,2));
xlabel('x') 
ylabel('y') 
title("Position of Vehicle")

figure();
plot(t,y(:,4));
xlabel('t') 
ylabel('theta') 
title("heading vehicle")

figure();
plot(t,y(:,3));
xlabel('t') 
ylabel('speed') 
title("vehicle speed")