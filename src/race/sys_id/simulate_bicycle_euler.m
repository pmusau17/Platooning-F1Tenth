function xp = simulate_bicycle_euler(x,u,stepSize)

% euler simulation of bicycle dynamics
% sysid parameters
ca = 1.9569;
cm = 0.0342;
ch = -37.1967;
lf = 0.225;
lr = 0.225;
theta = x(4);
% compute derivative
dx = [x(3)* cos(theta); 
      x(3)* sin(theta); 
      -ca *x(3) + (ca*cm)*(u(1)-ch);
      (x(3)/(lf+lr))*tan(u(2))];

xp = (stepSize * dx) + x;

end
