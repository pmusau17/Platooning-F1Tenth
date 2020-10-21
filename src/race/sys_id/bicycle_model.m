function [dx,y] = bicycle_model(t,x,u,ca,cm,ch,lf,lr,varargin)
% Vehicle Bicyle Model as described in:
% https://repository.upenn.edu/cgi/viewcontent.cgi?article=1908&context=cis_papers

% Assumes that Slip angle B= 0. Valid at low speeds.

% states
% x(1): x position 
% x(2): y position
% x(3): vehicle linear velocity 
% x(4): vehicle heading

% u(1) is throttle
% u(2) is steering angle

% output equation
y(1) = x(1);
y(2) = x(2);
y(3) = x(3);
y(4) = wrapToPi(x(4));
theta = wrapToPi(x(4));

dx = [x(3)* cos(theta); 
      x(3)* sin(theta); 
      -ca *x(3) + (ca*cm)*(u(1)-ch);
      (x(3)/(lf+lr))*tan(u(2))];
end 