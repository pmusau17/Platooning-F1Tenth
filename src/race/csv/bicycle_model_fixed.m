function dx = bicycle_model_fixed(t,x)
% Vehicle Bicyle Model as described in:
% https://repository.upenn.edu/cgi/viewcontent.cgi?article=1908&context=cis_papers

% Assumes that Slip angle B= 0. Valid at low speeds.


u = [1.5;0.266]; % simulates the car going in a circle

ca = 1.633;
cm = 0.2;
ch = 4;
lf = 0.225;
lr = 0.225;
% states
% x(1): x position 
% x(2): y position
% x(3): vehicle linear velocity 
% x(4): vehicle heading


dx = [x(3)* cos(x(4)); x(3)* sin(x(4)); -ca *x(3) + (ca*cm)*(u(1)-ch);(x(3)/(lf+lr))*tan(u(2))];
end 