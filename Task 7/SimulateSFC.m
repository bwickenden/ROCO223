function [y, t, xout] = SimulateSFC(A, B, C, D, K, t, x0)


%get signal length 
len = length(t);

%init output
y = zeros(1,len);
xout = zeros(2,len);

%record the initial state
xout(:, 1) = x0;
x = x0;

%calculate the command 
u(1)= C(1) * x(1) + C(2) * x(2);

%calculate output from theta and thetaDot states
y(1)= C(1) * x(1) + C(2) * x(2) + D(1)* u(1);

%for all remaining data points, simulate state space model using C

for idx = 2:len
    
    %state feedback rule
    u(idx) = -K(1) *x(1) -K(2) * x(2);
    
    %get the duration between updates 
    h = t(idx) - t(idx-1);
    
    %calculate state derivative 
    xdot(1) = A(1,1) * x(1) + A(1,2) * x(2) + B(1)* u(idx);
    xdot(2) = A(2,1) * x(1) + A(2,2) * x(2) + B(2) * u(idx);
    
    %update the state 
    x(1) = x(1) + h *xdot(1);
    x(2) = x(2) + h *xdot(2);
    
    %record the state 
    xout(:, idx) = x;
    
    %calculate output from theta and thetaDot staets only
    y(idx)= C(1) * x(1) + C(2) * x(2) + D(1) * u(idx);
end 


    