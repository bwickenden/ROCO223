
% clean up matlab before launching script
clear all
close all 
clc
        
u = 0.05;  %Coefficent of viciously
m = 0.314;% pendulum point mass
M = 2;% cart mass
L = 1; % pendulum length
l=0.64;
g = -9.81;% acceleration due to gravity
d = 1;% damping
I = ((1/12)*m*(l^2)); %Moment of Inertial of the pendulum rod at centre of mass

%Coefficients
a0 = 1;
a1 = (m*u)/(I+m*(l/2)^2);
a2 = (-m*g*(l/2))/(I+m*(l/2)^2);
b0 = (m*(l/2))/(I+m*(l/2)^2);
b1 = 0;

%State spaces 
A = [0 1; -a2 -a1; ];
B = [b0; (b1-a1*b0);];
C = [1 0;];
D = 0;

% Inverted config
A_inverted = [0 1; -a2 -a1; ];

%Feedback 
PX =8 *[-1 -1.1];
Feedback = place(A,B,PX);



% time steps of 100ms for integration
timeStep = 0.005;
% total time for simulation
totalTime = 1.5;
% build timepoint vector
tspan = 0:timeStep:totalTime;


titleMessage = 'controlled nonlinear sim of FC pendulum on cart';
disp(titleMessage)


% initial conditions
% located at x=0
% velocity =0
% angle  pi (inverted)
% angular velocity = 0.5 rads-1
y0 = [0; 1];

% use ode to solve with FCPendOnCart with no control force input u
% representing a force controlled pendulum on a cart
% model introduces slight amount of noise to wont stay balanced
[t,y] = ode45(@(t,y)Feedback_function(A, y, B,-Feedback*y),tspan,y0);

% for all time point animate the results
range=1;
len = length(tspan);
kickFlag = zeros(1,len);

% get variables
x = y(:, 1)*0;    % cart positon
th = y(:, 1);   % pendulum angle

% animate pendulum
figure
AnimatePendulumCart(pi+ th,  x, L/2, tspan, range, kickFlag, 'With feedback' );

plot(t,y);
