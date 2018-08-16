
%parameters 
u = 0.05;  %Coefficent of viciously
m = 0.314; %Mass 
l = 0.64; %length the the centre of mass 
g = 9.81; % Acceleration due to gravity
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



A_inverted = [0 1; -a2 -a1; ];

A_noninverted = [0 1; a2 -a1; ];


%Observability, controllability and stability 

disp('observability matrix for 2x2 A system matrix system');
CA = C*A;
MXo=[C; CA;]
rank(MXo)

%Controllability 
disp('controllability matrix for 2x2 A system matrix system');
AB = A * B;
MXc=[B AB ]
rank(MXc)

%Inverted config
A_inverted = [0  1; -a2 -a1; ];

% eigen values of non-inverted config 
eig(A_inverted)

%Get non-inverted config 
A_nonInverted = [0 1; a2 -a1; ];

%Eigen values of non-inverted config
eig(A_nonInverted)



