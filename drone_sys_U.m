function [y,time] = drone_sys_U(obj, time)
%% define constants
dt = obj.time_interval;
% time = obj.LogOut.time(end);
state_init = [obj.pos;zeros(9,1)];

%% define input U
syms u1 u2 u3 u4
U = [u1;u2;u3;u4];

%% define symbols for state X
syms x y z phi theta psi1;
syms vx vy vz;
syms w_psi1 w_theta w_phi;

%% X1234 sub states of state X
X1 = [x; y; z];% position
X2 = [vx; vy; vz];% linear velocities
X3 = [phi; theta; psi1];% angles: roll pitch yaw
X4 = [w_phi; w_theta; w_psi1];% angular velocity omega: roll pitch yaw in body frame

%% xdot1 linear velocity
Xdot1 = X2;
%% Xdot2 Linear acceleration
TB = [0; 0; 1]*U(1);% ft U

Rx = [1     0           0;
    0       cos(phi)    -sin(phi);
    0       sin(phi)    cos(phi)];
Ry = [cos(theta) 0          sin(theta);
        0        1          0;
    -sin(theta)  0          cos(theta)];

Rz = [cos(psi1) -sin(psi1)    0;
    sin(psi1)    cos(psi1)    0;
    0           0           1];
R = Rz*Ry*Rx;
FD = -obj.kd .* Xdot1;
Xdot2 = [0;0;-obj.g] + 1/obj.m*R*TB + 1/obj.m*FD;

%% Xdot3 Angular velocities
rot = [1   0   -sin(theta);
    0   cos(phi)  cos(theta)*sin(phi);
    0   -sin(phi)   cos(theta)*cos(phi)];
Xdot3 = inv(rot)*X4;

%% Xdot4 Angular accelerations 
tau = [U(2)
        U(3)
        U(4)];% ft input
Xdot4_p1 = tau.*[obj.I(1,1)^(-1); obj.I(2,2)^(-1); obj.I(3,3)^(-1)];
Xdot4_p2 = - [(obj.I(2,2) - obj.I(3,3))/obj.I(1,1) * X4(2)*X4(3);...
    (obj.I(3,3) - obj.I(1,1))/obj.I(2,2) * X4(1)* X4(3);...
    (obj.I(1,1) - obj.I(2,2))/obj.I(3,3) * X4(1)* X4(2)];
Xdot4 = Xdot4_p1 + Xdot4_p2;

%% summary of state 
X = [X1; X2; X3; X4];
Xdot = [Xdot1; Xdot2; Xdot3; Xdot4];

%% compute and substitude Jacobians 
% Compute the jacobian matrices Aj Bj
Aj = jacobian(Xdot, X);
Bj = jacobian(Xdot, U);

oldvar = [u1 theta phi psi1 w_theta w_phi w_psi1];% vx vy vz
newvar = [9.2*0.2/1 1e-4 1e-4 1e-4 zeros(1,3)];% ft inputs

% carry out subsitution 
% ie. subsituting operating points and constants into jacobian matrices
A = double(subs(Aj,oldvar,newvar));
B = double(subs(Bj,oldvar,newvar));
C = eye(12);
D = zeros(12,4);

%% compute continuous system and discrete system
cont_sys = ss(A,B,C,D);
disc_sys = c2d(cont_sys,dt,'zoh');

% define time and gamma
time = 0:dt:time+dt;

% for u inputs
gamma0 = 0.2*9.2/4/1*ones(4,1);
gamma_t0 = zeros(4,1);
gamma_t2_4 = gamma0.*1.2 - gamma0;
gamma_t4_8 = gamma_t2_4;
gamma_t4_8(4) = -gamma0(4);


U = zeros(4,length(time));
g = gamma_t4_8;
U(:, :) = [1*sum(g);...
0.2*1*(g(1) - g(3));...
0.2*1*(g(2) - g(4));...
0.1*(g(1) - g(2) + g(3) - g(4))].*ones(size(U(:, :)));
g = gamma_t2_4;
U(:, time<=4) = [1*sum(g);...
0.2*1*(g(1) - g(3));...
0.2*1*(g(2) - g(4));...
0.1*(g(1) - g(2) + g(3) - g(4))].*ones(size(U(:, time<=4)));
g = gamma_t0;
U(:, time<=2) = [1*sum(g);...
    0.2*1*(g(1) - g(3));... 
    0.2*1*(g(2) - g(4));...
    0.1*(g(1) - g(2) + g(3) - g(4))].*ones(size(U(:, time<=2)));
U = U';

% Obtain the LTI simulated model y
[y, time] = lsim(disc_sys, U, time, state_init);% y is the sampled X, state_init

% Constraint the z direction position: it cannot go below 0
for i = 1:height(y)
    if y(i,3)<0
        y(i,3)=0;
    end
end
end

