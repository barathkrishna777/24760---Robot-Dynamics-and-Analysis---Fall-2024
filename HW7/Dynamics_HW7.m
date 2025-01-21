%% Problem 1 Initialization

clear;
clc;

syms q1 q2 dq1 dq2 ddq1 ddq2 real
syms m g l real
syms F tau real
q_g = [q1;q2];
dq_g = [dq1;dq2];
ddq_g = [ddq1;ddq2];
I_o = 1/12*m*l^2;

%% Problem 1.1

% TODO: Kinematic energy
T_g = sym(0);
M = [m, 0, 0, 0, 0, 0; 0, m, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, I_o];  % inertial matrix for single link

e11 = [0, l/2, 0, 0, 0, 1]';  % effect on velocity of link 1's COM due to q1
e12 = zeros(6, 1);  % effect on velocity of link 1's COM due to q2

e21 = [0, 0, 0, 0, 0, 0]';  % effect on velocity of link 2's COM due to q1
e22 = [1, 0, 0, 0, 0, 0]';  % effect on velocity of link 2's COM due to q2

Jb1 = [e11 e12];  % Jacobian for first link
Jb2 = [e21 e22];  % Jacobian for second link

%% alternate method of calculating kinetic energy using body velocities etc.
% w1 = [0, 0, dq1]';
% p1 = [-l/2, 0, 0]';
% v1 = -cross(w1, p1);
% Vb1 = [v1; w1];
% 
% w2 = [0, 0, 0]';
% v2 = [dq2, 0, 0]';
% Vb2 = [v2; w2];
% 
% T1 = T(M, Vb1);
% T2 = T(M, Vb2);
%%

T_g = 1/2 * dq_g' * (Jb1' * M * Jb1 + Jb2' * M * Jb2) * dq_g; % total kinetic energy

% TODO: Potential energy
V_g = sym(0);

V1 = m * l/2 * sin(q1) * g;  % m * g * h for link 1
V2 = m * (l + l/2) * sin(q1) * g;  % m * g * h for link 2

V_g = V1 + V2;  % total potential energy

% TODO: Lagrangian in generalized coordinates
L_g = sym(0);

L_g = T_g - V_g;  % Lagrangian = kinetic - potential energy

%% Problem 1.2

% TODO: Applied forces to each generalized coordinate
Y_g = sym(zeros(2, 1));

Y_g = [tau; F];  % external foces

% TODO: Equations of motion by using the Lagrange equations to differentiate the Lagrangian in generalized coordinates
EOM_g1 = sym(zeros(2, 1));

d_dL_ddq_dt_g = jacobian(gradient(L_g, dq_g), q_g) * dq_g + jacobian(gradient(L_g, dq_g), dq_g) * ddq_g;  % using chain rule to compute d/dt (dL/d(dq))

EOM_g1 = simplify(d_dL_ddq_dt_g  - jacobian(L_g, q_g)' - Y_g == 0);  % equation of motion from Lagrangian differentiaion

%% Problem 1.3

% TODO: Inertia tensor
M_g = sym(zeros(2, 2));

M_g = Jb1' * M * Jb1 + Jb2' * M * Jb2;  % inertia tensor for the manipulator from individual links

% TODO: Coriolis matrix
C_g  = sym(zeros(2, 2));

for i = 1:2
    for j = 1:2
        for k = 1:2
            C_g(i, j) = C_g(i, j) + 0.5 * (diff(M_g(i, j), q_g(k)) + diff(M_g(i, k), q_g(j)) - diff(M_g(k, j), q_g(i))) * dq_g(k);  % Coriolis forces...

        end
    end
end

% TODO: Nonlinear terms
N_g = sym(zeros(2, 1));

N_g = [jacobian(V_g, q_g)]';  % gravitational forces...

% TODO: Equations of motion computed directly
EOM_g2 = sym(zeros(2, 1));

EOM_g2 = simplify(M_g * ddq_g + C_g * dq_g + N_g - Y_g == 0);  % equation of motion from M * ddq + C * dq + N - Y = 0

diff_EOM = EOM_g2 - EOM_g1; % Compute the difference
disp('EOM_g2-EOM_g1 =');
disp(vpa(diff_EOM, 4)); % Display the result with 4 decimal places

%% Problem 2 Initialization

% Declare symbolic variables for maximal coordinates and collect into vectors
syms x1 y1 phi_1 x2 y2 phi_2 dx1 dy1 dphi_1 dx2 dy2 dphi_2 ddx1 ddy1 ddphi_1 ddx2 ddy2 ddphi_2 real
syms lambda_1 lambda_2 lambda_3 lambda_4 real
q_m = [x1; y1; phi_1; x2; y2; phi_2];
dq_m = [dx1; dy1; dphi_1; dx2; dy2; dphi_2];
ddq_m = [ddx1; ddy1; ddphi_1; ddx2; ddy2; ddphi_2];
lambda = [lambda_1; lambda_2; lambda_3; lambda_4];
I_o = 1/12*m*l^2;

%% Problem 2.1

% TODO: Constraint function
a_m = sym(zeros(4, 1));

a_m = [l/2*cos(phi_1) - x1; l/2 * sin(phi_1) - y1; l * cos(phi_1) + l/2 * cos(phi_2) - x2; l * sin(phi_1) + l/2 * sin(phi_2) - y2];  % position constraints

% x1 = (l/2)*cos(phi_1), y1 = (l/2)*sin(phi_1); 
% x2 = l*cos(phi_1) + (l/2)*cos(phi_2), y2 = l*sin(phi_1) + (l/2)*sin(phi_2);

% TODO: A as the differential of a
A_m = sym(zeros(4, 6));

A_m = jacobian(a_m, q_m);  % velocity constraints A_m, given as D(a_m)

%% Problem 2.2

% TODO: Kinematic energy
T_m = sym(0);

M_m = [m, 0, 0, 0, 0, 0; 0, m, 0, 0, 0, 0; 0, 0, I_o, 0, 0, 0; 0, 0, 0, m, 0, 0; 0, 0, 0, 0, m, 0; 0, 0, 0, 0, 0, I_o];  % combined inertia matrix of the manipulator

T_m = 1/2 * dq_m' * M_m * dq_m;  % total kinetic energy

% TODO: Potential energy
V_m = sym(0);

V_m = g * (m * y1 + m * y2);  % total potential energy

% TODO: Lagrangian in maximal coordinates
L_m = sym(0);

L_m = T_m - V_m;  % Lagrangian

%% Problem 2.3

% TODO: Applied forces in maximal coordinates
Y_m = sym(zeros(6, 1));

Y_m = [-F*cos(phi_2); -F*sin(phi_2); tau; F*cos(phi_2); F*sin(phi_2); -tau];  % external forces & torques on each link

%% Problem 2.4

% TODO: Equations of motion by using the Lagrange equations to differentiate the Lagrangian in maximal coordinates
EOM_m1 = sym(zeros(6, 1));


d_dL_ddq_dt_m = jacobian(gradient(L_m, dq_m), q_m) * dq_m + jacobian(gradient(L_m, dq_m), dq_m) * ddq_m;  % using chain rule to compute d/dt (dL/d(dq))

EOM_m1 = simplify(d_dL_ddq_dt_m  - jacobian(L_m, q_m)' - Y_m == 0);  % equation of motion from Lagrangian differentiation

%% Problem 2.5

% TODO: Inertia tensor
M_m = sym(zeros(6, 6));

M_m = [m, 0, 0, 0, 0, 0; 0, m, 0, 0, 0, 0; 0, 0, I_o, 0, 0, 0; 0, 0, 0, m, 0, 0; 0, 0, 0, 0, m, 0; 0, 0, 0, 0, 0, I_o];  % combined inertia matrix of the manipulator

% TODO: Coriolis matrix
C_m  = sym(zeros(6, 6));

for i = 1:6
    for j = 1:6
        for k = 1:6
            C_m(i, j) = C_m(i, j) + 0.5 * (diff(M_m(i, j), q_m(k)) + diff(M_m(i, k), q_m(j)) - diff(M_m(k, j), q_m(i))) * dq_m(k);  % coriolis forces...

        end
    end
end

% TODO: Nonlinear terms
N_m = sym(zeros(6, 1));

N_m = [jacobian(V_m, q_m)]';  % gravitational forces...

% TODO: Equations of motion computed directly
EOM_m2 = sym(zeros(6, 1));

EOM_m2 = simplify(M_m * ddq_m + C_m * dq_m + N_m - Y_m == 0);  % equation of motion from M * ddq + C * dq + N - Y = 0

disp('EOM_m2-EOM_m1=');
disp(EOM_m2-EOM_m1);

%% Problem 2.6

% TODO: Constraint forces
lambdaVec = sym(zeros(4, 1));

dA_m_dt = sym(zeros(size(A_m)));  % d/dt (A_m)
for i = 1:size(A_m, 1)
    for j = 1:size(A_m, 2)
        for k = 1:length(q_m)
        dA_m_dt(i, j) = dA_m_dt(i, j) + diff(A_m(i, j), q_m(k)) * dq_m(k);  % d/dt (A_m) computed using chain rule
        end
    end
end

lambda = simplify(inv(A_m * inv(M_m) * A_m') * (A_m * inv(M_m) * (Y_m - C_m * dq_m - N_m) + dA_m_dt * dq_m));  % closed-form solution of Lagrange multipliers

lambdaVec = simplify(A_m' * lambda);  % constraint forces computed as A_m' * lambda

disp('lambdaVec=')
disp(lambdaVec);

%% Problem 2.7 (Optional)

% TODO: q_m = h(q_g)
h = sym(zeros(6, 1));


% TODO: dq_m = H*dq_g
H = sym(zeros(6, 2));

% TODO: Reconstruct
Mhat = sym(zeros(2, 2));
Chat = sym(zeros(2, 2));
Nhat = sym(zeros(2, 1));
Yhat = sym(zeros(2, 1));

% TODO: Equations of motion
EOM_g3 = sym(zeros(2, 1));
