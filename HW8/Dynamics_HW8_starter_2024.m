%% Initialization

clear;clc;close all

% States
syms theta1 theta2 o_x o_y o_theta dtheta1 dtheta2 do_x do_y do_theta ddtheta1 ddtheta2 ddo_x ddo_y ddo_theta real

% Physics
syms m_l L I_l w m_o I_o g real

% Inputs
syms tau_1 tau_2 real

% Constraint forces
syms lambda_1 lambda_2
lambda = [lambda_1; lambda_2];

% Finger coordinates
theta = [theta1;theta2];
dtheta = [dtheta1;dtheta2];
ddtheta = [ddtheta1;ddtheta2];

% Body coordinates
x = [o_x; o_y; o_theta];
dx = [do_x; do_y; do_theta];
ddx = [ddo_x; ddo_y; ddo_theta];

% Local coordinates
q = [theta;x];
dq = [dtheta; dx];
ddq = [ddtheta; ddx];

%% Problem 1.1

% TODO: Compute inertia matrix in local coordinates M
M = sym(zeros(5, 5));

% in planar coordinates
M_l = [m_l, 0, 0; 0, m_l, 0; 0, 0, I_l];

Jb1 = [0, 0; L/2, 0; 1, 0];
Jb2 = [L*sin(theta2), 0; L/2 + L*cos(theta2), L/2; 1, 1];

M_links = simplify(Jb1'*M_l*Jb1 + Jb2'*M_l*Jb2);

M_o = [m_o, 0, 0; 0, m_o, 0; 0, 0, I_o];

% combining the manipulator and object mass matrices to get the net inertia
% matrix of the system
M = [M_links, zeros(2, 3); zeros(3, 2), M_o];

% TODO: Compute Coriolis matrix in local coordinates C
C  = sym(zeros(5, 5));

for i = 1:5
    for j = 1:5
        for k = 1:5
            C(i, j) = C(i, j) + 0.5 * (diff(M(i, j), q(k)) + diff(M(i, k), q(j)) - diff(M(k, j), q(i))) * dq(k);

        end
    end
end
C = simplify(C);

% TODO: Compute nonlinear terms N
N = sym(zeros(5, 1));

V = (m_l*L/2*sin(theta1) + m_l*(L*sin(theta1) + L/2*sin(theta1 + theta2)) + m_o * o_y) * g;  % potential energy

N = simplify(jacobian(V, q)');

% TODO: Compute applied force Y
Y = sym(zeros(5, 1));

Y = [tau_1, tau_2, 0, 0, 0]';

% TODO: Compute A matrix
A = sym(zeros(2, 5));

Bc = [0 1; 1 0; 0 0; 0 0; 0 0; 0 0]; % wrench basis

goc = [1, 0, 0, 0;
       0, 1, 0, -w/2;
       0, 0, 1, 0;
       0, 0, 0, 1];

G = tform2adjoint(inv(goc))'*Bc;

Jb_o = [0, 0, cos(o_theta), sin(o_theta), 0; 0, 0, -sin(o_theta), cos(o_theta), 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 1];

gsc = [cos(o_theta), -sin(o_theta), 0, L*cos(theta1) + L*cos(theta1 + theta2); 
       sin(o_theta), cos(o_theta), 0, L*sin(theta1) + L*sin(theta1 + theta2); 
       0, 0, 1, 0; 
       0, 0, 0, 1];

Js = [0, L*sin(theta1), 0, 0, 0; 0, -L*cos(theta1), 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 0, 0, 0, 0, 0; 1, 1, 0, 0, 0];

Jh = Bc'*tform2adjoint(inv(gsc))*Js;

A = Jh - G'*Jb_o;

% TODO: Compute dA matrix
dA = sym(zeros(2, 5));

for i = 1:size(A, 1)
    for j = 1:size(A, 2)
        for k = 1:length(q)
        dA(i, j) = dA(i, j) + diff(A(i, j), q(k)) * dq(k);  % d/dt (A) computed using chain rule
        end
    end
end
dA = simplify(dA);

% TODO: Compute equations of motion
EOM = sym(zeros(5, 1));

EOM = [simplify(M*ddq + C*dq + N + A'*lambda - Y == 0)];

%% Problem 1.2

% TODO: Compute acceleration ddq_massive
init_cond = [[pi/2, -pi/2, 0.1, 0.2, 0]', zeros(5, 1)];
vals_massive = [200, 0, 0.1, 1, 8.33*1e-4, 0.2, 24, 0.16, -9.81];

ddq_massive = sym(zeros(5, 1));
M = subs(M, [tau_1, tau_2, L, m_l, I_l, w, m_o, I_o, g], vals_massive);
A = subs(A, [tau_1, tau_2, L, m_l, I_l, w, m_o, I_o, g], vals_massive);

lambda_massive = inv(A*inv(M)*A') * (A*inv(M) * (Y - C*dq - N) + dA*dq);
ddq_massive = inv(M) * (Y - C*dq - N - A'*lambda_massive);

% TODO: Compute numerical acceleration ddq_eval_massive
ddq_eval_massive = zeros(5, 1);

lambda_eval_massive = subs(lambda_massive, [q, dq], init_cond);

ddq_eval_massive = eval(subs(subs(subs(ddq_massive, [tau_1, tau_2, L, m_l, I_l, w, m_o, I_o, g], vals_massive), [q, dq], init_cond), lambda_massive, lambda_eval_massive));

%% Problem 1.3

% TODO: Compute massless EOM EOM_massless
EOM_massless = sym(zeros(5, 1));

[M_dagger, A_dagger, lambda_dagger, ~] = getdaggerMatrices(M, A);

EOM_massless = [ddq; lambda] - [M_dagger, A_dagger'; A_dagger, lambda_dagger] * [Y - C*dq - N; -dA*dq] == [zeros(5, 1); zeros(2, 1)];

%% Problem 1.4

% TODO: Compute numerical acceleration ddq_eval_massless
ddq_eval_massless = zeros(5, 1);

ddq_massless = M_dagger*(Y - C*dq - N) + A_dagger'*(-dA*dq);
lambda_massless = A_dagger * (Y - C*dq - N) + lambda_dagger*(-dA*dq);

vals_massless = [200, 0, 0.1, 0, 0, 0.2, 24, 0.16, -9.81];

lambda_eval_massless = subs(subs(lambda_massless, [tau_1, tau_2, L, m_l, I_l, w, m_o, I_o, g], vals_massless), [q, dq], init_cond);
ddq_eval_massless = eval(subs(subs(subs(ddq_massless, [tau_1, tau_2, L, m_l, I_l, w, m_o, I_o, g], vals_massless), [q, dq], init_cond), lambda_massless, lambda_eval_massless));

% TODO: Compute error percentage error_from_massless
error_from_massless = zeros(5, 1);

error_from_massless = abs(ddq_eval_massless - ddq_eval_massive)./ddq_eval_massive * 100;

%% Problem 1.5

% TODO: Compute the updated A matrix A_frictionless
A_frictionless = sym(zeros(1, 5));

Bc_frictionless = [0; 1; 0; 0; 0; 0];

G_frictionless = tform2adjoint(inv(goc))'*Bc_frictionless;

Jh_frictionless = Bc_frictionless'*tform2adjoint(inv(gsc))*Js;

A_frictionless = Jh_frictionless - G_frictionless'*Jb_o;

% TODO: Compute the updated dA matrix dA_frictionless
dA_frictionless = sym(zeros(1, 5));

for i = 1:size(A_frictionless, 1)
    for j = 1:size(A_frictionless, 2)
        for k = 1:length(q)
        dA_frictionless(i, j) = dA_frictionless(i, j) + diff(A_frictionless(i, j), q(k)) * dq(k);  % d/dt (A_frictionless) computed using chain rule
        end
    end
end
dA_frictionless = simplify(dA_frictionless);

% TODO: Compute the updated EOM EOM_frictionless
EOM_frictionless = sym(zeros(5, 1));
lambda_frictionless = lambda_1;

EOM = [simplify(M*ddq + C*dq + N + A_frictionless'*lambda_frictionless - Y == 0)];

%% Problem 1.6

% TODO: Compute massless and frictionless EOM EOM_massless_frictionless
EOM_massless_frictionless = sym(zeros(5, 1));

[M_dagger_frictionless, A_dagger_frictionless, lambda_dagger_frictionless, block_matrix_frictionless] = getdaggerMatrices(M, A_frictionless);

EOM_massless_frictionless = [ddq; lambda_frictionless] - [M_dagger_frictionless, A_dagger_frictionless'; A_dagger_frictionless, lambda_dagger_frictionless] * [Y - C*dq - N; -dA_frictionless*dq] == [zeros(5, 1); 0];

% TODO: Compute the rank of the block matrix
rank_block = 0;

rank_block = rank(subs(subs(block_matrix_frictionless, [tau_1, tau_2, L, m_l, I_l, w, m_o, I_o, g], vals_massless), [q, dq], init_cond));
% this errors as the number of constraints is not sufficient to make up for
% the lack of masses. The matrix is not invertible, and therefore we are
% not able to compute the instantaneous acceleration ddq.

% If the mass of any one link was zero, we could have still computed the
% instantaneous acceleration, since the frictionless contact still offers
% one constraint that can make for the loss of rank due to the mass being
% zero.
