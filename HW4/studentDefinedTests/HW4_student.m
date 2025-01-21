clear; clc; close all;

%% PROBLEM 1
% Define symbolic variables
syms q1 q2 q3 q4 q5 q6 real;
q = [q1 q2 q3 q4 q5 q6]';

% Define known frame offsets
lx2 = 320; lx5 = 887; lx6 = 200; lz2 = 680; lz3 = 975; lz4 =200;

% Compute forward  kinematics
% You can reuse your code from HW3
gst = zeros(4, 4);
gs1 = sym(zeros(4, 4));
g12 = sym(zeros(4, 4));
g23 = sym(zeros(4, 4));
g34 = sym(zeros(4, 4));
g45 = sym(zeros(4, 4));
g5t = sym(zeros(4, 4));

% reusing computations from HW3
gs1 = [cos(q1), -sin(q1), 0, 0; sin(q1), cos(q1), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
gs12 = [cos(q2), 0, sin(q2), lx2; 0, 1, 0, 0; -sin(q2), 0, cos(q2), lz2; 0, 0, 0, 1];
gs23 = [cos(q3), 0, sin(q3), 0; 0, 1, 0, 0; -sin(q3), 0, cos(q3), lz3; 0, 0, 0, 1];
gs34 = [1, 0, 0, 0; 0, cos(q4), -sin(q4), 0; 0, sin(q4), cos(q4), lz4; 0, 0, 0, 1];
gs45 = [cos(q5), 0, sin(q5), lx5; 0, 1, 0, 0; -sin(q5), 0, cos(q5), 0; 0, 0, 0, 1];
gs5t = [1, 0, 0, lx6; 0, cos(q6), -sin(q6), 0; 0, sin(q6), cos(q6), 0; 0, 0, 0, 1];

gst = simplify(gs1 * gs12 * gs23 * gs34 * gs45 * gs5t);
%% Problem 1.1

% TODO: Calculate the inverse of Foward Kinematics transformation
gst_inv = zeros(4, 4);
gst_inv = inv(gst);

% TODO: Calculate each portion of the spatial Jacobian and get Js
J1 = sym(zeros(6, 1));
J2 = sym(zeros(6, 1));
J3 = sym(zeros(6, 1));
J4 = sym(zeros(6, 1));
J5 = sym(zeros(6, 1));
J6 = sym(zeros(6, 1));

J1 = simplify(rbvel2twist(diff(gst, q1) * gst_inv));
J2 = simplify(rbvel2twist(diff(gst, q2) * gst_inv));
J3 = simplify(rbvel2twist(diff(gst, q3) * gst_inv));
J4 = simplify(rbvel2twist(diff(gst, q4) * gst_inv));
J5 = simplify(rbvel2twist(diff(gst, q5) * gst_inv));
J6 = simplify(rbvel2twist(diff(gst, q6) * gst_inv));

Js = sym(zeros(6, 6));
Js = [J1, J2, J3, J4, J5, J6];
size(Js)
%% Problem 1.2

% TODO: Calculate adjoint of product of exponential map
xi1_prime = sym(zeros(6, 1));
xi2_prime = sym(zeros(6, 1));
xi3_prime = sym(zeros(6, 1));
xi4_prime = sym(zeros(6, 1));
xi5_prime = sym(zeros(6, 1));
xi6_prime = sym(zeros(6, 1));

% reusing computations from HW3
xi1 = [0, 0, 0, 0, 0, 1]';
xi2 = [-lz2, 0, lx2, 0, 1, 0]';
xi3 = [-(lz2 + lz3), 0, lx2, 0, 1, 0]';
xi4 = [0, lz2 + lz3 + lz4, 0, 1, 0, 0]';
xi5 = [-(lz2 + lz3 + lz4), 0, lx2 + lx5, 0, 1, 0]';
xi6 = [0, lz2 + lz3 + lz4, 0, 1, 0, 0]';

xi1_prime = xi1;
xi2_prime = tform2adjoint(expm(twist2rbvel(xi1) * q1)) * xi2;
xi3_prime = tform2adjoint(expm(twist2rbvel(xi1) * q1) * expm(twist2rbvel(xi2) * q2)) * xi3;
xi4_prime = tform2adjoint(expm(twist2rbvel(xi1) * q1) * expm(twist2rbvel(xi2) * q2) * expm(twist2rbvel(xi3) * q3)) * xi4;
xi5_prime = tform2adjoint(expm(twist2rbvel(xi1) * q1) * expm(twist2rbvel(xi2) * q2) * expm(twist2rbvel(xi3) * q3) * expm(twist2rbvel(xi4) * q4)) * xi5;
xi6_prime = tform2adjoint(expm(twist2rbvel(xi1) * q1) * expm(twist2rbvel(xi2) * q2) * expm(twist2rbvel(xi3) * q3) * expm(twist2rbvel(xi4) * q4) * expm(twist2rbvel(xi5) * q5)) * xi6;

% TODO: Calculate and compare spatial Jacobian
Js_exp = sym(zeros(6, 6));
Js_exp = [xi1_prime, xi2_prime, xi3_prime, xi4_prime, xi5_prime, xi6_prime];

% TODO: Compare to 1.1
disp('Element-wise difference between Js and Js_exp:')
disp(simplify(Js - Js_exp))

%% Problem 1.3

% TODO: Define body twist in initial configuration
Vb = zeros(6, 1);
Vb = [0 1 0 0 0 0]';  % pure linear velocity in +y direction

% TODO: Convert to spatial twist through adjoint, at initial configuration
Vs = zeros(6, 1);
gst0 = subs(gst, q, zeros(6, 1));
Vs = tform2adjoint(gst0) * Vb;

%%Problem 1.4
% TODO: Compute rank of spatial jacobian (singular if rank < 6)
Js0 = zeros(6, 6);
Js0 = subs(Js, q, zeros(6, 1));

rank_Js = rank(Js);
rank_Js0 = rank(Js0);

disp('Rank of Js is:')
disp(rank_Js)

disp('Rank of Js0 is:')
disp(rank_Js0)

if(rank_Js ~= rank_Js0)
    disp("There is a singularity in the configuration!")
end
%% PROBLEM 2
syms theta_1 theta_2 l_1 l_2 m g real

%% 2.1
% TODO: compute the forward kinematicsgstand g_st
g_st=sym(zeros(4, 4));
g_sl1=sym(zeros(4, 4));
g_l1l2=sym(zeros(4, 4));
g_l2t=sym(zeros(4, 4));

g_sl1 = [cos(theta_1) -sin(theta_1) 0 0; sin(theta_1) cos(theta_1) 0 0; 0 0 1 0; 0 0 0 1];
g_l1l2 = [cos(theta_2) -sin(theta_2) 0 l_1; sin(theta_2) cos(theta_2) 0 0; 0 0 1 0; 0 0 0 1];
g_l2t = [1 0 0 l_2; 0 1 0 0; 0 0 1 0; 0 0 0 1];

g_st = simplify(g_sl1 * g_l1l2 * g_l2t);

%% 2.2
% TODO: compute the spatial and body jacobians Js and Jb
J_s=sym(zeros(6, 2));
g_st_inv = inv(g_st);

Js1 = simplify(rbvel2twist(diff(g_st, theta_1) * g_st_inv));
Js2 = simplify(rbvel2twist(diff(g_st, theta_2) * g_st_inv));
J_s = [Js1 Js2];

J_b=sym(zeros(6, 2));
J_b = simplify(inv(tform2adjoint(g_st)) * J_s);
rank(J_b)

%% 2.3
% TODO: compute the body wrench Ft as a function of the configuration
F_t=sym(zeros(6, 1));
F_s = [0 -m*g 0 0 0 -m*g*(l_1 * cos(theta_1) + l_2 * cos(theta_1 + theta_2))]'; % body wrench in S frame

% F_t = (g_st)' * F_s
F_t = simplify(tform2adjoint(g_st)' * F_s);

%% 2.4
% TODO: compute the joint torque tau_pm that counteracts this body wrench
tau_pm=sym(zeros(2, 1));

% tau_pm = (J_b)' * F_t
tau_pm = simplify(J_b' * F_t);
%% 2.5
syms m_1 m_2 real
f1s = [0, -m_1 * g, 0]';
p1s = [l_1/2*cos(theta_1), l_1/2*sin(theta_1), 0]';
f2s = [0, -m_2 * g, 0]';
p2s = [l_1*cos(theta_1) + l_2*cos(theta_1 + theta_2)/2, l_1/2*sin(theta_1) + l_2*sin(theta_1 + theta_2)/2, 0]';

F1s = [f1s' cross(p1s, f1s)']';
F2s = [f2s' cross(p2s, f2s)']';
F_s_new = F1s + F2s;
F_t_new = simplify((tform2adjoint(g_st))' * F_s_new);

% TODO: compute the joint torques tau_lm that needs to be applied to counteract just the weight of the links themselves
tau_lm=sym(zeros(2, 1));

tau_lm = simplify(J_b' * F_t_new);

%% 2.6
% TODO: compute the actuator effort 
energy = sym(zeros(1, 1));

tau_lm_unit = simplify(subs(tau_lm, [m_1, m_2, l_1, l_2, g], [ones(1, 4), 9.81]));
theta = [theta_1, theta_2];

energy = tau_lm_unit' * tau_lm_unit;
tau_fun = matlabFunction(energy, 'Vars', {theta});

options = optimoptions(@fminunc, 'Display', 'iter', 'MaxFunctionEvaluations', 1e5, 'MaxIterations', 1e3);
x_sol = fminunc(tau_fun, [0, 0], options);

disp('Solution found for [theta1 theta2]: ')
disp(x_sol)

disp('Torque values at joints 1 and 2 for solution: ')
disp(eval(subs(tau_lm_unit, [theta_1, theta_2], x_sol)))