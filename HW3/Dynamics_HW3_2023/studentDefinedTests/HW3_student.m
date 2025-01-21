clear all; clc; close all;

%% Problem 2.1
% TODO: Writing out by hand the rigid body transformation from the 
% stationary frame to the tool frame in the initial configuration
gst0 = zeros(4, 4);
gst0 = [1, 0, 0, 1407; 0, 1, 0, 0; 0, 0, 1, 1855; 0, 0, 0, 1];
%% Problem 2.2

% Define symbolic variables
syms q1 q2 q3 q4 q5 q6 real;
q = [q1 q2 q3 q4 q5 q6]';

% Define known frame offsets
lx2 = 320; lx5 = 887; lx6 = 200; lz2 = 680; lz3 = 975; lz4 =200;

% TODO: Define rigid body transformations between successive links
gs1 = sym(zeros(4, 4));
g12 = sym(zeros(4, 4));
g23 = sym(zeros(4, 4));
g34 = sym(zeros(4, 4));
g45 = sym(zeros(4, 4));
g5t = sym(zeros(4, 4));

% Computations of g's have been included in the "Calculations.pdf" file
% attached in the zip file
gs1 = [cos(q1), -sin(q1), 0, 0; sin(q1), cos(q1), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
gs12 = [cos(q2), 0, sin(q2), lx2; 0, 1, 0, 0; -sin(q2), 0, cos(q2), lz2; 0, 0, 0, 1];
gs23 = [cos(q3), 0, sin(q3), 0; 0, 1, 0, 0; -sin(q3), 0, cos(q3), lz3; 0, 0, 0, 1];
gs34 = [1, 0, 0, 0; 0, cos(q4), -sin(q4), 0; 0, sin(q4), cos(q4), lz4; 0, 0, 0, 1];
gs45 = [cos(q5), 0, sin(q5), lx5; 0, 1, 0, 0; -sin(q5), 0, cos(q5), 0; 0, 0, 0, 1];
gs5t = [1, 0, 0, lx6; 0, cos(q6), -sin(q6), 0; 0, sin(q6), cos(q6), 0; 0, 0, 0, 1];

% TODO: Compute forward kinematics
gst = sym(zeros(4, 4));
gst = gs1 * gs12 * gs23 * gs34 * gs45 * gs5t;

% TODO: Compute gst(0) with assigning all symbolic variables as 0
gst0_sym = zeros(4, 4);
gst0_sym = subs(gst, [q1, q2, q3, q4, q5, q6], zeros(1, 6));

% TODO: Compare gst0 to gst0_sym
disp('Element-wise difference between gst0 and gst0_sym: ')
disp(gst0 - gst0_sym)

%% Problem 2.3

% TODO: Define joint twists in initial configuration
xi1 = sym(zeros(6, 1));
xi2 = sym(zeros(6, 1));
xi3 = sym(zeros(6, 1));
xi4 = sym(zeros(6, 1));
xi5 = sym(zeros(6, 1));
xi6 = sym(zeros(6, 1));

% Computations of xi's have been included in the "Calculations.pdf" file
% attached in the zip file
xi1 = [0, 0, 0, 0, 0, 1]';
xi2 = [-680, 0, 320, 0, 1, 0]';
xi3 = [-1655, 0, 320, 0, 1, 0]';
xi4 = [0, 1855, 0, 1, 0, 0]';
xi5 = [-1855, 0, 1207, 0, 1, 0]';
xi6 = [0, 1855, 0, 1, 0, 0]';
xi1_hat = twist2rbvel(xi1); 
xi2_hat = twist2rbvel(xi2);
xi3_hat = twist2rbvel(xi3);
xi4_hat = twist2rbvel(xi4);
xi5_hat = twist2rbvel(xi5);
xi6_hat = twist2rbvel(xi6);


% TODO: Compute product of exponentials
gst_exp = sym(zeros(4, 4));
gst_exp = expm(xi1_hat * q1) * expm(xi2_hat * q2) * expm(xi3_hat * q3) * expm(xi4_hat * q4) * expm(xi5_hat * q5) * expm(xi6_hat * q6) * gst0;

% TODO: Compare to 2.2
disp('Element-wise difference between gst and gst_exp:')
disp(simplify(gst - gst_exp))
%% Problem 2.4

% TODO: Define goal position as initial configuration +100mm in +y direction 
gDes = zeros(4, 4);
gDes = gst0;
gDes(2, 4) = 100;

% TODO: Define an optimization cost function
F = sym(0);
syms x real;
gst_fun = matlabFunction(gst, 'Vars', {q});  % this step is necessary as the norm of symbolic matrices are very hard to compute
F = @(x) (norm(gst_fun(x) - gDes));

%% commented out since the minimization problem is the cost function itself, which is already a MATLAB function
% Define an optimization problem
% fun = matlabFunction(F, 'Vars', {q});  

%%
% Call fminunc to solve Inverse Kinematics
x0 = ones(6, 1);
F(x0)
options = optimoptions(@fminunc, 'Display', 'iter', 'MaxFunctionEvaluations', 1e5, 'MaxIterations', 1e3);
x_sol = fminunc(F, x0, options);

% TODO: Compute the Forward Kinematics using the IK solution
gAchieved = zeros(4, 4);
gAchieved = gst_fun(x_sol);

% TODO: Compare gAchieved with gDes
disp('Element-wise difference between gDes and gAchieved:')
disp(gAchieved - gDes)