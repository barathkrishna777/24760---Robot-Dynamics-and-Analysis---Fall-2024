%% Initialization

clear all; close all; clc;

% States
syms x y theta real

% Physics
syms w h m real

% Local coordinates
q = [x; y; theta;];

% Constraints
a = [
    y - h*cos(theta) - w*sin(theta);
    y - h*cos(theta) + w*sin(theta);
];

%% Problem 1.1

% TODO: Compute A matrix

% Considering that both the contact modes will be active at the time of
% impact
A = sym(zeros(2, 3));
A = jacobian(a, q);

% TODO: Compute M matrix
M = sym(zeros(3, 3));
M = [m, 0, 0;
     0, m, 0;
     0, 0, m/3*(w^2 + h^2)];

%% Problem 1.2

% TODO: Compute dq_plus for the wide block
dq_plus_wide = sym(zeros(3, 1));
[~, A_dagger, ~, ~] = getdaggerMatrices(M, A);
dq_minus_wide = [1; -2; -1];
dq_plus_wide = subs(dq_minus_wide - A_dagger'*A*dq_minus_wide, [m, w, h, x, y, theta], [1, 2, 1, 2, 1, 0]);

% TODO: Compute P_hat for the wide block
P_hat_wide = sym(zeros(2, 1));
P_hat_wide = subs(A_dagger*M*dq_minus_wide, [m, w, h, x, y, theta], [1, 2, 1, 2, 1, 0]);

% TODO: Verify post-impact constraint velocities (A*dq_plus >= 0) for the
% wide block
A_dq_plus_wide = sym(zeros(2,1));
A_dq_plus_wide = A*dq_plus_wide;

% TODO: Verify impulses are valid (P_hat <= 0) for the wide block (No
% calculations need to be done for this part as P_hat_wide has already
% been calculated)
if all(P_hat_wide <= 0)
    disp('All elements of P_hat_wide are less than or equal to zero when block is wide and both contacts modes are assumed to be active.');
else
    disp('Some elements of P_hat_wide are greater than zero when block is wide and both contacts modes are assumed to be active.');
end
disp(P_hat_wide)

%% Problem 1.3

% TODO: Compute dq_plus for the narrow block
dq_plus_narrow = sym(zeros(3, 1));
dq_minus_narrow = [2; -1; -1];
dq_plus_narrow = subs(dq_minus_narrow - A_dagger'*A*dq_minus_narrow, [m, w, h, x, y, theta], [1, 1, 2, 1, 2, 0]);

% TODO: Compute P_hat for the narrow block
P_hat_narrow = sym(zeros(2, 1));
P_hat_narrow = subs(A_dagger * M * dq_minus_narrow, [m, w, h, x, y, theta], [1, 1, 2, 1, 2, 0]);

% TODO: Verify post-impact constraint velocities (A*dq_plus >= 0) are
% either valid or invalid for the narrow block
A_dq_plus_narrow = sym(zeros(2,1));
A_dq_plus_narrow = subs(A*dq_minus_narrow, [m, w, h, x, y, theta], [1, 1, 2, 1, 2, 0]);

% TODO: Verify impulses (P_hat <= 0) are either valid or invalid for the
% narrow block (No calculations need to be done for this part as
% P_hat_narrow has already been calculated)
if all(P_hat_narrow <= 0)
    disp('All elements of P_hat_narrow are less than or equal to zero when block is narrow and both contacts modes are assumed to be active.');
else
    disp('Some elements of P_hat_narrow are greater than zero when block is narrow and both contacts modes are assumed to be active.');
end
disp(P_hat_narrow)

%% Problem 1.4

% Considering that the object will tip over and only the second contact
% mode will be active during impact

A_correct = jacobian(a(2, :), q);
[~, A_dagger_correct, ~, ~] = getdaggerMatrices(M, A_correct);

% TODO: Compute dq_plus for the narrow block with the correct contact mode
dq_plus_correct = sym(zeros(3, 1));
dq_plus_correct = subs(dq_minus_narrow - A_dagger_correct'*A_correct*dq_minus_narrow, [m, w, h, x, y, theta], [1, 1, 2, 1, 2, 0]);

% TODO: Compute P_hat for the narrow block with the correct contact mode
P_hat_correct = sym(zeros(1, 1));
P_hat_correct = subs(A_dagger_correct*M*dq_minus_narrow, [m, w, h, x, y, theta], [1, 1, 2, 1, 2, 0]);

% TODO: Verify post-impact constraint velocities (A*dq_plus >= 0) are
% valid for the narrow block with the correct contact mode
A_dq_plus_correct = sym(zeros(2,1));
A_dq_plus_correct = subs(A_correct*dq_plus_correct, [m, w, h, x, y, theta], [1, 1, 2, 1, 2, 0]);

% TODO: Verify impulses (P_hat <= 0) are valid for the narrow block with
% the correct contact mode (No calculations need to be done for this part
% as P_hat_correct has already been calculated)
if all(P_hat_correct <= 0)
    disp('All elements of P_hat_correct are less than or equal to zero when block is narrow and only the second contact mode is assumed to be active.');
else
    disp('Some elements of P_hat_correct are greater than zero when block is narrow and only the second contact mode is assumed to be active.');
end
disp(P_hat_correct)