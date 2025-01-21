clear; clc; close all;
%% Initialization

% Define start and stop times, set an h to keep outputs at constant time
% interval. You should only change h for different requirements.
t = 0;
tfinal = 3;
h = 0.01; % seconds
tspan = t:h:tfinal;

% Initialize state and contact mode
q = [0.2; 1];
dq = [0; 0];

% Initialize arrays for logging data
xout = []; % collection of state vectors at each time
lambdaout = []; % collection of contact forces at each time

% Initialize contact mode
oldContactMode = zeros(0, 1);
disp(['Initialize in mode {', num2str(oldContactMode'), '}.']);
%% Main Loop

for i = 1:length(tspan)

    % Solve for new state and contact forces
    % Your code here
    syms q_p1 q_p2 dq_p1 dq_p2 lambda_p1 lambda_p2 real
    q_p = [q_p1; q_p2];
    dq_p = [dq_p1; dq_p2];
    lambda_p = [lambda_p1; lambda_p2];

    a = compute_a(q_p);
    A = computeA(q_p);

    M = [1, 0; 0, 1];
    N = [0; 9.8];
    eqn1 = M*(dq_p - dq) + h*N + A'*lambda_p == 0;
    eqn2 = q_p - q - h*dq_p == 0;
    eqn3 = a >= 0;
    eqn4 = - lambda_p >= 0;
    eqn5 = - a.*lambda_p == 0;

    eqns = [eqn1, eqn2, eqn3, eqn4, eqn5];
    vars = [q_p, dq_p, lambda_p];
    sols = solve(eqns, vars);
    
    % Log state and contact forces
    % Your code here
    q_p = double([sols.q_p1; sols.q_p2]);
    dq_p = double([sols.dq_p1; sols.dq_p2]);
    lambda_p = double([sols.lambda_p1; sols.lambda_p2]);
    
    % Check new contact mode and determine if there has been a transition
    % Your code here, and display the following message when appropriate
    newContactMode = - lambda_p > 0;
    if ~isequal(newContactMode, oldContactMode)
        disp(['Transition from mode {', num2str(oldContactMode'), '} to mode {', num2str(newContactMode'), '} at t = ', num2str(tspan(i+1)),'.']);
    end    
    % Reset data
    % Your code here
    oldContactMode = newContactMode;
    q = q_p;
    dq = dq_p;
end

disp(['Terminate in mode {', num2str(oldContactMode'), '} at t = ', num2str(tfinal), '.']);

% This function shows animation for the simulation, don't modify it
animateHW12(xout, h);
