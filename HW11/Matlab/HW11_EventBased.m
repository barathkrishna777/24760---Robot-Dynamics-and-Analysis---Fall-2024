clear;clc;close all;

% Define start and stop times, set a dt to keep outputs at constant time
% interval
tstart = 0;
tfinal = 5;
dt = 0.01;

% Initialize state and contact mode
q0 = [1; 5];
dq0 = [0; 0];
x0 = [q0; dq0];
contactMode = [];
%% Main Loop

% Tell ode45 what event function to use and set max step size to make sure
% we don't miss a zero crossing
options = odeset('Events', @(t, x) guardFunctions(x, contactMode), 'MaxStep', 0.01);

% Initialize output arrays
tout = tstart;
xout = x0';

t_impact_out = [];
x_impact_out = [];
i_impact_out = [];

% Main simulation loop
while tstart < tfinal
    % Initialize simulation time vector
    tspan = [tstart : dt : tfinal];

    if(length(tspan) == 1)
        break
    end
    
    % Simulate with ode45
    [t, x, te, xe, ie] = ode45(@(t, x) dynamics(x, contactMode), tspan, x0, options);

    % Sometimes the events function will record a nonterminal event if the
    % initial condition is a zero. We want to ignore this, so we will only
    % use the last row in the terminal state, time, and index.
    if ~isempty(ie)
        te = te(end, :);
        xe = xe(end, :);
        ie = ie(end, :);
    end
    
    % Log output
    nt = length(t);
    tout = [tout; t(2:nt)];
    xout = [xout; x(2:nt,:)];
    t_impact_out = [t_impact_out; te];
    x_impact_out = [x_impact_out; xe];
    i_impact_out = [i_impact_out; ie];
    
    % Quit if simulation completes
    if isempty(ie) 
        disp('Final time reached');
        break; % abort if simulation has completed
    end

    disp("state vector at impact/liftoff: ")
    disp(xe)
    disp("time of impact/liftoff: ")
    disp(te)
    disp("mode: ")
    disp(ie)
    
    % If flag was caused by a_i < 0 (i not in contact mode), compute the
    % proper contact mode via IV complemetarity
    
    if ie < 4
        contactMode = compIV(xe');
        [dq_p, p_hat] = resetMap(xe', contactMode);
        xe(3:4) = dq_p;
    
    % Check to see if there should be liftoff (positive lambda), if so
    % compute the proper contact mode via FA complementarity
    else
        contactMode = compFA(xe');
        dq_p = xe(3:4);
    end

    options = odeset('Events', @(t, x) guardFunctions(x, contactMode), 'MaxStep', 0.01);

    % Update initial conditions for next iteration
    x0 = xe';
    tstart = t(end);
    
    % Stop if the particle comes to a rest
    if all(abs(dq_p) < 1e-6)
        break;
    end
end

% This function shows animation for the simulation, don't modify it
animateHW11(xout, dt);
