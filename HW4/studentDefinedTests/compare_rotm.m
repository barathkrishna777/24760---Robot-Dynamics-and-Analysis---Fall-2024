%% The function should return w, t, rotm and rotm_matlab, and rotm and rotm_matlab should be the same.

function [w, t, rotm, rotm_matlab] = compare_rotm()
    % TODO: generate a random unit vector w and rotation amount t
    w = zeros(3, 1);
    t = 0;
    w = rand(3, 1);
    w = w/sqrt(sum(w .^ 2));  % unit vector aLONG AXIS
    t = rand * 2 * pi;  % random angle between 0 and 2*pi
    % TODO: Compute the 3x3 rotation matrix rotm_matlab generated with 
    % Matlab built-in function axang2rotm 
    rotm_matlab = zeros(3, 3);
    rotm_matlab = axang2rotm([w', t]);  % R = e^(omega_hat * theta)
 
    % TODO: Compute the 3x3 rotation matrix rotm generated with generated 
    % with your function angvel2skew and the Matlab function expm
    rotm = zeros(3, 3);
    w_hat = angvel2skew(w);
    rotm = expm(w_hat * t);
end