%% The function should return g, V_s_Ad_g, V_s,V_b.

function [g, V_s_Ad_g, V_s, V_b] = compare_twist()
    % TODO: generate a random transformation g
    g = zeros(4, 4);
    
    % generating a random rotation
    R = zeros(3, 3); 
    l = rand(3, 1);
    l = l/sqrt(sum(l .^ 2)); % unit vector for axis of rotation
    theta = rand * 2 * pi;
    axang = [l' theta];
    R = axang2rotm(axang); % inbuilt MATLAB function to generate rotation matrix from an axis and an angle

    % since the R_dot matrix is associated with R and the skew symmetrim
    % matrix form of angular velocity, we can generate a valid R_dot by
    % multiplying R with the skew symmetric form of an aribtrary angular velocity 
    R_dot = zeros(3, 3);
    R_dot = R*angvel2skew(rand(3, 1));

    g = [R, rand(3, 1); zeros(1, 3), 1]; % plugging rotation matrix R and a random translation into g
    g_dot = [R_dot, rand(3, 1); zeros(1, 3), 0];  % plugging rotation matrix derivative R_dot and a random velocity into g_dot, and making the 4x4th element 0
    

    % TODO: compute the spacial velocity V_s_hat
    % Hint use g_dot and g to find both V_s_hat and V_b_hat
    V_s_hat = zeros(4, 4);
    V_s_hat = g_dot / g;
    
    % TODO: compute the body velocity V_b_hat
    V_b_hat = zeros(4, 4);
    V_b_hat = g \ g_dot;
    
    % TODO: Convert the spatial and body velocities
    % in homogeneous coordinates into twists V_s and V_b 
    % with your function rbvel2twist
    V_s = zeros(6, 1);
    V_b = zeros(6, 1);
    
    V_s = rbvel2twist(V_s_hat);
    V_b = rbvel2twist(V_b_hat);

    % TODO: Compute the conversion from twist V_b to 
    % twist V_s_Ad_g using tform2adjoint
    Ad_g = zeros(6, 6);
    V_s_Ad_g = zeros(6, 1);

    Ad_g = tform2adjoint(g);

    V_s_Ad_g = Ad_g * V_b;

    % TODO: compare V_s with V_s_Ad_g 
    % and make sure they are identical.
    %Hint: Round tha values to 2 or 3 decimal places before comapring.
    test = round(V_s, 3) - round(V_s_Ad_g, 3);
    disp(['V_s test'': [', num2str(test'), ']'])

end