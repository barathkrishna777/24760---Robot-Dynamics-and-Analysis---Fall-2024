%% The function should return w, t, p, g_w, g_w_matlab, g_v, g_v_matlab, g and g_matlab.

function [w, t, p, g_w, g_w_matlab, g_v, g_v_matlab, g, g_matlab] = compare_tform()
    % TODO: generate a random unit vector w, rotation amount t, and the
    % twist pitch p
    w = zeros(3, 1);
    t = zeros();
    p = zeros();
    w = rand(3, 1);
    w = w/sqrt(sum(w .^ 2));
    t = rand * 2 * pi;
    p = rand();

    % TODO: using w and t, compute the 4x4 rigid body transformation matrix 
    % g_w_matlab generated with axang2tform, and g_w generated with your 
    % function twist2rbvel and the Matlab function expm
    g_w_matlab = zeros(4, 4);
    g_w = zeros(4, 4);
    g_w_matlab = axang2tform([w', t]);
    
    V = [p * w; t * w];
    V_hat = twist2rbvel(V);
    g_w(1:3, 1:3) = expm(V_hat(1:3, 1:3));  % dropping the *t since V_hat already accounts for the amount of rotation
    g_w(4,4) = 1;

    % TODO: compute the velocity of 3-vector v = wp
    v = zeros(3, 1);
    v = p * w;

    % TODO: using the pure translation with a velocity of v and the amount 
    % t, compute the 4x4 rigid body transformation matrix g_v_matlab 
    % generated with trvec2tform, and g_v generated with your function 
    % twist2rbvel and the Matlab function expm
    g_v_matlab = zeros(4, 4);
    g_v = zeros(4, 4);
    d = v;  % assuming time to be 1 sec, translation vector is just v * 1sec
    g_v_matlab = trvec2tform(d');
    
    v_twist = V(1:3);
    t_tr = 0;  % since it is pure translation, setting rotation angle to 0
    V_tr = [v_twist; t_tr * w];  % setting the twist vector from the translational and rotational velocities
    V_tr_hat = twist2rbvel(V_tr);  % computing V_hat from V
    g_v = [expm(V_tr_hat(1:3, 1:3)), v_twist; zeros(1, 3), 1];  % g = [R, d; 0 0 0 1]; here, taking d = v * 1sec

    % TODO: using w, t, and v, compute the 4x4 rigid body transformation 
    % matrix g generated with your function twist2rbvel and the Matlab 
    % function expm and g_matlab generated with the composition of 
    % axang2tform and trvec2tform
    g_matlab = zeros(4, 4);
    g = zeros(4, 4);

    g_matlab = g_v_matlab * g_w_matlab;
    g = g_v * g_w;
end