%% The function should return V, g, V_s_Ad_g, V_s_tform, V_b_Ad_g and V_b_tform.

function [V, g, V_s_Ad_g, V_s_tform, V_b_Ad_g, V_b_tform] = compare_twist()
    % TODO: generate a random unit vector w, rotation amount t, and the
    % twist pitch p
    w = zeros(3, 1);
    t = zeros();
    p = zeros();
    w = rand(3, 1);
    w = w / sqrt(sum(w .^ 2));
    t = rand * 2 * pi;
    p = rand;

    % TODO: compute the velocity of 3-vector v = wp
    v = zeros(3, 1);
    v = p * w;

    % TODO: using w, t, and v, construct a random twist V and compute the 
    % 4x4 rigid body transformation matrix g
    V = zeros(6, 1);
    g = zeros(4, 4);
    V = [v; t * w];  % constructing twist vector
    V_hat = twist2rbvel(V);
    g(1:3, 1:3) = expm(V_hat(1:3, 1:3));  % populating transformation matrix with rotation and translation elements
    g(1:3, 4) = V(1:3);
    g(4,4) = 1;

    % TODO: first, treating V as a body velocity, compute the conversion to 
    % spatial velocity V_s_Ad_g using tform2adjoint and the conversion in 
    % homogeneous coordinates V_s_tform using twist2rbvel and rbvel2twist
    Ad_g = zeros(6, 6);
    V_s_Ad_g = zeros(6, 1);
    V_s_tform = zeros(6, 1);
    Ad_g = tform2adjoint(g);  % adjoint of g

    V_s_Ad_g = Ad_g * V;  % Vs = adjoint(g) * Vb
    V_s_tform_hat = g * V_hat *inv(g);  % Vs_hat = g * Vb_hat * inv(g)
    V_s_tform = rbvel2twist(V_s_tform_hat);

    % TODO: repeat the test the other way, treating V as a spatial velocity 
    % and converting to body velocity and compute V_b_Ad_g and V_b_tform
    V_b_Ad_g = zeros(6, 1);
    V_b_tform = zeros(6, 1);

    V_b_Ad_g = inv(Ad_g) * V;  % Vb = inv(adjoint(g)) * Vs
    V_b_tform_hat = inv(g) * V_hat * g;  % Vb_hat = inv(g) * Vs_hat * g
    V_b_tform = rbvel2twist(V_b_tform_hat);
end