%% twist2rbvel(V) maps the 6-vector twist V to the 4x4 rigid body velocity matrix in homogeneous coordinates V_hat

function V_hat=twist2rbvel(V)
    % TODO: construct the 4x4 rigid body velocity matrix in homogeneous
    % coordinates V_hat from V
    V_hat = zeros(4, 4);
    w_hat = angvel2skew(V(4 : 6));
    v = V(1 : 3);
    
    V_hat = [w_hat, v; zeros(1, 4)];
end