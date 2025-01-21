%% tform2adjoint(g) maps the the rigid body transformation g, 
%% in homogeneous coordinates, to the transformation adjoint matrix, Adg.

function Adg=tform2adjoint(g)
    % TODO: construct the 6x6 transformation adjoint matrix Adg from g
    Adg = sym(zeros(6, 6));
    R = sym(zeros(3, 3));
    p_hat = sym(zeros(3, 3));

    R = g(1:3, 1:3);
    p_hat = angvel2skew(g(1:3, 4));

    Adg = [R, p_hat*R; zeros(3, 3), R];
end
