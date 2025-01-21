%% tform2adjoint(g) maps the the rigid body transformation g, 
%% in homogeneous coordinates, to the transformation adjoint matrix, Adg.

function Adg=tform2adjoint(g)
    R = g(1:3,1:3);
    p = g(1:3,4);
    Adg(1:3,1:3) = R;
    Adg(1:3,4:6) = angvel2skew(p)*R;
    Adg(4:6,4:6) = R;
end
