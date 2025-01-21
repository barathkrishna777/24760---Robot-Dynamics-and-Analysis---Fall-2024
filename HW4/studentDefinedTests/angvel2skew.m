%% angvel2skew(w) maps the 3-vector w to the 3x3 skew-symmetric matrix w_hat

function w_hat=angvel2skew(w)
    % TODO: construct the 3x3 skew-symmetric matrix w_hat from w
    w_hat = sym(zeros(3, 3));
    w_hat(1, 2) = -w(3); w_hat(2, 1) = w(3);
    w_hat(1, 3) = w(2); w_hat(3, 1) = -w(2);
    w_hat(2, 3) = -w(1); w_hat(3, 2) = w(1);
end