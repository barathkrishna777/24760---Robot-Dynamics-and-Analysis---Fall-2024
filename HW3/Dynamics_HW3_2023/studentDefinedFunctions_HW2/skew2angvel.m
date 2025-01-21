%% skew2angel(w_hat) is the inverse mapping of angel2skew

function w = skew2angvel(w_hat)
    % TODO: construct the 3-vector w from w_hat
    w = zeros(3, 1);
    w(3) = w_hat(2, 1);
    w(2) = w_hat(1, 3);
    w(1) = w_hat(3, 2);
end

