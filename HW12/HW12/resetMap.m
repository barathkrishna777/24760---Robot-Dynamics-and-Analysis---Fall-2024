function [dq_p, p_hat] = resetMap(x, contactMode)
    dq = x(3:4);
    
    [~, A, ~] = compute_As(x, contactMode);

    M = eye(2);    
    blockMatrix = get_dagger_matrices(x, contactMode);
    sol = blockMatrix*[M*dq; zeros(size(A, 1), 1)]; % Solve equation for dq_p and p_hat

    dq_p = sol(1:2);
    p_hat = sol(3:end); 
end
