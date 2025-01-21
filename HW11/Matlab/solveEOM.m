function [dx, lambda] = solveEOM(x, contactMode)
    dq = x(3:4);
    
    [~, A, dA] = compute_As(x, contactMode);

    C = [0, 0;0, 0];
    N = [0; 1*9.8];
    Y = [0; 0];
    
    blockMatrix = get_dagger_matrices(x,contactMode);
    sol = blockMatrix*[Y - N;zeros(size(A,1),1)] - blockMatrix*[C;dA]*dq;
    ddq = sol(1:2);

    if length(sol)>=3
        lambda = sol(3:end);
    else
        lambda = [];
    end

    dx = [dq;ddq];
end
