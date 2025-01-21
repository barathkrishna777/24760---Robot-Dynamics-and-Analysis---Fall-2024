function contactMode = compFA(x)
    dq = x(3:4);

    [~, A, dA] = compute_As(x, 3); % Liftoff only possible from mode 3

    [dx, ~] = solveEOM(x, []);
    ddq = dx(3:4);

    FA = A*ddq + dA*dq;

    if FA > 0
        contactMode = 0;
    else
        contactMode = 3;
    end
end
