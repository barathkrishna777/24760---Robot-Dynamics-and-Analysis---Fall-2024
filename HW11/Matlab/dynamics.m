function dx = dynamics(x, contactMode)

    [dx, ~] = solveEOM(x, contactMode);

end
