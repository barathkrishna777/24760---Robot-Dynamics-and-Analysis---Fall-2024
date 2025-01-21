function [a_i, A_i, dA_i] = compute_As(x, contactMode)
    q = x(1:2);
    dq = x(3:4);
    
    a = [q(2); q(1) + q(2) + 1; (q(1)-2)^2 + (q(2)-1)^2 - 2];
    
    A = [0, 1; 1, 1; 2*(q(1)-2), 2*(q(2)-1)];

    dA = [0, 0; 0, 0; 2*dq(1), 2*dq(2)];

    if isempty(contactMode)
        scenario = 0;
    else
        scenario = contactMode(1);
    end

    switch scenario
        case 0
            % Unconstrained
            A_i = [];
            a_i = [];
            dA_i = [];
        case 1
            % a1
            A_i = A(1, :);
            a_i = a(1);
            dA_i = dA(1, :);
        case 2
            % a2
            A_i = A(2, :);
            a_i = a(2);
            dA_i = dA(2, :);
        case 3
            % a3
            A_i = A(3, :);
            a_i = a(3);
            dA_i = dA(3, :);
        case 4
            % a1 and a3
            A_i = [A(1, :); A(3, :)];
            a_i = [a(1); a(3)];
            dA_i = [dA(1, :); dA(3, :)];
    end
end
