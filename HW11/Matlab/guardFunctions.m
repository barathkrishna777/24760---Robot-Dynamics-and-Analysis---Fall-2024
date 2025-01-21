function [constraintFcns, isterminal, direction] = guardFunctions(x, contactMode)
    q = x(1:2);

    if isempty(contactMode)
        scenario = 0; % Unconstrained
    else
        scenario = contactMode(1);
    end

    a_1 = q(2);                
    a_2 = q(1) + q(2) + 1;              
    a_3 = (q(1)-2)^2 + (q(2)-1)^2 - 2;  

    [~, lambda] = solveEOM(x, contactMode);

    switch scenario
        
    case 0
        % Unconstrained
        constraintFcns = [a_1, a_2, a_3]; % Possible collision with these 
    case 1
        % Surface 1
        constraintFcns = [a_2, a_3]; % Possible collision with these modes
    case 2
        % Surface 2
        constraintFcns = [a_1, a_3]; % Possible collision with these modes
    case 3
        % Surface 3
        constraintFcns = [a_1, a_2, lambda]; % Possible collision with these modes
    case 4
        % Surfaces 1 and 3
        constraintFcns = [a_2]; % Possible collision with these modes
    end 

    isterminal = ones(1, length(constraintFcns));
    direction = zeros(1, length(constraintFcns));

end
