function contactMode = compIV(x)
    q = x(1:2);

    contactMode = 0;
    tolerance = 1e-6; % As zero might not be attained
    
    a_1 = q(2);                
    a_2 = q(1) + q(2) + 1;     
    a_3 = (q(1)-2)^2 + (q(2)-1)^2 - 2;  
    
    if (a_2 < tolerance) && (a_1 < tolerance)
        % 2 -> 1
        [dq_p, p_hat] = resetMap(x, 1);
        [~, A_other, ~] = compute_As(x, 2);
        if all(p_hat <= 0) && all(A_other * dq_p >= 0)
            contactMode = 1;
        end

        % 1 -> 2
        [dq_p, p_hat] = resetMap(x, 2);
        [~, A_other, ~] = compute_As(x, 1);
        if all(p_hat <= 0) && all(A_other * dq_p >= 0)
            contactMode = 2;
        end    

    elseif (a_2 < tolerance)
        % 0 -> 2
        [~, p_hat] = resetMap(x, 2);
        if all(p_hat <= 0)
            contactMode = 2;
        end

    elseif (a_3 < tolerance) && (a_1 < tolerance)
        % 1 -> 1,3
        [dq_p, p_hat] = resetMap(x, 4);
        [~, A_other, ~] = compute_As(x, 3);
        if all(p_hat <= 0) && all(A_other * dq_p >= 0)
            contactMode = 4;
        end

    elseif (a_3 < tolerance)
        % 0 -> 3
        [~, p_hat] = resetMap(x, 3);
        if all(p_hat <= 0)
            contactMode = 3;
        end    

    elseif a_1 < tolerance
        % 0 -> 1
        [~, p_hat] = resetMap(x, 1);
        if all(p_hat <= 0)
            contactMode = 1;
        end 
    end
end
