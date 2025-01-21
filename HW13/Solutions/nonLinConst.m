function [c1, c2] = nonLinConst(X)
    q = X(:, 1:2);
    dq = X(:, 3:4);
    tau = X(:, 5:6);

    q_in = [-pi/2; 0];
    dq_in = [0; 0];
    q_f = [pi/2; 0];
    dq_f = [0; 0];

    boundary_conditions = [q(1, :)' - q_in; dq(1, :)' - dq_in; q(end, :)' - q_f; dq(end, :)' - dq_f];

    ddq = [];
    for i = 1:length(X)
        [M, C, N, Y] = computeDynamicMatrices(q(i, :)', dq(i, :)', tau(i, :)');
        ddq = [ddq; (M\(Y - C*dq(i, :)' - N))'];    
    end

    q_t=[];
    dq_t=[];
    for i = 2:length(X)
        dq_t = [dq_t; (dq(i - 1, :)' - dq(i, :)') + 0.02*((ddq(i, :)' + ddq(i - 1, :)')/2)];
        q_t = [q_t; (q(i - 1, :)' - q(i, :)') + 0.02*((dq(i, :)' + dq(i - 1, :)')/2)];
    end
    
    c1 = [];
    c2 = [boundary_conditions; q_t(:); dq_t(:)];
end