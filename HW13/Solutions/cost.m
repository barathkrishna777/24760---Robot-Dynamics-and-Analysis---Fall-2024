function f = cost(x)

    tau = x(:, 5:6);
    f = 0;

    for i = 2:size(x, 1)
        f = f + (0.02*0.5*(tau(i - 1, :)*tau(i - 1, :)' + tau(i, :)*tau(i, :)'));
    end
    
end
