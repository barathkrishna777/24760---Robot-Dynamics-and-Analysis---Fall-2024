function inv_block_matrix = get_dagger_matrices(x, contactMode)

    M = eye(2);
    [~, A, ~] = compute_As(x, contactMode);

    Z = zeros(size(A, 1),size(A, 1));
    inv_block_matrix = inv([M, A'; A, Z]);
end
