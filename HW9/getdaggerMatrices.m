function [M_dagger, A_dagger, lambda_dagger, block_matrix] = getdaggerMatrices(M, A)

[r_M, c_M] = size(M);
[r_A, c_A] = size(A);

M_0 = [M, A'; A, zeros(r_A, r_A)];
block_matrix = simplify(inv(M_0));

M_dagger = block_matrix(1:r_M, 1:c_M);
A_dagger = block_matrix(r_M + 1:r_M + r_A, 1:c_A);
lambda_dagger = block_matrix(r_M + 1:r_M + r_A, r_M + 1:r_M + r_A);

end