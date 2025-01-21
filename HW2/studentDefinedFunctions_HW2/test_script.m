%% testing the composition of w and w_hat

w = rand(3, 1);
test_ang = skew2angvel(angvel2skew(w));

disp(['angvel test'': [', num2str((test_ang - w)'), ']']) %% w and test_twist should come out to be the same vector

V = rand(6, 1);
test_twist = rbvel2twist(twist2rbvel(V));

disp(['twist test'': [', num2str((test_twist - V)'), ']']) %% V and test_twist should come out to be the same vector

compare_twist();