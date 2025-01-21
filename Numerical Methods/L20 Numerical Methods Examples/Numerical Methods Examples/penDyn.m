function fout = penDyn(t,q)

%Global parameters
global alpha
global beta
if(isempty(alpha))
    alpha = 0;
end
if(isempty(beta))
    beta = 0;
end


% Break out input for convenience
dx = q(1);
dy = q(2);
dq = [dx;dy];
x  = q(3);
y  = q(4);

% System parameters
m = 1;
g = 9.8;
l = 1;

% Constraints
a = x^2 + y^2 - l^2;
A = [2*x, 2*y];
%A*dq
dA = [2*dx, 2*dy];

% Dynamics
M = [m 0; 0 m];
MA = [M, A'; A 0];
ddqlam = MA^-1 * [0 ; -m*g; -dA*dq-2*alpha*A*dq-beta*beta*a];

% Output
fout = [ddqlam(1:2);dq];