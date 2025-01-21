%% Pendulum Examples

% First, set alpha and beta to zero 
global alpha
global beta
alpha = 0;
beta = 0;
close all

%% Example 1, starting from the right, no stabilization, ode45
alpha = 0;
beta = 0;
[ts,qs] = ode45(@penDyn,[0 10],[0;0;1;0]);
animatePen(ts,qs,'r')
%% Compare to ode113
alpha = 0;
beta = 0;
[ts,qs] = ode113(@penDyn,[0 10],[0;0;1;0]);
animatePen(ts,qs,'b')

%% With constraint stabilization
alpha = 1;
beta = 1;
[ts,qs] = ode45(@penDyn,[0 10],[0;0;1;0]);
animatePen(ts,qs,'g')


%% Example 2, starting from the top
close all

%% no stabilization, ode113
alpha = 0;
beta = 0;
[ts,qs] = ode113(@penDyn,[0 10],[1;0;0;1]);
animatePen(ts,qs,'b')

%% with stabilization, ode45
alpha = 1;
beta = 1;
[ts,qs] = ode45(@penDyn,[0 10],[1;0;0;1]);
animatePen(ts,qs,'g')

%% with stabilization, ode113
alpha = 1;
beta = 1;
[ts,qs] = ode113(@penDyn,[0 10],[1;0;0;1]);
animatePen(ts,qs,'k')

%% with stabilization and better error tol, ode45
alpha = 1;
beta = 1;
options = odeset('RelTol',1e-4);
[ts,qs] = ode45(@penDyn,[0 10],[1;0;0;1],options);
animatePen(ts,qs,'m')



%% Now running the pendulum comparison
close all

%% No stabilization, low tolerance
alpha = 0;
beta = 0;
pendulumComparison(1e-3,'ro');

%% No stabilization, higher tolerance
alpha = 0;
beta = 0;
pendulumComparison(1e-4,'bo');

%% No stabilization, even higher tolerance
alpha = 0;
beta = 0;
pendulumComparison(1e-5,'go');

%% with stabilization, low tolerance
alpha = 1;
beta = 1;
pendulumComparison(1e-3,'mo');

%% with stabilization, higher tolerance
alpha = 1;
beta = 1;
pendulumComparison(1e-4,'co');

