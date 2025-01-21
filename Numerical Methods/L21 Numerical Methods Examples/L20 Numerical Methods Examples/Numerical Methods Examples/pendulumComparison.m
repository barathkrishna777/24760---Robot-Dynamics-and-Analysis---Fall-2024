function pendulumComparison(tol,ls)

% Initial conditions
q0 = [0;0;1;0];
tr = [0 10];
N = 10; % Number of trials to average over
options = odeset('RelTol',tol);


% Plot the error vs how long it took to run
figure(5)
hold on
%ls = 'co';
xlabel('Time')
ylabel('RMS constraint error')


% Then for each method, run it N times and plot the average time & error
t=0;
err=0;
for i=1:N
    tic
    [ts,qs] = ode45(@penDyn,tr,q0,options);
    t = t + toc;
    err = err + rms(sqrt(qs(:,3).^2+qs(:,4).^2)-1);
end
plot(t/N,err/N,ls)
text(t/N+.002,err/N,'45')

t=0;
err=0;
for i=1:N
    tic
    [ts,qs] = ode113(@penDyn,tr,q0,options);
    t = t + toc;
    err = err + rms(sqrt(qs(:,3).^2+qs(:,4).^2)-1);
end
plot(t/N,err/N,ls)
text(t/N+.002,err/N,'113')

t=0;
err=0;
for i=1:N
    tic
    [ts,qs] = ode23(@penDyn,tr,q0,options);
    t = t + toc;
    err = err + rms(sqrt(qs(:,3).^2+qs(:,4).^2)-1);
end
plot(t/N,err/N,ls)
text(t/N+.002,err/N,'23')

t=0;
err=0;
for i=1:N
    tic
    [ts,qs] = ode15s(@penDyn,tr,q0,options);
    t = t + toc;
    err = err + rms(sqrt(qs(:,3).^2+qs(:,4).^2)-1);
end
plot(t/N,err/N,ls)
text(t/N+.002,err/N,'15s')

t=0;
err=0;
for i=1:N
    tic
    [ts,qs] = ode23s(@penDyn,tr,q0,options);
    t = t + toc;
    err = err + rms(sqrt(qs(:,3).^2+qs(:,4).^2)-1);
end
plot(t/N,err/N,ls)
text(t/N+.002,err/N,'23s')

t=0;
err=0;
for i=1:N
    tic
    [ts,qs] = ode23t(@penDyn,tr,q0,options);
    t = t + toc;
    err = err + rms(sqrt(qs(:,3).^2+qs(:,4).^2)-1);
end
plot(t/N,err/N,ls)
text(t/N+.002,err/N,'23t')

t=0;
err=0;
for i=1:N
    tic
    [ts,qs] = ode23tb(@penDyn,tr,q0,options);
    t = t + toc;
    err = err + rms(sqrt(qs(:,3).^2+qs(:,4).^2)-1);
end
plot(t/N,err/N,ls)
text(t/N+.002,err/N,'23tb')


set(gca,'XScale','log','YScale','log')