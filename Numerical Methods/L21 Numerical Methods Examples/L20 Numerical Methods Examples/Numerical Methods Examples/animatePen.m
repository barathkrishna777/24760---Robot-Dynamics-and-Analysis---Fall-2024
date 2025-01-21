function animatePen(ts,qs,c)

% First plot the constraint violation
figure(2)
hold on
plot(ts,sqrt(qs(:,3).^2+qs(:,4).^2)-1,c)
title('a(q) constraint violation')
xlabel('Time')
ylabel('Constraint value')

% Plot the total energy over time 
% (The pendulum should not gain or lose energy)
figure(3)
hold on
plot(ts,9.8*(qs(:,4)+1)+(qs(:,1).^2+qs(:,2).^2)/2,c)
title('Total Energy')
xlabel('Time')
ylabel('Energy')


% Now animate the pendulum dynamics
figure(1)
clf    
hold on

plot(0,0,'bo')
h1=plot([0,qs(1,3)],[0,qs(1,4)],'r');
h2=plot(qs(1,3),qs(1,4),'ro');
plot(sin(0:.1:2*pi+.1),cos(0:.1:2*pi+.1),'k')
axis([-1.5 1.5 -1.5 1.5])
axis equal
    
for i = 1:length(ts)-1
    figure(1)
    set(h1,'Xdata',[0,qs(i,3)]);
    set(h1,'Ydata',[0,qs(i,4)]);
    set(h2,'Xdata',qs(i,3));
    set(h2,'Ydata',qs(i,4));
    axis([-1.5 1.5 -1.5 1.5])
    pause(ts(i+1)-ts(i)) % Not exact timing for animation
end

