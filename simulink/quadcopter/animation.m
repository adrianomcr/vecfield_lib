% Simulation

close all
% clear all

%Animation speed
SPEED = 1;

%Get simulated data
t = pos_t.time;
x = pos_t.signals(1).values;
y = pos_t.signals(2).values;
z = pos_t.signals(3).values;
quat = q_ori_t.signals.values;

%Get error signals
D = Errors.signals(1).values;
F_minus_v = Errors.signals(2).values;
alpha = Errors.signals(3).values;

%Get control signals
tau = control_signals.signals(1).values;
omega = [control_signals.signals(2).values';control_signals.signals(3).values';control_signals.signals(4).values'];


%Start the figure for the animation
figure(1)
plot3(0,0,0,'b.')
hold on
plot_frame(eye(4),L,2);
hold off
xlabel('$x_1$','interpreter','latex','FontSize',15)
ylabel('$x_2$','interpreter','latex','FontSize',15)
zlabel('$x_3$','interpreter','latex','FontSize',15)
set(1,'Color',[1 1 1])

%Define the workspace size
DX = max(x)-min(x);
DY = max(y)-min(y);
DZ = max(z)-min(z);
ws = [min(x) max(x) min(y) max(y) min(z) max(z)] + [-DX DX -DY DY -DZ DZ]*0.2;
axis equal
axis(ws)
grid on

hold on
%Plot drone
H = H_from_pq([x(1);x(2);x(3)],quat(1,:));
h_drone = plot_drone(H,L);

%Plot trajectory
h_traj = plot3(x(1),x(2),x(3),'r','LineWidth',2);

%Sample and plot the curve
theta_vec = linspace(0,2*pi,300);
curve = zeros(3,length(theta_vec));
for i = 1:1:length(theta_vec)
    curve(:,i) = my_curve(theta_vec(i), 0, curva);
end
h_curve = plot3(curve(1,:),curve(2,:),curve(3,:),'k-');
hold off


%% Animation loop
pause(0.5)
%Plot time
h_time = text(ws(2),ws(3),ws(6),sprintf('sim time:  %.2f\nreal time:  %.2f\n',0,0));
% h_time.VerticalAlignment = 'top';
tic
k = 1;
while k < length(t)
    
    %Recompute curve (for time dependent cases)
    for i = 1:1:length(theta_vec)
        curve(:,i) = my_curve(theta_vec(i), t(k), curva);
    end
    
    %Update the plot of the curve
    set(h_curve,'XData',curve(1,:),'YData',curve(2,:),'ZData',curve(3,:));
    
    %Update the plot of the drone
    H = H_from_pq([x(k);y(k);z(k)],quat(k,:));
    set_drone(H,L,h_drone);
    %Update the plot of the trajectory
    set(h_traj,'XData',x(1:k),'YData',y(1:k),'ZData',z(1:k));
    %Update the plot of the time
    set(h_time, 'String', sprintf('sim time:  %.2f\nreal time:  %.2f\n',t(k),toc));
    
    %Simulation time controller
    k = find(t>SPEED*toc,1);
    
    drawnow
end




%% Plot trajectory
figure(2)
plot3(curve(1,:),curve(2,:),curve(3,:),'k-')
hold on
plot3(x,y,z,'b-')
hold off
axis equal
axis(ws)
grid on
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
zlabel('$x_3$','interpreter','latex')
title('Trajectory','interpreter','latex')



%% Plot error signals
figure(3)
plot(t,D,'r-','LineWidth',2) %distance to the curve
hold on
plot(t,F_minus_v,'g-','LineWidth',2) %velocity error
plot(t,alpha,'b-','LineWidth',2) %orientation error
hold off
xlim([0 t(end)])
grid on
xlabel('t','interpreter','latex')
ylabel('Error signals','interpreter','latex')
title('Error signals','interpreter','latex','FontSize',18)
leg1 = legend('$D$','$\|F-v\|$','$\beta$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',14);


%% Plot control signals
figure(4)
subplot(2,1,1)
plot(t,tau,'r-','LineWidth',1.0)
xlim([0 t(end)])
grid on
xlabel('t','interpreter','latex')
ylabel('Thrust force','interpreter','latex')
title('Control signal - Thrust','interpreter','latex','FontSize',17)
leg2 = legend('$\tau$');
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',14);
subplot(2,1,2)
plot(t,omega(1,:),'r-','LineWidth',1.0)
hold on
plot(t,omega(2,:),'g-','LineWidth',1.0)
plot(t,omega(3,:),'b-','LineWidth',1.0)
hold off
xlim([0 t(end)])
grid on
xlabel('t','interpreter','latex')
ylabel('Angular rates','interpreter','latex')
title('Control signal - Angular rates','interpreter','latex','FontSize',17)
leg22 = legend('$\omega_x$','$\omega_y$','$\omega_z$');
set(leg22,'Interpreter','latex');
set(leg22,'FontSize',14);


%% Compute ultimate bound
% Delta_a = 0.0;
% Delta_v = Delta_a/Kv;
% Delta_p = (sqrt(2)/1)*tan(pi*Delta_v/(2*vr));

