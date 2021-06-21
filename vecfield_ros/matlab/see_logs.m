

PLOT_GRAPHS = 1;
SPEED = 4; %speed factor of the animation


FONT_SIZE_LABEL = 12;
FONT_SIZE_TITLE = 15;
FONT_SIZE_LEGEND = 10;

M1 = dlmread('/tmp/states.txt'); 
M2 = dlmread('/tmp/controls.txt');
M3 = dlmread('/tmp/acro_cmd.txt');
M4 = dlmread('/tmp/sensors.txt');


% States
x = M1(:,1:end-1)';
p = M1(:,1:3)';
v = M1(:,4:6)';
q = M1(:,7:10)';
o = M1(:,11:13)';
t = M1(:,end)';

%Compute euler angles
angles = [];
for k = 1:1:length(x(7,:))
    angles(:,k) = fliplr(quat2eul(x(7:10,k)'));
end

ws = [min(p(1,:)) max(p(1,:)) min(p(2,:)) max(p(2,:)) min(p(3,:)) max(p(3,:))];
ws = ws + [-1 1 -1 1 -1 1];

% Controls
% tau = M2(:,1)';
% omega = M2(:,2:4)';
u = M2(:,1:4)';
% t = M2(:,end)';


% Acro cmd
tau_r = M3(:,1)';
omega_r = M3(:,2:4)';
% t = M3(:,end)';


%Sensors
gyro = M4(:,1:3)';
accel = M4(:,4:6)';
gps = M4(:,7:9)';
bar = M4(:,10)';
% t = M4(:,end)';






L = 0.5;

figure(100)
plot3(0,0,0,'k')
set(100,'Position',[549 98 773 420])
set(100,'Color',[1 1 1])
view(30,30)
hold on
plot_frame(eye(4),0.5,2);
H = H_from_pq(p(:,1),q(:,1));
h_drone = plot_drone_sim(H,L);
h_traj = plot3(p(1,1),p(2,1),p(3,1),'b-','LineWidth',1);
h_text = text(ws(2),ws(4),ws(6),sprintf('sim time:  %.2f\nreal time:  %.2f\n',0,0));
h3.VerticalAlignment = 'top';
hold off
axis equal
axis(ws)
grid on
xlabel('$x$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
ylabel('$y$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
zlabel('$z$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
title('Animated results','Interpreter','latex','FontSize',FONT_SIZE_TITLE)

dt = mean(diff(t));

k = 1;
% for k = 1:1:length(t)
pause(1);
tic
while k <= length(t)
    
    H = H_from_pq(p(:,k),q(:,k));
    set_drone_sim(H,L,h_drone);
    set(h_traj,'XData',p(1,1:k),'YData',p(2,1:k),'ZData',p(3,1:k))
    set(h_text,'String',sprintf('sim time:  %.2f\nreal time:  %.2f\n',t(k),toc));
    
    k = find(t>SPEED*toc,1);
    
    drawnow
%     pause(dt)
end




if (PLOT_GRAPHS == 1)
    
    
%     
%     figure(1)
%     plot3(p(1,:),p(2,:),p(3,:),'b')
%     set(1,'Position',[180   180   560   420])
%     axis equal
%     grid on
%     xlabel('Position','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
%     xlabel('$x$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
%     ylabel('$y$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
%     zlabel('$z$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)

    
    %Position
    figure(1)
    subplot(3,1,1)
    plot(t,x(1,:),'b','LineWidth',1)
    hold on
    plot(t,gps(1,:),'r','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$x$ (m)','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    l = legend('ground truth','GPS');
    l.Interpreter = 'latex';
    l.FontSize = FONT_SIZE_LEGEND;
    title('Position','Interpreter','latex','FontSize',FONT_SIZE_TITLE)
    
    subplot(3,1,2)
    plot(t,x(2,:),'b','LineWidth',1)
    hold on
    plot(t,gps(2,:),'r','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$y$ (m)','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    l = legend('ground truth','GPS');
    l.Interpreter = 'latex';
    l.FontSize = FONT_SIZE_LEGEND;
    
    subplot(3,1,3)
    plot(t,x(3,:),'b','LineWidth',1)
    hold on
    plot(t,gps(3,:),'r','LineWidth',1)
    plot(t,bar,'g','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$z$ (m)','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    l = legend('ground truth','GPS', 'barometer');
    l.Interpreter = 'latex';
    l.FontSize = FONT_SIZE_LEGEND;
    
    
    %Linear velocities
    figure(2)
    subplot(3,1,1)
    plot(t,x(4,:),'b','LineWidth',1)
    hold on
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$v_x^w \ (\mathrm{\frac{m}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    title('Velocities (world)','Interpreter','latex','FontSize',FONT_SIZE_TITLE)
    
    subplot(3,1,2)
    plot(t,x(5,:),'b','LineWidth',1)
    hold on
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$v_y^w \ (\mathrm{\frac{m}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    
    subplot(3,1,3)
    plot(t,x(6,:),'b','LineWidth',1)
    hold on
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$v_z^w \ (\mathrm{\frac{m}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    
    
    %Euler angles
    figure(3)
    subplot(3,1,1)
    plot(t,angles(1,:),'b','LineWidth',1)
    hold on
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$\phi \ (\mathrm{\frac{m}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    title('Euler angles (world)','Interpreter','latex','FontSize',FONT_SIZE_TITLE)
    
    subplot(3,1,2)
    plot(t,angles(2,:),'b','LineWidth',1)
    hold on
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$\theta \ (\mathrm{\frac{m}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    
    subplot(3,1,3)
    plot(t,angles(3,:),'b','LineWidth',1)
    hold on
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$\psi \ (\mathrm{\frac{m}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    
    %Angular velocities
    figure(4)
    subplot(3,1,1)
    plot(t,x(11,:),'b','LineWidth',1)
    hold on
    plot(t,omega_r(1,:),'r','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$\omega_x^b \ (\mathrm{\frac{rad}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    l = legend('executed','reference');
    l.Interpreter = 'latex';
    l.FontSize = FONT_SIZE_LEGEND;
    title('Angular velocities (body)','Interpreter','latex','FontSize',FONT_SIZE_TITLE)
    
    subplot(3,1,2)
    plot(t,x(12,:),'b','LineWidth',1)
    hold on
    plot(t,omega_r(2,:),'r','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$\omega_y^b \ (\mathrm{\frac{rad}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    
    subplot(3,1,3)
    plot(t,x(13,:),'b','LineWidth',1)
    hold on
    plot(t,omega_r(3,:),'r','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$\omega_z^b \ (\mathrm{\frac{rad}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    
    
    
    
    %Propeller controls
    figure(5)
    plot(t,u(1,:),'r','LineWidth',1)
    hold on
    plot(t,u(2,:),'g','LineWidth',1)
    plot(t,u(3,:),'b','LineWidth',1)
    plot(t,u(4,:),'k','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$u_i \ (\mathrm{\frac{rad}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    l = legend('$u_1$','$u_2$','$u_3$','$u_4$');
    l.Interpreter = 'latex';
    l.FontSize = FONT_SIZE_LEGEND;
    title('Control signals (propellers)','Interpreter','latex','FontSize',FONT_SIZE_TITLE)
    
    
    
        
    %IMU
    figure(6)
    subplot(2,1,1)
    plot(t,gyro(1,:),'r','LineWidth',1)
    hold on
    plot(t,gyro(2,:),'g','LineWidth',1)
    plot(t,gyro(3,:),'b','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$\omega \ (\mathrm{\frac{rad}{s}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    l = legend('$\omega_x$','$\omega_y$','$\omega_z$');
    l.Interpreter = 'latex';
    l.FontSize = FONT_SIZE_LEGEND;
    title('Gyro','Interpreter','latex','FontSize',FONT_SIZE_TITLE)
    
    subplot(2,1,2)
    plot(t,accel(1,:),'r','LineWidth',1)
    hold on
    plot(t,accel(2,:),'g','LineWidth',1)
    plot(t,accel(3,:),'b','LineWidth',1)
    hold off
    grid on
    xlim([0 t(end)])
    ylabel('$acc \ (\mathrm{\frac{m}{s^2}})$','Interpreter','latex','FontSize',FONT_SIZE_LABEL)
    l = legend('$acc_x$','$acc_y$','$acc_z$');
    l.Interpreter = 'latex';
    l.FontSize = FONT_SIZE_LEGEND;
    title('Accelerometer','Interpreter','latex','FontSize',FONT_SIZE_TITLE)
    
end





