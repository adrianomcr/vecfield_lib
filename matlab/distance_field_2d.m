
clear all
close all

%Global variables
global r cost Kf

%----- Select the desired curve (alll parametrized in the interval [0, 2*pi]) -----
%----- ---------------------------------------------------------------------- -----
% r = @(s) [2.0*cos(s); 1.0*sin(s)]; %ellipse
r = @(s) [(cos(s).^4+sin(s).^4).^(-0.25).*cos(s); (cos(s).^4+sin(s).^4).^(-0.25).*sin(s)]; %square
% r = @(s) [2*cos(s); 1*sin(2*s)]; %8
% r = @(s) [1*cos(s) + 1*cos(2*s); 1*sin(s)-1*sin(2*s)]; %trifoil
% r = @(s) [4*sin(4*s); 3*cos(3*s)]/3; %fill square
% r = @(s) [1.5*cos(s)-cos(20*s); 1.5*sin(s)-sin(20*s)]/2; %orbital circle

%Distance between a point (p) a another point (r(s)) on the curve
cost = @(p,s) norm(p-r(s));

%Convergence gain of the vector field
Kf = 1;

%List of parameters
sv = linspace(0,2*pi,1000);
%Sample the curve
C = r(sv);

%Define the size of the workspace
ws = [-4 4 -2 2];

%Perform animation at the end? (0 or 1)
ANIMATION = 1;
SPEED = 5; %speed of the simulation


%% Sample field

% Define the grid 
dx = 0.2;
dy = 0.2;
xv = ws(1):dx:ws(2);
yv = ws(3):dy:ws(4);
[X, Y] = meshgrid(xv,yv);
F = 0*X;

%Double loop to comput the field in a grid
for i = 1:1:length(xv)
    x = xv(i);
    for j = 1:1:length(yv)
        y = yv(j);
        
        %Define point
        p = [x; y];
        
        %Compute field
        f = compute_dist_field(p);

        %Save the vector
        Fx(j,i) = f(1);
        Fy(j,i) = f(2);
        
    end
end

% Plot field
figure(1)
quiver(X,Y,Fx,Fy,'b','LineWidth',1.0,'MaxHeadSize',0.8,'AutoScale','on')
hold on
plot(C(1,:),C(2,:),'k','LineWidth',2)
hold off
axis equal
axis(ws)
grid on
title('Distance field 2d')
xlabel('x_1')
ylabel('x_2')


%% Integral line

%Click on image to define an initial condition (right click to cancel)

[x,y,b] = ginput(1);
while (b==1)
    sim = [x ;y];
    dt = 0.02;
    T = 50;
    t = 0:dt:T;
    %Simulate trajectory
    for k = 1:1:(length(t)-1)

        %Compute field
        f = compute_dist_field(sim(:,k));

        %Integrate system
        sim(:,k+1) = sim(:,k) + f*dt;

    end

    % Plot integral line
    hold on
    plot(sim(1,:),sim(2,:),'r','LineWidth',1.5)
    hold off
    [x,y,b] = ginput(1);
end







%% Animation loop (last trajectory)
if(ANIMATION==1)
    figure(2)
    set(2,'Color',[1 1 1])
    quiver(X,Y,Fx,Fy,'b','LineWidth',1.0,'MaxHeadSize',0.8,'AutoScale','on')
    hold on
    plot(C(1,:),C(2,:),'k','LineWidth',2)
    h1 = plot(sim(1,1),sim(2,1),'r','LineWidth',1.5);
    h2 = plot(sim(1,1),sim(2,1),'ro','LineWidth',2,'MarkerSize',12);
    h3 = text(ws(1),ws(3),sprintf('sim time:  %.2f\nreal time:  %.2f\n',0,0));
    h3.VerticalAlignment = 'top';
    hold off
    axis equal
    axis(ws)
    title('Distance field 2d - animation')
    xlabel('x_1')
    ylabel('x_2')
    axis off
 

    tic
    k = find(t>=SPEED*toc,1);
    while(k<=length(t))

        set(h1,'XData',sim(1,1:k),'YData',sim(2,1:k))
        set(h2,'XData',sim(1,k),'YData',sim(2,k))
        
        h3.String = sprintf('sim time:  %.2f\nreal time:  %.2f\n',t(k),toc);

        %Find the next k such that the SPEED factor is respected
        k = find(t>=SPEED*toc,1);
        drawnow
    end
end
    