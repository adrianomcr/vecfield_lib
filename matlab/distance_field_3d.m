
clear all
close all

%Global variables
global r cost Kf

%Define a rotation for the curve
R = Rot('x',rand(1))*Rot('y',rand(1))*Rot('z',rand(1)); %matlab_lib

%----- Select the desired curve (alll parametrized in the interval [0, 2*pi]) -----
%----- ---------------------------------------------------------------------- -----
% r = @(s) [2*cos(s); 1*sin(s); 0*s]; %ellipse
r = @(s) R*[(cos(s).^4+sin(s).^4).^(-0.25).*cos(s); (cos(s).^4+sin(s).^4).^(-0.25).*sin(s);0*s]; %square
% r = @(s) R*[2*cos(s); 1*sin(2*s)]; %8
% r = @(s) R*[1*cos(s) + 1*cos(2*s); 1*sin(s)-1*sin(2*s)]; %trifoil
% r = @(s) R*[4*sin(4*s); 3*cos(3*s)]/3; %fill square
% r = @(s) R*[1.5*cos(s)-cos(20*s); 1.5*sin(s)-sin(20*s)]/2; %orbital circle

%Distance between a point (p) a another point (r(s)) on the curve
cost = @(p,s) norm(p-r(s));

%Convergence gain of the vector field
Kf = 1;


%List of parameters
sv = linspace(0,2*pi,1000);
%Sample the curve
C = r(sv);

%Define the size of the workspace
ws = [-4 4 -2 2 -2 2];

%Perform animation at the end? (0 or 1)
ANIMATION = 1;
SPEED = 5; %speed of the simulation

%%

% Plot curve
figure(1)
plot3(C(1,:),C(2,:),C(3,:),'k','LineWidth',2)
hold off
axis equal
axis(ws)
grid on
title('Distance field 3d')
xlabel('x_1')
ylabel('x_2')
zlabel('x_3')


%% Integral line

%Create a list of randon initial conditions
n = 10; %number of trajectories to be simulated
initial = rand(3,n).*((ws([2 4 6])-ws([1 3 5]))'*ones(1,n))+[ws([1 3 5])']*ones(1,n);

for i = 1:1:length(initial(1,:))
    sim = initial(:,i);
    dt = 0.02;
    T = 20;
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
    plot3(sim(1,:),sim(2,:),sim(3,:),'r','LineWidth',1.5)
    hold off
    drawnow
end







%% Animation loop (last trajectory)
if(ANIMATION==1)
    figure(2)
    set(2,'Color',[1 1 1])
    plot3(C(1,:),C(2,:),C(3,:),'k','LineWidth',2)
    hold on
    h1 = plot3(sim(1,1),sim(2,1),sim(3,1),'r','LineWidth',1.5);
    h2 = plot3(sim(1,1),sim(2,1),sim(3,1),'ro','LineWidth',2,'MarkerSize',12);
    h3 = text(ws(1),ws(3),ws(5),sprintf('sim time:  %.2f\nreal time:  %.2f\n',0,0));
    h3.VerticalAlignment = 'top';
    plot_frame(H_from_pq(ws([1 3 5])',[1 0 0 0]),1,4);
    hold off
    axis equal
    axis(ws)
    title('Distance field 3d - animation')
    xlabel('x_1')
    ylabel('x_2')
    zlabel('x_3')
    axis off
 

    tic
    k = find(t>=SPEED*toc,1);
    while(k<=length(t))

        set(h1,'XData',sim(1,1:k),'YData',sim(2,1:k),'ZData',sim(3,1:k))
        set(h2,'XData',sim(1,k),'YData',sim(2,k),'ZData',sim(3,k))
        
        h3.String = sprintf('sim time:  %.2f\nreal time:  %.2f\n',t(k),toc);

        %Find the next k such that the SPEED factor is respected
        k = find(t>=SPEED*toc,1);
        drawnow
    end
end
    