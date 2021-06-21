

close all;


dec_rate = 10;

E = 2000*0;
% Obtain variables related to the UAV
time = result.time(1:dec_rate:end-E);

V_North = result.signals(1).values(1:dec_rate:end-E,1);
V_East = result.signals(1).values(1:dec_rate:end-E,2);
P_North = result.signals(1).values(1:dec_rate:end-E,3);
P_East = result.signals(1).values(1:dec_rate:end-E,4);
Psi_heading = result.signals(1).values(1:dec_rate:end-E,5);
h = result.signals(1).values(1:dec_rate:end-E,6);

vt_ref = result.signals(1).values(1:dec_rate:end-E,7);
h_ref = result.signals(1).values(1:dec_rate:end-E,8);
heading_ref = result.signals(1).values(1:dec_rate:end-E,9);
x_ref = result.signals(1).values(1:dec_rate:end-E,10);
y_ref = result.signals(1).values(1:dec_rate:end-E,11);
theta_c = result.signals(1).values(1:dec_rate:end-E,12);

bank_angle = result_phi.signals(1).values(1:dec_rate:end-E,2);
pitch_angle = result_pitch.signals(1).values(1:dec_rate:end-E,1);

% % Obtain variables related to the UAV
% time = result.time(250000:dec_rate:end);
% 
% V_North = result.signals(1).values(250000:dec_rate:end,1);
% V_East = result.signals(1).values(250000:dec_rate:end,2);
% P_North = result.signals(1).values(250000:dec_rate:end,3);
% P_East = result.signals(1).values(250000:dec_rate:end,4);
% Psi_heading = result.signals(1).values(250000:dec_rate:end,5);
% h = result.signals(1).values(250000:dec_rate:end,6);
% 
% vt_ref = result.signals(1).values(250000:dec_rate:end,7);
% h_ref = result.signals(1).values(250000:dec_rate:end,8);
% heading_ref = result.signals(1).values(250000:dec_rate:end,9);
% x_ref = result.signals(1).values(250000:dec_rate:end,10);
% y_ref = result.signals(1).values(250000:dec_rate:end,11);
% theta_c = result.signals(1).values(250000:dec_rate:end,12);
% 
% bank_angle = result_phi.signals(1).values(250000:dec_rate:end,2);
% pitch_angle = result_pitch.signals(1).values(250000:dec_rate:end,1);






% Print the trajectory of the UAV
% figure(1)
figure('units','normalized','outerposition',[0 0 1 1])
plot3(P_East/1e3,P_North/1e3,h/1e3,'-','Color',[1 0 0],'LineWidth',1);
hold on;
if (curve == 1) % Saddle / Ellipse
    px = (a*cos(0:.01:2*pi)+centro_curva(1))/1e3;
    py = (b*sin(0:.01:2*pi)+centro_curva(2))/1e3;
    pz = (Heq - Ksad*(a^-2*(px*1e3).^2-1))/1e3;
    plot3( px, py, pz,'k');
elseif(curve == 2) % Shape
%     load('./curve_shape.mat','curve_shape');
%     px = curve_shape(1,:)*scale_fac/1e3;
%     py = curve_shape(2,:)*scale_fac/1e3;
%     pz = ones(size(curve_shape(1,:)))*Heq/1e3;
% %     plot3(curve_shape(1,:)/1e3, curve_shape(2,:)/1e3, Heq*ones(1,length(curve_shape(1,:)))/1e3, 'k','LineWidth',2)  
%     plot3(px, py, pz, 'k--','LineWidth',1)
    
    th = 0:2*pi/200:2*pi;
    r = 1./((a*cos(th).^4 - b*cos(th).^2.*sin(th).^2 + c*sin(th).^4).^0.25);
    px = r.*cos(th)*scale_fac/1e3;
    py = r.*sin(th)*scale_fac/1e3;
    pz = ones(size(px))*Heq/1e3;
    plot3(px, py, pz, 'k--','LineWidth',1)
elseif(curve == 3) % Random
    th = (0:2*pi/200:2*pi) + 2*pi/200/2;
    A = b*cos(th).^2.*sin(th).^2;
    B = a*cos(th).^2 + c*sin(th).^2;
    C = -1;
    m = -B + sqrt(B.^2-4.*A.*C); m = m./(2*A);
    r = sqrt(m);
    px = r.*cos(th);
    py = r.*sin(th);
    pz = (Heq/1e3)*ones(size(th));
    plot3(px, py, pz, 'k--','LineWidth',1)

elseif(curve == 4) % Polynomial
    px = -1:0.05:1;
    py = a*px.^3 + b*px.^2 + c*px.^1 + d*px.^0;
    pz = (Heq/1e3)*ones(size(px));
    plot3(px, py, pz, 'k--','LineWidth',1)    
end
% legend('Aircraft','Target');
plot3(P_East(1)/1e3,P_North(1)/1e3,h(1)/1e3,'o','Color',[1 0 0],'LineWidth',2,'MarkerSize',12);
if (curve ~= 4)
%     plot3( (LIM_CENTER*cos(0:.01:2*pi)+centro_curva(1))/1e3,(LIM_CENTER*sin(0:.01:2*pi)+centro_curva(2))/1e3,Heq/1e3*ones(1,length(sin(0:.01:2*pi))),'k--');
end
hold off;
xlabel('East Position (km)');
ylabel('North Position (km)');
zlabel('Height (km)');
grid on;
% title('Trajectory');
axis equal;




theta = Psi_heading'*pi/180-pi/2;
p = [P_East'; P_North'; h']/1e3;
roll = -bank_angle'*pi/180;
pitch = pitch_angle'*pi/180;
axis equal
w_s = [min(P_East) max(P_East) min(P_North) max(P_North) min(h) max(h)]/1e3 + [-1 1 -1 1 -1 1]*0.1;
axis(w_s);


% ANIMATION
% ----------  ----------  ----------  ----------  ----------  ----------  ----------
% figure(1)
hold on
h1 = fill3(0,0,0,'b','LineWidth',2); h1.LineStyle = 'none';
h2 = fill3(0,0,0,'b','LineWidth',2); h2.LineStyle = 'none';
h3 = fill3(0,0,0,'b','LineWidth',2); h3.LineStyle = 'none';
h4 = fill3(0,0,0,'b','LineWidth',2); h4.LineStyle = 'none';
h5 = fill3(0,0,0,'b','LineWidth',2); h5.LineStyle = 'none';
h6 = fill3(0,0,0,'b','LineWidth',2); h6.LineStyle = 'none';

h7 = fill3(0,0,0,'b','LineWidth',2); h7.LineStyle = 'none';
h8 = fill3(0,0,0,'b','LineWidth',2); h8.LineStyle = 'none';
h9 = fill3(0,0,0,'b','LineWidth',2); h9.LineStyle = 'none';
grid on
hold off
uav_e = 0.07;
uav_size = 20; %uav_size = uav_size^1;
UAV0 = [[0 0.5 0 0.2   0 0.5 0 0.2];[1 0 -1 0   1 0 -1 0]; [1 1 1 1   -1 -1 -1 -1]*uav_e]/uav_size;
TAIL0 = [[0.2 -0.7 -0.7 -0.55 -0.4 0.2     0.2 -0.7 -0.7 -0.55 -0.4 0.2];[1 1 1 1 1 1     -1 -1 -1 -1 -1 -1]*uav_e;[-1 -1 3 3 1 1     -1 -1 3 3 1 1]*uav_e]/uav_size;
k = 1;
R = [cos(theta(k)) sin(theta(k)) 0; -sin(theta(k)) cos(theta(k)) 0; 0 0 1];
Rroll = [1 0 0; 0 cos(roll(k)) sin(roll(k)); 0 -sin(roll(k)) cos(roll(k))];
Rpitch = [cos(pitch(k)) 0 -sin(pitch(k)); 0 1 0; sin(pitch(k)) 0 cos(pitch(k))];
UAV = R*Rpitch*Rroll*UAV0;
TAIL = R*Rpitch*Rroll*TAIL0;
set(h1,'XData',UAV(1,1:4)+p(1,k),'YData',UAV(2,1:4)+p(2,k),'ZData',UAV(3,1:4)+p(3,k));
set(h2,'XData',UAV(1,5:8)+p(1,k),'YData',UAV(2,5:8)+p(2,k),'ZData',UAV(3,5:8)+p(3,k));
set(h3,'XData',UAV(1,[1 5 6 2])+p(1,k),'YData',UAV(2,[1 5 6 2])+p(2,k),'ZData',UAV(3,[1 5 6 2])+p(3,k));
set(h4,'XData',UAV(1,[2 6 7 3])+p(1,k),'YData',UAV(2,[2 6 7 3])+p(2,k),'ZData',UAV(3,[2 6 7 3])+p(3,k));
set(h5,'XData',UAV(1,[3 7 8 4])+p(1,k),'YData',UAV(2,[3 7 8 4])+p(2,k),'ZData',UAV(3,[3 7 8 4])+p(3,k));
set(h6,'XData',UAV(1,[4 8 5 1])+p(1,k),'YData',UAV(2,[4 8 5 1])+p(2,k),'ZData',UAV(3,[4 8 5 1])+p(3,k));
set(h6,'XData',UAV(1,[4 8 5 1])+p(1,k),'YData',UAV(2,[4 8 5 1])+p(2,k),'ZData',UAV(3,[4 8 5 1])+p(3,k));
set(h7,'XData',TAIL(1,1:6)+p(1,k),'YData',TAIL(2,1:6)+p(2,k),'ZData',TAIL(3,1:6)+p(3,k));
set(h8,'XData',TAIL(1,7:12)+p(1,k),'YData',TAIL(2,7:12)+p(2,k),'ZData',TAIL(3,7:12)+p(3,k));
set(h9,'XData',TAIL(1,[1 2 8 7])+p(1,k),'YData',TAIL(2,[1 2 8 7])+p(2,k),'ZData',TAIL(3,[1 2 8 7])+p(3,k));
pause(1)
set(1,'Position',[0.3542 0.0324 0.6354 0.6806])
for k = 1:5:round(length(p(1,:))/1)
    R = [cos(theta(k)) sin(theta(k)) 0; -sin(theta(k)) cos(theta(k)) 0; 0 0 1];
    Rroll = [1 0 0; 0 cos(roll(k)) sin(roll(k)); 0 -sin(roll(k)) cos(roll(k))];
    Rpitch = [cos(pitch(k)) 0 -sin(pitch(k)); 0 1 0; sin(pitch(k)) 0 cos(pitch(k))];
    UAV = R*Rpitch*Rroll*UAV0;
    TAIL = R*Rpitch*Rroll*TAIL0;


    set(h1,'XData',UAV(1,1:4)+p(1,k),'YData',UAV(2,1:4)+p(2,k),'ZData',UAV(3,1:4)+p(3,k));
    set(h2,'XData',UAV(1,5:8)+p(1,k),'YData',UAV(2,5:8)+p(2,k),'ZData',UAV(3,5:8)+p(3,k));
    set(h3,'XData',UAV(1,[1 5 6 2])+p(1,k),'YData',UAV(2,[1 5 6 2])+p(2,k),'ZData',UAV(3,[1 5 6 2])+p(3,k));
    set(h4,'XData',UAV(1,[2 6 7 3])+p(1,k),'YData',UAV(2,[2 6 7 3])+p(2,k),'ZData',UAV(3,[2 6 7 3])+p(3,k));
    set(h5,'XData',UAV(1,[3 7 8 4])+p(1,k),'YData',UAV(2,[3 7 8 4])+p(2,k),'ZData',UAV(3,[3 7 8 4])+p(3,k));
    set(h6,'XData',UAV(1,[4 8 5 1])+p(1,k),'YData',UAV(2,[4 8 5 1])+p(2,k),'ZData',UAV(3,[4 8 5 1])+p(3,k));
    set(h6,'XData',UAV(1,[4 8 5 1])+p(1,k),'YData',UAV(2,[4 8 5 1])+p(2,k),'ZData',UAV(3,[4 8 5 1])+p(3,k));
    set(h7,'XData',TAIL(1,1:6)+p(1,k),'YData',TAIL(2,1:6)+p(2,k),'ZData',TAIL(3,1:6)+p(3,k));
    set(h8,'XData',TAIL(1,7:12)+p(1,k),'YData',TAIL(2,7:12)+p(2,k),'ZData',TAIL(3,7:12)+p(3,k));
    set(h9,'XData',TAIL(1,[1 2 8 7])+p(1,k),'YData',TAIL(2,[1 2 8 7])+p(2,k),'ZData',TAIL(3,[1 2 8 7])+p(3,k));
%     pause(0.1)

    set(1,'Position',[0.3542 0.0324 0.6354 0.6806])
    drawnow
end
% ----------  ----------  ----------  ----------  ----------  ----------  ----------

% % 
% % %%
% % % % alpha(x,y) = a*x^4 - b*x^2*y^2 + c*y^4 - 1000 = 0;
% % a = 1;
% % b = 2;
% % c = 1;
% % t = 0:2*pi/200:2*pi; 
% % r = (1000./(a*cos(t).^4-b*cos(t).^2.*sin(t).^2+c*sin(t).^4)).^0.25;
% % x = r.*cos(t);
% % y = r.*sin(t);
% % plot(x,y)
% % axis equal
% % axis([-1 1 -1 1]*7)
% % grid on
% % 
% % 
% % 
% % x = -20:0.1:20;
% % y = -20:0.1:20;
% % [X,Y] = meshgrid(x,y);
% % 
% % 
% % ALPHA = X*0;
% % for i = 1:1:length(x)
% %     xn = x(i);
% %     for j = 1:1:length(y)
% %     yn = y(j);
% %     
% %         ALPHA(j,i) = a*xn^4 - b*xn^2*yn^2 + c*yn^4 - 1000;
% %     
% %     end
% % end
% % 
% % surf(X,Y,ALPHA,'LineStyle','none')
% % 
% % 
% % t = 0:2*pi/200:2*pi; 
% % r = (1000./(a*cos(t).^4-b*cos(t).^2.*sin(t).^2+c*sin(t).^4)).^0.25;
% % x = r.*cos(t);
% % y = r.*sin(t);
% % hold on
% % plot3(x,y,y*0,'LineWidth',3)
% % hold off
% % % axis equal
% % axis([-1 1 -1 1]*20)
% % grid on
