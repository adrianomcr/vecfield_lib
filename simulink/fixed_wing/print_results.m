

close all;

% Tales_aerosonde_uav_conf_model

dec_rate = 10;

% Obtain variables related to the UAV
time = result.time(1:dec_rate:end);

V_North = result.signals(1).values(1:dec_rate:end,1);
V_East = result.signals(1).values(1:dec_rate:end,2);
P_North = result.signals(1).values(1:dec_rate:end,3);
P_East = result.signals(1).values(1:dec_rate:end,4);
Psi_heading = result.signals(1).values(1:dec_rate:end,5);
h = result.signals(1).values(1:dec_rate:end,6);

vt_ref = result.signals(1).values(1:dec_rate:end,7);
h_ref = result.signals(1).values(1:dec_rate:end,8);
heading_ref = result.signals(1).values(1:dec_rate:end,9);
x_ref = result.signals(1).values(1:dec_rate:end,10);
y_ref = result.signals(1).values(1:dec_rate:end,11);
theta_c = result.signals(1).values(1:dec_rate:end,12);

bank_angle = result_phi.signals(1).values(1:dec_rate:end,2);



% Obtain variables related to the potential functions
time2 = (PotFncs.time)';
V_theta = (PotFncs.signals.values(:,1))';
V_fld = (PotFncs.signals.values(:,2))';



% save('CBA_results/poly_r1.mat','P_East','P_North','h')
% save('CBA_results/poly_r2.mat','P_East','P_North','h')
% save('CBA_results/poly_r3.mat','P_East','P_North','h')
% load('CBA_results/poly_r1.mat','P_East','P_North','h')
% load('CBA_results/poly_r2.mat','P_East','P_North','h')
% load('CBA_results/poly_r3.mat','P_East','P_North','h')



% Print the trajectory of the UAV
figure(1)
clf(1)
plot3(P_East/1e3,P_North/1e3,h/1e3,'Color',[1 0 0],'LineWidth',2);
hold on;
if (curve == 1) % Saddle / Ellipse
    
    px = (a*cos(0:.01:2*pi)+centro_curva(1))/1e3;
    py = (b*sin(0:.01:2*pi)+centro_curva(2))/1e3;
    pz = (Heq - Ksad*(a^-2*(px*1e3).^2-1))/1e3;
    plot3( px, py, pz,'k','LineWidth',1.5);
    pz_max = max(pz);
    pz_min = min(pz);
    
elseif(curve == 2) % Shape
    
    th = 0:2*pi/200:2*pi;
    r = 1./((a*cos(th).^4 - b*cos(th).^2.*sin(th).^2 + c*sin(th).^4).^0.25);
    px = r.*cos(th)*scale_fac/1e3;
    py = r.*sin(th)*scale_fac/1e3;
    pz = ones(size(px))*Heq/1e3;
    plot3(px, py, pz, 'k','LineWidth',1)
    pz_max = max(pz);
    pz_min = min(pz);
    
    plot3( (LIM_CENTER*cos(0:.01:2*pi)+centro_curva(1))*scale_fac/1e3,(LIM_CENTER*sin(0:.01:2*pi)+centro_curva(2))*scale_fac/1e3,Heq/1e3*ones(1,length(sin(0:.01:2*pi))),'k--');
    
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
    plot3(px,py,pz,'k-','LineWidth',1)
    pz_max = max(pz);
    pz_min = min(pz);
    
    plot3( (LIM_CENTER*cos(0:.01:2*pi)+centro_curva(1))*scale_fac/1e3,(LIM_CENTER*sin(0:.01:2*pi)+centro_curva(2))*scale_fac/1e3,Heq/1e3*ones(1,length(sin(0:.01:2*pi))),'k--');
    
elseif(curve == 4) % Polynomial
    
    px = -1:0.05:1;
    py = a*px.^3 + b*px.^2 + c*px.^1 + d*px.^0;
    pz = (Heq/1e3)*ones(size(px));
    plot3(px,py,pz,'k-','LineWidth',2)
    pz_max = max(pz);
    pz_min = min(pz);
    
end
lg = legend('Aircraft','Target');
plot3(P_East(1)/1e3,P_North(1)/1e3,h(1)/1e3,'o','Color',[1 0 0],'LineWidth',2,'MarkerSize',12);
plot3(P_East/1e3,P_North/1e3,h/1e3,'Color',[1 0 0],'LineWidth',2);
hold off;
xlabel('East Position (km)','Interpreter','latex','FontSize',15);
ylabel('North Position (km)','Interpreter','latex','FontSize',15);
zlabel('Height (km)','Interpreter','latex','FontSize',15);
grid on;
title('\textbf{Trajectory}','Interpreter','latex','FontSize',15);
axis equal;



% %Print result Monte carlo ---------- ---------- ----------
% Remember to comment 'close all'
% figure(1)
% subplot(1,2,3-figure_num)
% hold on
% plot3(P_East(1)/1e3,P_North(1)/1e3,h(1)/1e3,'o','Color',cores(kci,:)/255,'MarkerSize',10,'LineWidth',2);
% % plot3( (LIM_CENTER*cos(0:.01:2*pi)+centro_curva(1))*scale_fac/1e3,(LIM_CENTER*sin(0:.01:2*pi)+centro_curva(2))*scale_fac/1e3,Heq/1e3*ones(1,length(sin(0:.01:2*pi))),'k--');
% plot3(P_East/1e3,P_North/1e3,h/1e3,'Color',cores(kci,:)/255,'LineWidth',3);
% 
% xlabel('East Position (km)','Interpreter','latex','FontSize',15);
% ylabel('North Position (km)','Interpreter','latex','FontSize',15);
% zlabel('Height (km)','Interpreter','latex');
% grid on;
% title('\textbf{Trajectory}','Interpreter','latex','FontSize',15);
% axis equal;
% axis([-1200 1500 -1000 1400]/1000)
% % ----------  ----------  ----------  ----------  ----------














%Plot Lyapunov Function % ----------  ----------  ----------  ----------
figure(2)
subplot(2,1,1)
plot(time2,V_theta,'r-','LineWidth',2)
title('\textbf{Lyapunov function $V_\theta$}','Interpreter', 'latex','FontSize',15)
xlabel('$t (s)$','Interpreter', 'latex','FontSize',15)
ylabel('\textbf{$V_\theta$}','Interpreter', 'latex','FontSize',15) %ylabel('V_\theta = 1-cos(\bar{\theta})^2')
ylim([-0.1 1.1]*(max(V_theta)+1e-3))
if (norm([P_East(1) P_North(1)]-centro_curva) < LIM_CENTER*scale_fac)
    s = 1;
    lista = [1];
else
    s = 0;
    lista = [];
end
for k = 2:1:length(P_North)-1
    if (s==1 && norm([P_East(k) P_North(k)]-centro_curva) > LIM_CENTER*scale_fac)
        s = 0;
        lista = [lista k*dec_rate];
    elseif (s==0 && norm([P_East(k) P_North(k)]-centro_curva) < LIM_CENTER*scale_fac)
        s = 1;
        lista = [lista k*dec_rate];
    end
end
if (s==1)
    lista = [lista, (k+1)*dec_rate];
end
if (length(lista) > 10) lista(end-1:end) = []; end
hold on
if (curve ~= 4)
    for k = 1:2:length(lista) 
        plot(time2(lista(k):lista(k+1)),V_theta(lista(k):lista(k+1)),'y--','LineWidth',2)
        plot(time2(lista(k)),V_theta(lista(k)),'k.','MarkerSize',25)
        plot(time2(lista(k+1)),V_theta(lista(k+1)),'k.','MarkerSize',25)
    end
end
%Ultimate bound on V_\theta
gamma = asin(U_theta/Kp);
plot([0*time2(end)/4 time2(end)],[1 1]*(1-cos(gamma)),'k--','LineWidth',2)
hold off
grid on

subplot(2,1,2)
plot(time2,V_fld,'r-','LineWidth',2)
title('\textbf{Field potential function $P$}','Interpreter','latex','FontSize',15)
xlabel('$t (s)$','Interpreter','latex','FontSize',15)
ylabel('\textbf{$P$}','Interpreter','latex','FontSize',15)
ylim([-0.1 1.1]*(max(V_fld+1e-3)))
%Ultimate bound on P -CHECAR ISSO !!!!!!!!!!
hold on
%Computaion of the ultimate bound
v_ref = Veq;
zeta = U_z/v_ref; %rad
omega = vz_max/v_ref;
cos_chi = (cos(gamma)-zeta)/(1+zeta);
% cos_chi = (cos(gamma)-zeta)/(sqrt(1+2*omega*zeta+zeta^2));
chi = acos(cos_chi);
sin_chi = sin(chi);
maxP = (1/gain_G)*tan(-(pi/2)*( -sin_chi ));
if (curve == 1 || curve == 2)
    plot([0*time2(end)/4 time2(end)],[1 1]*maxP,'k--','LineWidth',2)
elseif (curve == 3 || curve == 4)
    plot([0*time2(end)/4 time2(end)],[1 1]*(1/2)*(tan(gamma)/gain_G)^2,'k--','LineWidth',2)
end

hold off
grid on




% Print
figure(3)
subplot(3,1,1);
plot(time,h,'b-');
hold on;
plot(time,h_ref,'r--');
%axis([0 time(end) 199.5 200.5]);
grid minor;
ylabel('\textbf{$z \ (m)$}','Interpreter','latex','FontSize',15);
legend('Aircraft','Reference Model');
xlabel('$t \ (s)$','Interpreter','latex','FontSize',15);
title('\textbf{Altitude}','Interpreter','latex','FontSize',15);

subplot(3,1,2);
plot(time,sqrt(V_North.^2 + V_East.^2),'b-');
hold on;
plot(time,vt_ref,'r--');
%axis([0 time(end) 22.9 23.1]);
grid minor;
ylabel('\textbf{$v \ (m/s)$}','Interpreter','latex','FontSize',15);
legend('Aircraft','Reference Model');
xlabel('$t \ (s)$','Interpreter','latex','FontSize',15);
title('\textbf{Ground speed}','Interpreter','latex','FontSize',15);

subplot(3,1,3);
plot(time,bank_angle,'b');
hold on;
plot([0 time(end)],[phi_min phi_min]*180/pi,'r--');
plot([0 time(end)],[phi_max phi_max]*180/pi,'r--');
%axis([0 time(end) -50 50]);
grid on;
ylabel('\textbf{$\phi \ (deg.)$}','Interpreter','latex','FontSize',15);
legend('\phi','Safety limits');
xlabel('$t \ (s)$','Interpreter','latex','FontSize',15);
title('\textbf{Bank Angle}','Interpreter','latex','FontSize',15);
%print('AltSpeedBank', '-dpdf');








% Refference and execution of w
figure(4)
subplot(2,1,1)
time = w_analise.time;
w_ref = w_analise.signals.values(:,1);
w_real = w_analise.signals.values(:,2);
plot(time,w_ref,'r--','LineWidth',1)
hold on
plot(time,w_real,'b','LineWidth',1)
hold off
title('Turning ratio','FontSize',15)
xlabel('t (s)','FontSize',15)
ylabel('\omega (rad/s)','FontSize',15)
legend('reference','execution')
grid on

subplot(2,1,2)
plot(time,w_real-w_ref,'b','LineWidth',1)
hold on
plot([time(1) time(end)],+U_theta*[1 1],'r--','LineWidth',2)
plot([time(1) time(end)],-U_theta*[1 1],'r--','LineWidth',2)
hold off
title('Reference model error','FontSize',15)
xlabel('t (s)','FontSize',15)
ylabel('\omega (rad/s)','FontSize',15)
% legend('','')
grid on












% Refference and execution of acceleration
figure(5)
subplot(2,1,1)
time = a_analise.time;
a_ref = a_analise.signals.values(:,1);
a_real = a_analise.signals.values(:,2);
plot(time,a_ref,'r--','LineWidth',1)
hold on
plot(time,a_real,'b','LineWidth',1)
hold off
title('Linear acceleration','FontSize',15)
xlabel('t (s)','FontSize',15)
ylabel('a (m/s^2)','FontSize',15)
legend('reference','execution')
grid on

subplot(2,1,2)
plot(time,a_real-a_ref,'b','LineWidth',1)
hold on
plot([time(1) time(end)],+U_v*[1 1],'r--','LineWidth',2)
plot([time(1) time(end)],-U_v*[1 1],'r--','LineWidth',2)
hold off
title('Reference model error','FontSize',15)
xlabel('t (s)','FontSize',15)
ylabel('a (m/s^2)','FontSize',15)
% legend('','')
grid on









%%
figure(6)
subplot(3,1,1)
plot(linspace(0,T_sim,length(h)),h,'b-','LineWidth',1);
hold on
plot([0 T_sim],Heq*0+[1 1]*U_z*tau_z + [1 1]*pz_max*scale_fac*1e3,'r--','LineWidth',2);
plot([0 T_sim],Heq*0-[1 1]*U_z*tau_z + [1 1]*pz_min*scale_fac*1e3,'r--','LineWidth',2);
hold off
% axis ([0, T_sim, [-1 1]*U_z*tau_z+Heq*0+[pz_min pz_max]*scale_fac*1e3 ])
xlabel('$t(s)$','interpreter','latex','FontSize',15)
ylabel('$z \ (m)$','interpreter','latex','FontSize',15)
title('Height','interpreter','latex','FontSize',15)

subplot(3,1,2)
plot(linspace(0,T_sim,length(V_North)),sqrt(V_North.^2 + V_East.^2),'b-','LineWidth',1);
hold on
plot([0 T_sim],Veq+[1 1]*U_v*tau_v,'r--','LineWidth',2);
plot([0 T_sim],Veq-[1 1]*U_v*tau_v,'r--','LineWidth',2);
hold off
axis ([0, T_sim, [-1 1]*U_v*tau_v+Veq ])
xlabel('$t(s)$','interpreter','latex','FontSize',15)
ylabel('$v \ (m/s)$','interpreter','latex','FontSize',15)
title('Forward velocity','interpreter','latex','FontSize',15)

subplot(3,1,3)
time = w_analise.time;
w_ref = w_analise.signals.values(:,1);
w_real = w_analise.signals.values(:,2);
plot(linspace(0,T_sim,length(w_real)),w_real-w_ref,'b-','LineWidth',1)
hold on
plot([0 T_sim],+[1 1]*U_theta,'r--','LineWidth',2);
plot([0 T_sim],-[1 1]*U_theta,'r--','LineWidth',2);
hold off
axis ([0 T_sim -U_theta U_theta])
xlabel('$t(s)$','interpreter','latex','FontSize',15)
% ylabel('$\omega-\dot{\theta} \ (rad/s)$','interpreter','latex','FontSize',15)
ylabel('$\Delta\omega \ (rad/s)$','interpreter','latex','FontSize',15)
title('Error in $\omega$','interpreter','latex','FontSize',15)







figure(5)
figure(4)
figure(3)
figure(2)
figure(1)







PLOT_QUIVER = 1;
if (PLOT_QUIVER == 1)

    if (curve == 1)
    elseif (curve == 2)
        shape_field_conatant = 1;

%         G = Gc;
%         LIM_CENTER = LIM_CENTER*scale_fac/1e3;
%         a = 0.5/(scale_fac/1e3)^4;
%         b = 0/(scale_fac/1e3)^4;
%         c = 1.5/(scale_fac/1e3)^4;

        %Definition of the grid to plot the vector field
        w_s = [-1.2 1.2 -0.8 0.8];
        df = 0.12-0.04;
        x = w_s(1):df:w_s(2);
        y = w_s(3):df:w_s(4);
        [X,Y] = meshgrid(x,y);


        d_beta = 2*pi/50;
        %Compute the vectro field to plot it
        Vx = 0*X;
        Vy = 0*Y;
        Vz = 0*X;
        zn = 0;
        M = 0;
        for j = 1:1:length(x)
            xn = x(j);
            for i = 1:1:length(y)  
                yn = y(i);

                [Vxyz, Vxyz_hat, conv_V, circ_V, alpha_1, alpha_2, v_alpha] = get_field([xn,yn,zn],[a,b,c,g1,gain_G],'shape');

                Vx(i,j) = Vxyz(1);
                Vy(i,j) = Vxyz(2);
                Vz(i,j) = Vxyz(3);
                absValue = (Vx(i,j)^2+Vy(i,j)^2+Vz(i,j)^2)^0.5;
                Vx(i,j) = Vx(i,j)/absValue;
                Vy(i,j) = Vy(i,j)/absValue;
                Vz(i,j) = Vz(i,j)/absValue;

                %Only to plot
                CONV_Vx(i,j) = conv_V(1)/absValue;
                CONV_Vy(i,j) = conv_V(2)/absValue;
                CONV_Vz(i,j) = conv_V(3)/absValue;
                CIRC_Vx(i,j) = circ_V(1)/absValue;
                CIRC_Vy(i,j) = circ_V(2)/absValue;
                CIRC_Vz(i,j) = circ_V(3)/absValue;
                
                if norm([xn,yn]) < LIM_CENTER
                    Vx(i,j) = 0;
                    Vy(i,j) = 0;
                else
                    [M1,M2] = get_grad([xn,yn,0],[a,b,c,g1,gain_G],'shape');
                    if(norm(M1) > M) M = norm(M1); end
                    if(norm(M2) > M) M = norm(M2); end
                end
                
                
            end
        end
        hold on
        nf = 1;
        quiver3(X,Y,ones(size(X))*Heq/1e3,Vx*nf,Vy*nf,zeros(size(X)),'Color',[0.7 0.7 1],'AutoScale','on')
        hold off
        axis(w_s)
        
        
	elseif (curve == 3)
        shape_field_conatant = 1;

        %Definition of the grid to plot the vector field
        w_s = [-1.2 1.2 -0.8 0.8];
        df = 0.08/5;
        x = w_s(1):df:w_s(2);
        y = w_s(3):df:w_s(4);
        [X,Y] = meshgrid(x,y);


        d_beta = 2*pi/50;
        %Compute the vectro field to plot it
        Vx = 0*X;
        Vy = 0*Y;
        Vz = 0*X;
        zn = 0;
        M = 0; M_mat = 0*X;
        for j = 1:1:length(x)
            xn = x(j);
            for i = 1:1:length(y)  
                yn = y(i);

                [Vxyz, Vxyz_hat, conv_V, circ_V, alpha_1, alpha_2, v_alpha] = get_field([xn,yn,zn],[a,b,c,g1,gain_G],'random');

                Vx(i,j) = Vxyz(1);
                Vy(i,j) = Vxyz(2);
                Vz(i,j) = Vxyz(3);
                absValue = (Vx(i,j)^2+Vy(i,j)^2+Vz(i,j)^2)^0.5;
                Vx(i,j) = Vx(i,j)/absValue;
                Vy(i,j) = Vy(i,j)/absValue;
                Vz(i,j) = Vz(i,j)/absValue;

                %Only to plot
                CONV_Vx(i,j) = conv_V(1)/absValue;
                CONV_Vy(i,j) = conv_V(2)/absValue;
                CONV_Vz(i,j) = conv_V(3)/absValue;
                CIRC_Vx(i,j) = circ_V(1)/absValue;
                CIRC_Vy(i,j) = circ_V(2)/absValue;
                CIRC_Vz(i,j) = circ_V(3)/absValue;
                

%                 [M1,M2] = get_grad([xn,yn,0],[a,b,c,g1,gain_G],'random');
                if norm([xn,yn]) < LIM_CENTER
                    Vx(i,j) = 0;
                    Vy(i,j) = 0;
                    CURL(i,j) = 0;
                    DIV(i,j) = 0;
                    M_mat(i,j) = 0;
                else
%                     [M1,M2] = get_grad([xn,yn,0],[a,b,c,g1,gain_G],'random');
                    [M1, M2] = get_curl_div([xn,yn,zn],[a,b,c,g1,gain_G],'random');
                    [CURL(i,j),DIV(i,j)] = get_curl_div([xn,yn,zn],[a,b,c,g1,gain_G],'random');
                    if(norm(M1) > M) M = norm(M1); end
                    if(norm(M2) > M) M = norm(M2); end
                    M_mat(i,j) = max([norm(M1),norm(M2)]);
                end
                
            end
        end
        hold on
        nf = 1;
        quiver3(X,Y,ones(size(X))*Heq/1e3,Vx*nf,Vy*nf,zeros(size(X)),'Color',[0.7 0.7 1],'AutoScale','on')
        hold off
        axis(w_s)
        
        % Plot curl and divergence
%         figure(7)
%         surf(X,Y,CURL,'LineStyle','none')
%         title('Curl','FontSize',15)
%         colorbar
%         figure(8)
%         surf(X,Y,DIV,'LineStyle','none')
%         title('Divergence','FontSize',15)
%         colorbar
%         figure(9)
%         surf(X,Y,M_mat,'LineStyle','none')
%         title('Maximum norm of the gradient','FontSize',15)
%         colorbar
        
        
     elseif (curve == 4)
        shape_field_conatant = 1;

        %Definition of the grid to plot the vector field
        w_s = [-1 1 -0.7 0.7];
        df = 0.1/10;
        x = w_s(1):df:w_s(2);
        y = w_s(3):df:w_s(4);
        [X,Y] = meshgrid(x,y);


        d_beta = 2*pi/50;
        %Compute the vectro field to plot it
        Vx = 0*X;
        Vy = 0*Y;
        Vz = 0*X;
        zn = 0;
        M = 0;
        for j = 1:1:length(x)
            xn = x(j);
            for i = 1:1:length(y)  
                yn = y(i);

                [Vxyz, Vxyz_hat, conv_V, circ_V, alpha_1, alpha_2, v_alpha] = get_field([xn,yn,zn],[a,b,c,d,g1,gain_G],'poly');

                Vx(i,j) = Vxyz(1);
                Vy(i,j) = Vxyz(2);
                Vz(i,j) = Vxyz(3);
                absValue = (Vx(i,j)^2+Vy(i,j)^2+Vz(i,j)^2)^0.5;
                Vx(i,j) = Vx(i,j)/absValue;
                Vy(i,j) = Vy(i,j)/absValue;
                Vz(i,j) = Vz(i,j)/absValue;

                %Only to plot
                CONV_Vx(i,j) = conv_V(1)/absValue;
                CONV_Vy(i,j) = conv_V(2)/absValue;
                CONV_Vz(i,j) = conv_V(3)/absValue;
                CIRC_Vx(i,j) = circ_V(1)/absValue;
                CIRC_Vy(i,j) = circ_V(2)/absValue;
                CIRC_Vz(i,j) = circ_V(3)/absValue;
                

%                 [M1,M2] = get_grad([xn,yn,0],[a,b,c,d,g1,gain_G],'poly');
                [M1,M2] = get_curl_div([xn,yn,0],[a,b,c,d,g1,gain_G],'poly');
                [CURL(i,j),DIV(i,j)] = get_curl_div([xn,yn,zn],[a,b,c,g1,gain_G],'random');
                if(norm(M1) > M) M = norm(M1); end
                if(norm(M2) > M) M = norm(M2); end
                
                
            end
        end
%         
%        % Plot curl and divergence
%         figure(7)
%         surf(X,Y,CURL,'LineStyle','none')
%         title('Curl','FontSize',15)
%         colorbar
%         figure(8)
%         surf(X,Y,DIV,'LineStyle','none')
%         title('Divergence','FontSize',15)
%         colorbar
        
        hold on
        nf = 1;
        quiver3(X,Y,ones(size(X))*Heq/1e3,Vx*nf,Vy*nf,zeros(size(X)),'Color',[0.7 0.7 1],'AutoScale','on')
        hold off
        axis(w_s)
        
        
        
    end

end %if plot






PLOT_TUBE = 0;
if (PLOT_TUBE == 1)
    
    if (curve == 1)
        
        border_i = [];
        border_o = [];
    
        U_z_save = U_z;
        U_z = 0;
        a = a*scale_fac/1e3;
        b = b*scale_fac/1e3;
        Ksad = Ksad*scale_fac/1e3;
        LIM_CENTER = LIM_CENTER*scale_fac/1e3;
        
        
        hold on
        dkt = 2*pi/16;
%         dkt = 2*pi/200;
        for kt = 0:dkt:2*pi
        % for kt = 0
            px = a*cos(kt);
            py = b*sin(kt);
            pz = -Ksad*(a^-2*(a*cos(kt)).^2-1);

            alpha_lim = tan(asin(U_theta/Kp));
            a_M = a*sqrt(1+alpha_lim);
            b_M = b*sqrt(1+alpha_lim);
            a_m = a*sqrt(1-alpha_lim);
            b_m = b*sqrt(1-alpha_lim);
            r_M = sqrt((a_M*cos(kt))^2+(b_M*sin(kt))^2);
            r_m = sqrt((a_m*cos(kt))^2+(b_m*sin(kt))^2);
            ret = [[1 -1 -1 1]*(r_M-r_m)/2; 0 0 0 0; U_z U_z -U_z -U_z];
            pxM = a_M*cos(kt);
            pyM = b_M*sin(kt);
            pxm = a_m*cos(kt);
            pym = b_m*sin(kt);
            ret = [[1 -1 -1 1]*sqrt((pxM-pxm)^2+(pyM-pym)^2)/2; 0 0 0 0; U_z U_z -U_z -U_z];
            ktheta = kt;
            Rotz = [cos(ktheta) sin(ktheta) 0; -sin(ktheta) cos(ktheta) 0; 0 0 1]; % in z
            kphi_z = atan2(Ksad*2*abs(px)*a^(-2),1);
            Roty = [cos(kphi_z) 0 -sin(kphi_z); 0 1 0; sin(kphi_z) 0 cos(kphi_z)]; % in local y
            ret = Rotz^-1*Roty^-1*ret;
            ret(1,:) = ret(1,:)+px;
            ret(2,:) = ret(2,:)+py;
            ret(3,:) = ret(3,:)+pz;
        %     plot3(px,py,pz,'c*','LineWidth',1)
            plot3(ret(1,[1 2 3 4 1]),ret(2,[1 2 3 4 1]),ret(3,[1 2 3 4 1])+Heq/1e3,'b-','LineWidth',1)
        %     fill3(ret(1,[1 2 3 4 1]),ret(2,[1 2 3 4 1]),ret(3,[1 2 3 4 1]),'b-','FaceAlpha',0.5,'LineStyle','none')


            border_i = [border_i, [pxm pym pz]'];
            border_o = [border_o, [pxM pyM pz]'];
        
        
            px = a*cos(kt+dkt);
            py = b*sin(kt+dkt);
            pz = -Ksad*(a^-2*(a*cos(kt+dkt)).^2-1);

            alpha_lim = tan(asin(U_theta/Kp));
            a_M = a*sqrt(1+alpha_lim);
            b_M = b*sqrt(1+alpha_lim);
            a_m = a*sqrt(1-alpha_lim);
            b_m = b*sqrt(1-alpha_lim);
            r_M = sqrt((a_M*cos(kt+dkt))^2+(b_M*sin(kt+dkt))^2);
            r_m = sqrt((a_m*cos(kt+dkt))^2+(b_m*sin(kt+dkt))^2);
            ret2 = [[1 -1 -1 1]*(r_M-r_m)/2; 0 0 0 0; U_z U_z -U_z -U_z];
            pxM = a_M*cos(kt+dkt);
            pyM = b_M*sin(kt+dkt);
            pxm = a_m*cos(kt+dkt);
            pym = b_m*sin(kt+dkt);
            ret2 = [[1 -1 -1 1]*sqrt((pxM-pxm)^2+(pyM-pym)^2)/2; 0 0 0 0; U_z U_z -U_z -U_z];
            ktheta = kt+dkt;
            Rotz = [cos(ktheta) sin(ktheta) 0; -sin(ktheta) cos(ktheta) 0; 0 0 1]; % in z
            kphi_z = atan2(Ksad*2*abs(px)*a^(-2),1);
            Roty = [cos(kphi_z) 0 -sin(kphi_z); 0 1 0; sin(kphi_z) 0 cos(kphi_z)]; % in local y
            ret2 = Rotz^-1*Roty^-1*ret2;
            ret2(1,:) = ret2(1,:)+px;
            ret2(2,:) = ret2(2,:)+py;
            ret2(3,:) = ret2(3,:)+pz;


            fill3([ret(1,[1 2]) ret2(1,[2 1])],[ret(2,[1 2]) ret2(2,[2 1])],[ret(3,[1 2]) ret2(3,[2 1])]+Heq/1e3,'b-','FaceAlpha',0.1,'LineStyle','none')
            fill3([ret(1,[2 3]) ret2(1,[3 2])],[ret(2,[2 3]) ret2(2,[3 2])],[ret(3,[2 3]) ret2(3,[3 2])]+Heq/1e3,'b-','FaceAlpha',0.1,'LineStyle','none')
            fill3([ret(1,[3 4]) ret2(1,[4 3])],[ret(2,[3 4]) ret2(2,[4 3])],[ret(3,[3 4]) ret2(3,[4 3])]+Heq/1e3,'b-','FaceAlpha',0.1,'LineStyle','none')
            fill3([ret(1,[4 1]) ret2(1,[1 4])],[ret(2,[4 1]) ret2(2,[1 4])],[ret(3,[4 1]) ret2(3,[1 4])]+Heq/1e3,'b-','FaceAlpha',0.1,'LineStyle','none')

%             fill3([ret(1,[1 2]) ret2(1,[2 1])],[ret(2,[1 2]) ret2(2,[2 1])],[ret(3,[1 2]) ret2(3,[2 1])]+Heq/1e3,'b-','FaceAlpha',0.2,'LineStyle','none')
            
        end
%         plot3(border_i(1,:),border_i(2,:),border_i(3,:)+Heq/1e3,'k--','LineWidth',2)
%         plot3(border_o(1,:),border_o(2,:),border_o(3,:)+Heq/1e3,'k--','LineWidth',2)
        U_z = U_z_save;
        hold off
    elseif (curve == 2)
    end
end



figure(1)
print_tube