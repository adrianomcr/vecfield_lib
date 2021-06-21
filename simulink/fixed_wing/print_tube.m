
N = 16 + 4 + 4; %Number of faces
N2 = 16; %Number of direction ina face

step = 1/scale_fac; %Search step
% step = step/5

%Computaion of the ultimate bound
v_ref = Veq;
gamma = asin(U_theta/Kp); %rad
% gamma = 0;
zeta = U_z/v_ref; %rad
% zeta = 0;
omega = vz_max/v_ref;
cos_chi = (cos(gamma)-zeta)/(1+zeta);
% cos_chi = (cos(gamma)-zeta)/(sqrt(1+2*omega*zeta+zeta^2));
chi = acos(cos_chi);
sin_chi = sin(chi);
maxP = (1/gain_G)*tan(-(pi/2)*( -sin_chi ));
maxP = maxP^2;



PLOT_TORUS = 1;
if (PLOT_TORUS == 1 && (curve == 1 || curve == 2))

    points_set = [];

    hold on
    for n= 0:1:(N-1)
        th = (2*pi/N)*n;
        xh = -1 + (2*N)*n;

        if (curve == 1)
            x1 = a*cos(th);
            y1 = b*sin(th);
            z1 = -Ksad*(a^(-2)*x1.^2-1);

            x2 = a*cos(th+10e-3);
            y2 = b*sin(th+10e-3);
            z2 = -Ksad*(a^(-2)*x2.^2-1);
        elseif (curve == 2)


            r1 = 1./((a*cos(th)^4-b*cos(th)^2*sin(th)^2+c*sin(th)^4)^0.25);
            x1 = r1*cos(th);
            y1 = r1*sin(th);
            z1 = 0;

            r2 = 1./((a*cos(th+10e-3)^4-b*cos(th+10e-3)^2*sin(th+10e-3)^2+c*sin(th+10e-3)^4)^0.25);
            x2 = r2*cos(th+10e-3);
            y2 = r2*sin(th+10e-3);
            z2 = 0;

        elseif (curve == 3)
                th = th + exp(1)*1e-5;
                A = b*cos(th).^2.*sin(th).^2;
                B = a*cos(th).^2 + c*sin(th).^2;
                C = -(1);
                m = -B + sqrt(B.^2-4.*A.*C); m = m./(2*A);
                r1 = sqrt(m);
                x1 = r1.*cos(th);
                y1 = r1.*sin(th);
                z1 = 0;

                A = b*cos(th+10e-3).^2.*sin(th+10e-3).^2;
                B = a*cos(th+10e-3).^2 + c*sin(th+10e-3).^2;
                C = -(1);
                m = -B + sqrt(B.^2-4.*A.*C); m = m./(2*A);
                r2 = sqrt(m);
                x2 = r2.*cos(th+10e-3);
                y2 = r2.*sin(th+10e-3);
                z2 = 0;

        elseif (curve == 4)
                x1 = xh;
                y1 = a*xh^3 + b*xh^2 + c*xh^1 + d*xh^0;
                z1 = 0;

                x2 = xh;
                y2 = a*xh^3 + b*xh^2 + c*xh^1 + d*xh^0;
                z2 = 0;

        else
            error('Specify curve')
        end

        v = [x2;y2;z2]-[x1;y1;z1];
        v = v*10;
    %     % Tangent vector
    %     quiver3(x1,y1,z1+Heq/1e3,v(1),v(2),v(3),'b','AutoScale','off','LineWidth',2,'MaxHeadSize',2);

        poligon = [cos(0:2*pi/N2:2*pi-10e-6);zeros(1,N2);sin(0:2*pi/N2:2*pi-10e-6)]*50;
        Rotz = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1]; % in z
        phi = atan2(v(3),norm(v(1:2)));
        Rotx = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)]; % in local y
        poligon = Rotz^-1*Rotx^-1*poligon;
        poligon(1,:) = poligon(1,:)+x1;
        poligon(2,:) = poligon(2,:)+y1;
        poligon(3,:) = poligon(3,:)+z1;


        points =  struct('x',[],'y',[],'z',[]);
        for k = 1:1:N2
            dir = poligon(:,k)-[x1;y1;z1]; %search direction
            dir = dir/norm(dir);

            p = [x1;y1;z1] + 0*dir;

            if (curve == 1)
                alpha_1 = p(3) + Ksad*(a^(-2)*p(1)^2-1);
                alpha_2 = a^(-2)*p(1)^2 + b^(-2)*p(2)^2 - 1;
            elseif (curve == 2)
                alpha_1 = p(3);
                alpha_2 = a*p(1)^4 - b*p(1)^2*p(2)^2 + c*p(2)^4 - 1;
            elseif (curve == 3)
                alpha_1 = p(3);
                alpha_2 = a*p(1)^2 + b*p(1)^2*p(2)^2 + c*p(2)^2 - 1;
            elseif (curve == 4)
                alpha_1 = p(3);
                alpha_2 = gain_G*(a*p(1)^3 + b*p(1)^2 + c*p(1)^1 + d*p(1)^0 - p(2));
            end

            P = (1/2)*g1*alpha_1^2 + (1/2)*alpha_2^2;
            count = 0;
            while (P<maxP && count < 1000)
                count = count+1;

                p = [x1;y1;z1] + (count*step)*dir;
                if (curve == 1)
                    alpha_1 = p(3) + Ksad*(a^(-2)*p(1)^2-1);
                    alpha_2 = a^(-2)*p(1)^2 + b^(-2)*p(2)^2 - 1;
                elseif (curve == 2)
                    alpha_1 = p(3);
                    alpha_2 = a*p(1)^4 - b*p(1)^2*p(2)^2 + c*p(2)^4 - 1;
                elseif (curve == 3)
                    alpha_1 = p(3);
                    alpha_2 = a*p(1)^2 + b*p(1)^2*p(2)^2 + c*p(2)^2 - 1;
                elseif (curve == 3)
                    alpha_1 = p(3);
                    alpha_2 = gain_G*(a*p(1)^3 + b*p(1)^2 + c*p(1)^1 + d*p(1)^0 - p(2));
                end
    %             P = g1*alpha_1^2 + alpha_2^2;
                P = (1/2)*g1*alpha_1^2 + (1/2)*alpha_2^2;
            end

    %         quiver3(x1,y1,z1+Heq/1e3,p(1)-x1,p(2)-y1,p(3)-z1,'r')
    %         quiver3(x1,y1,z1+Heq/1e3,dir(1)*0.1,dir(2)*0.1,dir(3)*0.1,'r')

            points.x = [points.x, p(1)];
            points.y = [points.y, p(2)];
            points.z = [points.z, p(3)];
            if (count == 1000)
                warning('Maximu count reached')
            end

        end


        points_set = [points_set, points];

    end

    hold off



    hold on
    for k = 1:1:length(points_set)
        %fill3(points_set(k).x,points_set(k).y,points_set(k).z,'y','FaceAlpha',0.5,'LineStyle','none')
        xx = points_set(k).x/1e3*scale_fac;
        yy = points_set(k).y/1e3*scale_fac;
        zz = points_set(k).z/1e3*scale_fac + Heq/1e3;
%         fill3(xx, yy, zz, 'y', 'FaceAlpha',0.0, 'LineStyle','--', 'EdgeColor', [0 0 1], 'EdgeAlpha', 0.8)
        fill3(xx, yy, zz, 'y', 'FaceAlpha',0.0, 'LineStyle','-', 'EdgeColor', [0 0 1], 'EdgeAlpha', 0.8)

        nk = mod(k,N)+1;

        for j = 1:1:N2
            nj = mod(j,N2)+1;
            xx = [points_set(k).x(j) points_set(k).x(nj) points_set(nk).x(nj) points_set(nk).x(j)]/1e3*scale_fac;
            yy = [points_set(k).y(j) points_set(k).y(nj) points_set(nk).y(nj) points_set(nk).y(j)]/1e3*scale_fac;
            zz = [points_set(k).z(j) points_set(k).z(nj) points_set(nk).z(nj) points_set(nk).z(j)]/1e3*scale_fac + Heq/1e3;
            fill3(xx,yy,zz,'b','FaceAlpha',0.1,'LineStyle','none')
        end 

    end
    hold off
end



% 
% %Ultimate in 2d - saddle/ellipse
% if curve == 1
%     th = 0:2*pi/100:2*pi;
%     level = (1/gain_G)*tan((pi/2)*sin(gamma)); %level in P
%     level = level^2;
%     level = sqrt(2*level); %level in alpha_2
% 
%     xa = a*sqrt(1+level)*cos(th)*scale_fac/1e3;
%     ya = b*sqrt(1+level)*sin(th)*scale_fac/1e3;
%     za = ones(size(xa))*Heq/1e3;
%     
%     xb = a*sqrt(1-level)*cos(th)*scale_fac/1e3;
%     yb = b*sqrt(1-level)*sin(th)*scale_fac/1e3;
%     zb = ones(size(xb))*Heq/1e3;
%     
%     
%     hold on
%     plot3(xa,ya,za,'b-','LineWidth',2)
%     plot3(xb,yb,zb,'b-','LineWidth',2)
%     hold off    
% end
% 
% 
% 
%Ultimate in 2d - shape
if curve == 2
    th = 0:2*pi/100:2*pi;
    level = (1/gain_G)*tan((pi/2)*sin(gamma)); %level in P
    level = level^2;
    level = sqrt(2*level); %level in alpha_2
%     level = 0
    
    ra = (1+level)^0.25./((a*cos(th).^4-b*cos(th).^2.*sin(th).^2+c*sin(th).^4).^0.25);
    xa = ra.*cos(th)*scale_fac/1e3;
    ya = ra.*sin(th)*scale_fac/1e3;
    za = ones(size(xa))*Heq/1e3;
    
    rb = (1-level)^0.25./((a*cos(th).^4-b*cos(th).^2.*sin(th).^2+c*sin(th).^4).^0.25);
    xb = rb.*cos(th)*scale_fac/1e3;
    yb = rb.*sin(th)*scale_fac/1e3;
    zb = ones(size(xb))*Heq/1e3;
    
    
    hold on
    plot3(xa,ya,za,'b-','LineWidth',2)
    plot3(xb,yb,zb,'b-','LineWidth',2)
    hold off    
end


%Ultimate in 2d - random
if curve == 3
    th = (0:2*pi/200:2*pi) + 2*pi/200/2;
%     level = (2/gain_G)*tan((pi/2)*sin(gamma));
    level = (gain_G^-1)*tan(gamma);
    
    A = b*cos(th).^2.*sin(th).^2;
    B = a*cos(th).^2 + c*sin(th).^2;
    C = -(1+level);
    m = -B + sqrt(B.^2-4.*A.*C); m = m./(2*A);
    ra = sqrt(m);
    xa = ra.*cos(th)*scale_fac/1e3;
    ya = ra.*sin(th)*scale_fac/1e3;
    za = ones(size(xa))*Heq/1e3;
    
    A = b*cos(th).^2.*sin(th).^2;
    B = a*cos(th).^2 + c*sin(th).^2;
    C = -(1-level);
    m = -B + sqrt(B.^2-4.*A.*C); m = m./(2*A);
    rb = sqrt(m);
    xb = rb.*cos(th)*scale_fac/1e3;
    yb = rb.*sin(th)*scale_fac/1e3;
    zb = ones(size(xb))*Heq/1e3;
    
    hold on
%     plot3(xa,ya,za,'b-','LineWidth',2)
%     plot3(xb,yb,zb,'b-','LineWidth',2)
    plot3(xa,ya,za,'k-','LineWidth',1)
    plot3(xb,yb,zb,'k-','LineWidth',1)
    hold off    
end



%Ultimate in 2d - poly
if curve == 4
    th = (0:2*pi/200:2*pi) + exp(1)*1e-5;
%     level = (2/gain_G)*tan((pi/2)*sin(gamma));
    level = (1/gain_G)*tan(gamma);
    
    
    xa = -1:0.05:1;
    ya = a*xa.^3 + b*xa.^2 + c*xa.^1 + d*xa.^0 + level;
    za = ones(size(xa))*Heq/1e3;
    
    xb = -1:0.05:1;
    yb = a*xb.^3 + b*xb.^2 + c*xb.^1 + d*xb.^0 - level;
    zb = ones(size(xa))*Heq/1e3;
    
    hold on
%     plot3(xa,ya,za,'b-','LineWidth',2)
%     plot3(xb,yb,zb,'b-','LineWidth',2)
    plot3(xa,ya,za,'k-','LineWidth',1)
    plot3(xb,yb,zb,'k-','LineWidth',1)
    hold off    
end


