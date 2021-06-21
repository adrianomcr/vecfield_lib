



function [G_Phi_x, G_Phi_y] = grad_saddle(a,b,K,g1,p)
    %OBS: K = 0 creates an ellipse

    % Delta for the numeric approximation
    delta_x = 1e-5;
    delta_y = 1e-5;
    delta_z = 1e-5;
    
    %Point in which the field will be evaluated
    xn = p(1);
    yn = p(2);
    zn = p(3); %!!!!!!!!!!!!!!!!!!!!!!
    
    %Field evaluatioin at p
%     Vx0 = -2*K*a^-2*xn*(zn+K*(a^-2*xn^2-1)) -2*a^-2*xn*(a^-2*xn^2+b^-2*yn^2-1)-2*b^-2*yn;
%     Vy0 = -2*b^-2*yn*(a^-2*xn^2+b^-2*yn^2-1)+2*a^-2*xn;
%     Vz0 = -(zn+K*(a^-2*xn^2-1)) + 4*K*a^-2*b^-2*xn*yn;
    alpha_1 = (zn + K*(a^-2*xn^2-1));
    alpha_2 = a^-2*xn^2 + b^-2*yn^2 - 1;
    Vx0 = -2*g1*K*a^-2*xn*alpha_1 - 2*a^-2*xn*alpha_2 - 2*b^-2*yn;
    Vy0 = -2*b^-2*yn*alpha_2 + 2*a^-2*xn;
    Vz0 = -g1*alpha_1 + 4*K*a^-2*b^-2*xn*yn;
    %Vector normalization
    absValue = (Vx0^2+Vy0^2)^0.5; %normalize only on (x,y)
    Vx = Vx0/absValue;
    Vy = Vy0/absValue;
    Vz = Vz0/absValue;

    
    %Variations on x, y and z
    xnM = xn+delta_x;
    ynM = yn+delta_y;
    znM = zn+delta_z;


    %Compute field at (x+delta,y,z)
%     Vxa = -2*K*a^-2*xnM*(zn+K*(a^-2*xnM^2-1)) -2*a^-2*xnM*(a^-2*xnM^2+b^-2*yn^2-1)-2*b^-2*yn;
%     Vya = -2*b^-2*yn*(a^-2*xnM^2+b^-2*yn^2-1)+2*a^-2*xnM;
    alpha_1 = (zn + K*(a^-2*xnM^2-1));
    alpha_2 = a^-2*xnM^2 + b^-2*yn^2 - 1;
    Vxa = -2*g1*K*a^-2*xnM*alpha_1 - 2*a^-2*xnM*alpha_2 - 2*b^-2*yn;
    Vya = -2*b^-2*yn*alpha_2 + 2*a^-2*xnM;
    absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
    Vy_xM_y = Vya/absValue;
    Vx_xM_y = Vxa/absValue;

    %Compute field at (x,y+delta,z)
%     Vxa = -2*K*a^-2*xn*(zn+K*(a^-2*xn^2-1)) -2*a^-2*xn*(a^-2*xn^2+b^-2*ynM^2-1)-2*b^-2*ynM;
%     Vya = -2*b^-2*ynM*(a^-2*xn^2+b^-2*ynM^2-1)+2*a^-2*xn;
    alpha_1 = (zn + K*(a^-2*xn^2-1));
    alpha_2 = a^-2*xn^2 + b^-2*ynM^2 - 1;
    Vxa = -2*g1*K*a^-2*xn*alpha_1 - 2*a^-2*xn*alpha_2 - 2*b^-2*ynM;
    Vya = -2*b^-2*ynM*alpha_2 + 2*a^-2*xn;
    absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
    Vx_x_yM = Vxa/absValue;
    Vy_x_yM = Vya/absValue;
    
    %Compute field at (x,y,z+delta)
%     Vxa = -2*K*a^-2*xn*(znM+K*(a^-2*xn^2-1)) -2*a^-2*xn*(a^-2*xn^2+b^-2*yn^2-1)-2*b^-2*yn;
%     Vya = -2*b^-2*yn*(a^-2*xn^2+b^-2*yn^2-1)+2*a^-2*xn;
    alpha_1 = (znM + K*(a^-2*xn^2-1));
    alpha_2 = a^-2*xn^2 + b^-2*yn^2 - 1;
    Vxa = -2*g1*K*a^-2*xn*alpha_1 - 2*a^-2*xn*alpha_2 - 2*b^-2*yn;
    Vya = -2*b^-2*yn*alpha_2 + 2*a^-2*xn;
    absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
    Vx_x_y_zM = Vxa/absValue;
    Vy_x_y_zM = Vya/absValue;

    
%     CUR = (Vy_xM_y-Vy)/delta_x - (Vx_x_yM-Vx)/delta_y;
%     DIV = (Vx_xM_y-Vx)/delta_x + (Vy_x_yM-Vy)/delta_y;
    G_Phi_x = [(Vx_xM_y-Vx)/delta_x; (Vx_x_yM-Vx)/delta_y; (Vx_x_y_zM-Vx)/delta_z];
    G_Phi_y = [(Vy_xM_y-Vy)/delta_x; (Vy_x_yM-Vy)/delta_y; (Vy_x_y_zM-Vy)/delta_z];
    
    
    
%     %Compute cross product
%     Vxa = -2*K*a^-2*xn*(znM+K*(a^-2*xn^2-1)) -2*a^-2*xn*(a^-2*xn^2+b^-2*yn^2-1)-2*b^-2*yn;
%     Vya = -2*b^-2*yn*(a^-2*xn^2+b^-2*yn^2-1)+2*a^-2*xn;
%     absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
%     Vx_zM = Vxa/absValue;
%     Vy_zM = Vya/absValue;
%     
% %     [(Vx_zM-Vx)/delta_z, (Vy_zM-Vy)/delta_z, 0]
% %     [Vx, Vy 0]
%     CROSS = cross([(Vx_zM-Vx)/delta_z, (Vy_zM-Vy)/delta_z, 0],[Vx, Vy 0]);
%     CROSS = CROSS(3);

end