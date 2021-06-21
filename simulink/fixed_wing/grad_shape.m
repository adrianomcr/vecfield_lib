



function [G_Phi_x, G_Phi_y] = grad_shape(a,b,c,G,g1,p)

    % Delta for the numeric approximation
    delta_x = 1e-4;
    delta_y = 1e-4;
    
    %Point in which the field will be evaluated
    xn = p(1);
    yn = p(2);
    zn = p(3);
    

    
    %Field evaluatioin at p
    alphan = a*xn^4 - b*xn^2*yn^2 + c*yn^4 - 1;
    Vx0 = -G*(4*a*xn^3 - 2*b*xn*yn^2)*alphan + (-4*c*yn^3 + 2*b*xn^2*yn);
    Vy0 = -G*(-2*b*xn^2*yn + 4*c*yn^3)*alphan + (4*a*xn^3 - 2*b*xn*yn^2);
    Vz0 = -g1*zn;
    %Vector normalization
    absValue = (Vx0^2+Vy0^2)^0.5; %normalize only on (x,y)
    Vx = Vx0/absValue;
    Vy = Vy0/absValue;
    Vz = Vz0/absValue;

    
    %Variations on x and y
    xnM = xn+delta_x;
    ynM = yn+delta_y;


    %Compute field at (x+delta,y)
    alphan = a*xnM^4 - b*xnM^2*yn^2 + c*yn^4 - 1;
    Vxa = -G*(4*a*xnM^3 - 2*b*xnM*yn^2)*alphan + (-4*c*yn^3 + 2*b*xnM^2*yn);
    Vya = -G*(-2*b*xnM^2*yn + 4*c*yn^3)*alphan + (4*a*xnM^3 - 2*b*xnM*yn^2);
    absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
    Vy_xM_y = Vya/absValue;
    Vx_xM_y = Vxa/absValue;

    %Compute field at (x,y+delta)
    alphan = a*xn^4 - b*xn^2*ynM^2 + c*ynM^4 - 1;
    Vxa = -G*(4*a*xn^3 - 2*b*xn*ynM^2)*alphan + (-4*c*ynM^3 + 2*b*xn^2*ynM);
    Vya = -G*(-2*b*xn^2*ynM + 4*c*ynM^3)*alphan + (4*a*xn^3 - 2*b*xn*ynM^2);
    absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
    Vx_x_yM = Vxa/absValue;
    Vy_x_yM = Vya/absValue;
    
%     %Compute field at (x,y,z+delta)
%     alphan = a*xn^4 - b*xn^2*yn^2 + c*yn^4 - 1;
%     Vxa = -G*(4*a*xn^3 - 2*b*xn*yn^2)*alphan + (-4*c*yn^3 + 2*b*xn^2*yn);
%     Vya = -G*(-2*b*xn^2*yn + 4*c*yn^3)*alphan + (4*a*xn^3 - 2*b*xn*yn^2);
%     absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
%     Vx_x_y_zM = Vxa/absValue;
%     Vy_x_y_zM = Vya/absValue;

    
%     CUR = (Vy_xM_y-Vy)/delta_x - (Vx_x_yM-Vx)/delta_y;
%     DIV = (Vx_xM_y-Vx)/delta_x + (Vy_x_yM-Vy)/delta_y;
    G_Phi_x = [(Vx_xM_y-Vx)/delta_x; (Vx_x_yM-Vx)/delta_y; 0];
    G_Phi_y = [(Vy_xM_y-Vy)/delta_x; (Vy_x_yM-Vy)/delta_y; 0];

end