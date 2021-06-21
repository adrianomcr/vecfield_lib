



function [CUR, DIV] = cur_div_ellipse(a,b,p)

    % Delta for the numeric approximation
    delta_x = 1e-4;
    delta_y = 1e-4;
    
    %Point in which the field will be evaluated
    xn = p(1);
    yn = p(2);
    zn = p(3);
    
    %Field evaluatioin at p
    Vx0 = -2*a^-2*xn*(a^-2*xn^2+b^-2*yn^2-1)-2*b^-2*yn;
    Vy0 = -2*b^-2*yn*(a^-2*xn^2+b^-2*yn^2-1)+2*a^-2*xn;
    Vz0 = -zn;
    %Vector normalization
    absValue = (Vx0^2+Vy0^2)^0.5; %normalize only on (x,y)
    Vx = Vx0/absValue;
    Vy = Vy0/absValue;
    Vz = Vz0/absValue;

    
    %Variations on x and y
    xnM = xn+delta_x;
    ynM = yn+delta_y;


    %Compute field at (x+delta,y)
    Vxa = -2*a^-2*xnM*(a^-2*xnM^2+b^-2*yn^2-1)-2*b^-2*yn;
    Vya = -2*b^-2*yn*(a^-2*xnM^2+b^-2*yn^2-1)+2*a^-2*xnM;
    absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
    Vy_xM_y = Vya/absValue;
    Vx_xM_y = Vxa/absValue;

    %Compute field at (x,y+delta)
    Vxa = -2*a^-2*xn*(a^-2*xn^2+b^-2*ynM^2-1)-2*b^-2*ynM;
    Vya = -2*b^-2*ynM*(a^-2*xn^2+b^-2*ynM^2-1)+2*a^-2*xn;
    absValue = (Vxa^2+Vya^2)^0.5; %normalize only on (x,y)
    Vx_x_yM = Vxa/absValue;
    Vy_x_yM = Vya/absValue;

    
    CUR = (Vy_xM_y-Vy)/delta_x - (Vx_x_yM-Vx)/delta_y;
    DIV = (Vx_xM_y-Vx)/delta_x + (Vy_x_yM-Vy)/delta_y;

end