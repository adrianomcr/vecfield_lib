
function [r, T, N] = my_curve(s, t, curva)

ds = 1e-8;

switch curva
    
    %For the selected curve, compute the sample (r(s,t)) and the tangent
    %vector and the normal vector
    
    case 1
        % Horizontal circle
        r = [cos(s); sin(s); 1 + 0.2*sin(0.4*t)];
        T = [-sin(s); cos(s); 0];
        Tm = [-sin(s-ds); cos(s-ds); 0 ];
        TM = [-sin(s+ds); cos(s+ds); 0 ];
        
    case 2
        % Vertical circle
        r = [0; cos(s); sin(s)];
        T = [0; -sin(s); cos(s)];
        Tm = [0; -sin(s-ds); cos(s-ds)];
        TM = [0; -sin(s+ds); cos(s+ds)];
    
    case 3
        % Spiral
        r = [4*(s-pi)/pi; cos(10*s); sin(10*s)];
        T = [4/pi; -10*sin(10*s); 10*cos(10*s)];
        Tm = [4/pi; -10*sin(10*(s-ds)); 10*cos(10*(s-ds))];
        TM = [4/pi; -10*sin(10*(s+ds)); 10*cos(10*(s+ds))];

    case 4
        % Eight like curve
        r = [cos(0.3*t) -sin(0.3*t) 0; sin(0.3*t) cos(0.3*t) 0; 0 0 1]*[2*cos(s); sin(2*s); 0.3*sin(s)+1];
        T = [cos(0.3*t) -sin(0.3*t) 0; sin(0.3*t) cos(0.3*t) 0; 0 0 1]*[-2*sin(s); 2*cos(2*s); 0.3*cos(s)];
        Tm = [cos(0.3*t) -sin(0.3*t) 0; sin(0.3*t) cos(0.3*t) 0; 0 0 1]*[-2*sin(s-ds); 2*cos(2*(s-ds)); 0.3*cos(s-ds)];
        TM = [cos(0.3*t) -sin(0.3*t) 0; sin(0.3*t) cos(0.3*t) 0; 0 0 1]*[-2*sin(s+ds); 2*cos(2*(s+ds)); 0.3*cos(s+ds)];
    
    case 5
        % Smooth square
        rr = (cos(s)^4 + sin(s)^4)^(-0.25); r = [rr*cos(s); rr*sin(s); 0*sin(s)+1];
        rrM = (cos(s+1e-8)^4 + sin(s+1e-8)^4)^(-0.25); rM = [rrM*cos(s+1e-8); rrM*sin(s+1e-8); 0*sin(s+1e-8)+1];    T = (rM-r)/1e-8;
        Tm = T;
        TM = T;
    
    case 6
        % Saddle
        r = [1.5*cos(s); 1.5*sin(s); 0.5*cos(t)*sin(s).^2+1];
        T = [-1.5*sin(s); 1.5*cos(s); cos(t)*2*0.5*sin(s).*cos(s)];
        Tm = [-1.5*sin(s-ds); 1.5*cos(s-ds); cos(t)*2*0.5*sin(s-ds).*cos(s-ds)];
        TM = [-1.5*sin(s+ds); 1.5*cos(s+ds); cos(t)*2*0.5*sin(s+ds).*cos(s+ds)];
    
    case 7
        % Crazy curve
        r = [1.1*cos(s)+0.5*cos(s+0.3); 1*sin(3*s)-sin(4*s-0.9); 0.5*sin(2*s)+1];
        T = [-1.1*sin(s)-0.5*sin(s+0.3); 3*cos(3*s)-4*cos(4*s-0.9); 2*0.5*cos(2*s)];
        Tm = [-1.1*sin(s-ds)-0.5*sin(s-ds+0.3); 3*cos(3*(s-ds))-4*cos(4*(s-ds)-0.9); 2*0.5*cos(2*(s-ds))];
        TM = [-1.1*sin(s+ds)-0.5*sin(s+ds+0.3); 3*cos(3*(s+ds))-4*cos(4*(s+ds)-0.9); 2*0.5*cos(2*(s+ds))];
    
    case 8
        % Line
        r = [tan((s-pi)/2); 0; 1];
        T = [(1/2)*1/(cos((s-pi)/2)^2); 0; 0];
        Tm = [(1/2)*1/(cos((s-ds-pi)/2)^2); 0; 0];
        TM = [(1/2)*1/(cos((s+ds-pi)/2)^2); 0; 0];
        
    case 9
        % Knot
        omega = 0;
        r = [cos(omega*t) -sin(omega*t) 0; sin(omega*t) cos(omega*t) 0; 0 0 1]*[sin(s)+2*sin(2*s); cos(s)-2*cos(2*s); -sin(3*s)];
        T = [cos(omega*t) -sin(omega*t) 0; sin(omega*t) cos(omega*t) 0; 0 0 1]*[cos(s)+4*cos(2*s); -sin(s)+4*sin(2*s); -3*cos(3*s)];
        Tm = [cos(omega*t) -sin(omega*t) 0; sin(omega*t) cos(omega*t) 0; 0 0 1]*[cos(s-ds)+4*cos(2*(s-ds)); -sin(s-ds)+4*sin(2*(s-ds)); -3*cos(3*(s-ds))];
        TM = [cos(omega*t) -sin(omega*t) 0; sin(omega*t) cos(omega*t) 0; 0 0 1]*[cos(s+ds)+4*cos(2*(s+ds)); -sin(s+ds)+4*sin(2*(s+ds)); -3*cos(3*(s+ds))];

    otherwise
        r = [0; 0; 0];
        T = [0; 0; 0];
        Tm = [0; 0; 0];
        TM = [0; 0; 0];
end

    N = ((TM/norm(TM) - Tm/norm(Tm))/(2*ds))/norm(T);



end