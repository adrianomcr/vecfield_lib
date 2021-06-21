function [Phi, Phi_v, Phi_c, Phi_hat, alpha_1, alpha_2, P] = get_field(point,param,field_str)



    xn = point(1);
    yn = point(2);
    zn = point(3);

    if strcmp(field_str,'saddle')
        
        a = param(1);
        b = param(2);
        Ksad = param(3);
        g1 = param(4);
        gain_G = param(5);
        
        alpha_1 = zn + Ksad*( a^(-2)*xn^2 - 1 );
        alpha_2 = a^(-2)*xn^(2) + b^(-2)*yn^2 - 1 ;
        
        P = g1*(1/2)*alpha_1^2 + (1/2)*alpha_2^2;
        
        grad_a_1 = [ 2*Ksad*a^(-2)*xn;
                    0;
                    1];
        grad_a_2 = [2*a^(-2)*xn;
                    2*b^(-2)*yn;
                    0];
        grad_P = g1*alpha_1*grad_a_1 + alpha_2*grad_a_2;
        
        cross_a1_a2 = cross(grad_a_1,grad_a_2);
        
        
        G = -(2/pi)*atan(gain_G*P^0.5);
%         G = -(2/pi)*atan(gain_G*P^1);
        H = sqrt(1-G^2);
        
        Phi_v = G*grad_P/(norm(grad_P)+10^-7);
        Phi_c = H*cross_a1_a2/(norm(cross_a1_a2)+10^-7);
        Phi = Phi_v+Phi_c;

        Phi_hat = [Phi(1)/(norm(Phi(1:2))+10^-7);
                   Phi(2)/(norm(Phi(1:2))+10^-7);
                   0];
               
    elseif strcmp(field_str,'shape')

        a = param(1);
        b = param(2);
        c = param(3);
        g1 = param(4);
        gain_G = param(5);
        
        alpha_1 = zn;
        alpha_2 = a*xn^4 - b*xn^2*yn^2 + c*yn^4 - 1 ;
        
        P = g1*(1/2)*alpha_1^2 + (1/2)*alpha_2^2;
        
        grad_a_1 = [0;
                    0;
                    1];
        grad_a_2 = [4*a*xn^3 - 2*b*xn*yn^2;
                    4*c*yn^3 - 2*b*xn^2*yn;
                    0];
        grad_P = g1*alpha_1*grad_a_1 + alpha_2*grad_a_2;
        
        cross_a1_a2 = cross(grad_a_1,grad_a_2);
        
        
        G = -(2/pi)*atan(gain_G*P^0.5);
%         G = -(2/pi)*atan(gain_G*P^1);
        H = sqrt(1-G^2);
        
        Phi_v = G*grad_P/(norm(grad_P)+10^-7);
        Phi_c = H*cross_a1_a2/(norm(cross_a1_a2)+10^-7);
        Phi = Phi_v+Phi_c;

        Phi_hat = [Phi(1)/(norm(Phi(1:2))+10^-7);
                   Phi(2)/(norm(Phi(1:2))+10^-7);
                   0];
        
    elseif strcmp(field_str,'random')

        a = param(1);
        b = param(2);
        c = param(3);
        g1 = param(4);
        gain_G = param(5);
        
        alpha_1 = zn;
        alpha_2 = a*xn^2 + b*xn^2*yn^2 + c*yn^2 - 1 ;
        
        P = g1*(1/2)*alpha_1^2 + (1/2)*alpha_2^2;
        
        grad_a_1 = [0;
                    0;
                    1];
        grad_a_2 = [2*a*xn^1 + 2*b*xn*yn^2;
                    2*c*yn^1 + 2*b*xn^2*yn;
                    0];
        grad_P = g1*alpha_1*grad_a_1 + alpha_2*grad_a_2;
        
        cross_a1_a2 = cross(grad_a_1,grad_a_2);
        

        Phi_v = -1*gain_G*grad_P;
        Phi_c = 1*cross_a1_a2;
        Phi = Phi_v+Phi_c;

        Phi_hat = [Phi(1)/(norm(Phi(1:2))+10^-7);
                   Phi(2)/(norm(Phi(1:2))+10^-7);
                   0];

               
               
               
    elseif strcmp(field_str,'poly')

        a = param(1);
        b = param(2);
        c = param(3);       
        d = param(4);
        g1 = param(5);
        gain_G = param(6);


        
        alpha_1 = zn;
        alpha_2 = a*xn^3 + b*xn^2 + c*xn^1 + d*xn^0 - yn;
        
        P = g1*(1/2)*alpha_1^2 + (1/2)*alpha_2^2;
        
        grad_a_1 = [0;
                    0;
                    1];
        grad_a_2 = [3*a*xn^2 + 2*b*xn^1 + 1*c*xn^0 + 0;
                    -1;
                    0];
        grad_P = g1*alpha_1*grad_a_1 + alpha_2*grad_a_2;
        
        cross_a1_a2 = cross(grad_a_1,grad_a_2);
        

        Phi_v = -1*gain_G*grad_P;
        Phi_c = 1*cross_a1_a2;
        Phi = Phi_v+Phi_c;

        Phi_hat = [Phi(1)/(norm(Phi(1:2))+10^-7);
                   Phi(2)/(norm(Phi(1:2))+10^-7);
                   0];

        

        
    else
        error('Specify which field')
    end

    









end