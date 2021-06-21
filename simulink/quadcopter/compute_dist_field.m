function [Phi, D] = compute_dist_field(p, time, vr, Kf, curva)

    delta_t = 0.001; 

    % Sequence of points representing the curve at instants t and t+
    theta_vec = linspace(0,2*pi,50);
    d_theta = theta_vec(2)-theta_vec(1);
    curve = zeros(3,length(theta_vec));
    curve_Mt = zeros(3,length(theta_vec));
    for i = 1:1:length(theta_vec)
        [curve(:,i),~,~] = my_curve(theta_vec(i), time, curva);
%         [curve_Mt(:,i),~,~] = my_curve(theta_vec(i), time+delta_t, curva);
    end
    
    
    
    %Computation of the distance vector (loop + aurea section)
    D = inf;
    for i = 1:1:length(curve(1,:))
        if(norm(p - curve(:,i)) < D)
            D_vec = p-curve(:,i);
            D = norm(D_vec);
            i_close = i;
        end
    end
%     D_Mt = inf;
%     for i = 1:1:length(curve_Mt(1,:))
%         if(norm(p - curve_Mt(:,i)) < D_Mt)
%             D_vec_Mt = p-curve_Mt(:,i);
%             D_Mt = norm(D_vec_Mt);
%             i_close_Mt = i;
%         end
%     end
    theta_opt = golden_search(theta_vec(i_close)-1.1*d_theta, theta_vec(i_close)+1.1*d_theta, p, time, curva);
    [curve_opt,T,~] = my_curve(theta_opt, time, curva);
%     theta_opt_Mt = golden_search(theta_vec(i_close_Mt)-1.1*d_theta, theta_vec(i_close_Mt)+1.1*d_theta, p, time+delta_t, curva);
%     [curve_opt_Mt,T_Mt,~] = my_curve(theta_opt_Mt, time+delta_t, curva);
    theta_opt_mt = golden_search(theta_vec(i_close)-3.1*d_theta, theta_vec(i_close)+3.1*d_theta, p, time-delta_t, curva);
    [curve_opt_mt,~,~] = my_curve(theta_opt_mt, time-delta_t, curva);
    theta_opt_Mt = golden_search(theta_vec(i_close)-3.1*d_theta, theta_vec(i_close)+3.1*d_theta, p, time+delta_t, curva);
    [curve_opt_Mt,~,~] = my_curve(theta_opt_Mt, time+delta_t, curva);
    %Normalize tangent vector
    T = T/norm(T);
%     T_Mt = T_Mt/norm(T_Mt);
    %Compute distance vector
    D_vec = p-curve_opt;
    D_vec_mt = p-curve_opt_mt;
    D_vec_Mt = p-curve_opt_Mt;
    %Compute scalar distance function
    D = norm(D_vec);
%     D_Mt = norm(D_vec_Mt);
    %Compute the gradient of the scalar distance function
    grad_D = D_vec/(D+1e-10);
%     grad_D_Mt = D_vec_Mt/(D_Mt+1e-10);
 
    
    %Feed-forward term
%     Psi_t = (D_vec_Mt-D_vec)/delta_t;
    Psi_t = -(D_vec_Mt-D_vec_mt)/(2*delta_t);
    
    %Computation of the tangent vector
%     [curve_opt_M,~,~] = my_curve(theta_opt+1e-10, curva);
%     [curve_opt_m,~,~] = my_curve(theta_opt-1e-10, curva);
%     T = (curve_opt_M-curve_opt_m); T = T/norm(T);

    
    %Lyapunov function
    P = (1/2)*D^2;
    
    %Ponderation functions
    G = -(2/pi)*atan(Kf*sqrt(P));
    H = sqrt(1-G^2);
    
    % Normalization
    Psi_g = G*grad_D;
    Psi_h = H*T;
    Psi_s = Psi_g + Psi_h;
    
    eta = -Psi_s'*Psi_t + sqrt((Psi_s'*Psi_t)^2 + vr^2 - Psi_t'*Psi_t);
    
%     Psi = G*grad_D + H*T;
%     Phi = vr*Psi;
    Phi = eta*Psi_s + Psi_t;




end % function