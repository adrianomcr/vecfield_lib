function f = compute_dist_field(p)

    global r cost Kf

    s_min = 0;
    D_min = cost(p,0);
    sv = linspace(0,2*pi,20);
    for k = 2:1:length(sv)
        if(cost(p,sv(k)) < D_min)
            s_min = sv(k);
            D_min = cost(p,s_min);
        end
    end
    
    %Compute the parametes of the closest point
    s_star = fminbnd(@(s) cost(p,s),s_min-pi/10,s_min+pi/10);
    %Compute the closest point on the curve
    p_star = r(s_star);
    
    %Distance vector
    D_vec = p-p_star;
    %Distance function
    D = norm(D_vec);

    %Gradient of the distance function
    grad_D = D_vec/(D+1e-6);

    %Tangent vector
    T = r(s_star+1e-4)-r(s_star-1e-4);
    T = T/norm(T);

    %Compute weights for convergence and circulation
    G = -(2/pi)*atan(Kf*D);
    H = sqrt(1-G^2);
    
    %Compute vector field
    f = G*grad_D + H*T;
    
end