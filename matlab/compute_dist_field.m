function f = compute_dist_field(p)

    global r cost Kf

    %Compute the parametes of the closest point
    s_star = fminbnd(@(s) cost(p,s),0,2*pi);
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