function [G_Phi_x,G_Phi_y] = get_grad(point,param,field_str)


    xn = point(1);
    yn = point(2);
    zn = point(3);

    % Delta for the numeric approximation
    delta_x = 1e-4;
    delta_y = 1e-4;
    delta_z = 1e-4;
    %Variations on x, y and z
    xnM = xn+delta_x;
    ynM = yn+delta_y;
    znM = zn+delta_z;
    
    
    [~, ~, ~, Vxyz_hat, ~, ~, ~] = get_field([xn,yn,zn],param,field_str);
    Vx = Vxyz_hat(1);
    Vy = Vxyz_hat(2);
    Vz = Vxyz_hat(3);

    [~, ~, ~, Vxyz_hat, ~, ~, ~] = get_field([xnM,yn,zn],param,field_str);
    Vx_xM = Vxyz_hat(1);
    Vy_xM = Vxyz_hat(2);
    Vz_xM = Vxyz_hat(3);

    [~, ~, ~, Vxyz_hat, ~, ~, ~] = get_field([xn,ynM,zn],param,field_str);
    Vx_yM = Vxyz_hat(1);
    Vy_yM = Vxyz_hat(2);
    Vz_yM = Vxyz_hat(3);

    [~, ~, ~, Vxyz_hat, ~, ~, ~] = get_field([xn,yn,znM],param,field_str);
    Vx_zM = Vxyz_hat(1);
    Vy_zM = Vxyz_hat(2);
    Vz_zM = Vxyz_hat(3);

    G_Phi_x = [(Vx_xM-Vx)/delta_x; (Vx_yM-Vx)/delta_y; (Vx_zM-Vx)/delta_z];
    G_Phi_y = [(Vy_xM-Vy)/delta_x; (Vy_yM-Vy)/delta_y; (Vy_zM-Vy)/delta_z];
    





end