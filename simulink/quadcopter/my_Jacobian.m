function [Jac, Jt]  = my_Jacobian(p, time, vr, Kf, curva)

delta_x = 0.0001;
delta_y = 0.0001;
delta_z = 0.0001;
delta_t = 0.008;

% [F, ~] = compute_dist_field(p, vr, Kf);
[F_Mx, ~] = compute_dist_field(p+[1;0;0]*delta_x, time, vr, Kf, curva);
[F_My, ~] = compute_dist_field(p+[0;1;0]*delta_y, time, vr, Kf, curva);
[F_Mz, ~] = compute_dist_field(p+[0;0;1]*delta_z, time, vr, Kf, curva);
[F_Mt, ~] = compute_dist_field(p, time+delta_t, vr, Kf, curva);


[F_mx, ~] = compute_dist_field(p-[1;0;0]*delta_x, time, vr, Kf, curva);
[F_my, ~] = compute_dist_field(p-[0;1;0]*delta_y, time, vr, Kf, curva);
[F_mz, ~] = compute_dist_field(p-[0;0;1]*delta_z, time, vr, Kf, curva);
[F_mt, ~] = compute_dist_field(p, time-delta_t, vr, Kf, curva);

% Jx = (F_Mx - F)/delta_x;
% Jy = (F_My - F)/delta_y;
% Jz = (F_Mz - F)/delta_z;

Jx = (F_Mx - F_mx)/(2*delta_x);
Jy = (F_My - F_my)/(2*delta_y);
Jz = (F_Mz - F_mz)/(2*delta_z);

Jt = (F_Mt - F_mt)/(2*delta_t);

Jac = [Jx Jy Jz];



end %function


