
global U_theta U_z U_v

tau_theta = 28;
tau_v = 20;
theta_asc=0;
tau_z = 20;

v_min = 18;
v_max = 28;
vz_max = 3;

global a b c d Ksad LIM_CENTER g1 scale_fac T_sim gain_G

curve = 1;

if (curve == 1)% Saddle ----------
    
    %Curve and parameters
    a = 600;
    b = 400;
    c = 0; d = 0;
    Ksad = 50; % ("heigth of the saddle")
    
    scale_fac = 1; %scale factor
    
    LIM_CENTER = 200;
    g1 = 10^(-5); % weigth of the function alpha_1 on P
    gain_G = 3; %gain of the function G(P)

    T_sim = 300+100;
    
    %Guidance controller gains
    Kp = 0.2; % orientation control gain
    Kv = 1; % velocity control gain to stablish a desired time conatant
    
    %Initial state - IROS_2018 - video
    posx_0 = 0;
    posy_0 = 0;
    theta_0 = pi;
    H_0 = 50+200;
    
    %Equilibrium velocity and shift on z
    Heq = 250; %m
    Veq = 23; %m/s
    
    U_z = 0.3; %U_z = 0;
    U_theta = 0.06; %U_theta = 0;       %OBS: U_theta = ErroMedicaoTheta/tau_theta
    U_v = 1; %U_v = 0;
    
elseif(curve == 2)% Shape ----------

    %Curve and parameters
    a = 6;
    b = 0;
    c = 18;
    Ksad = 0; d = 0;
    
    scale_fac = 1000; %scale factor
    
    LIM_CENTER = 0.2;
    
    gain_G = 2;
    g1 = 10; % weigth of the function alpha_1 on P
    T_sim = 300;
    
    %Guidance controller gains
    Kp = 0.18; % orientation control gain
    Kv = 1; % velocity control gain to stablish a desired time conatant

    %Initial state
    posx_0 = 500*(2*rand(1)-1);
    posy_0 = 500*(2*rand(1)-1);
    theta_0 = 200*rand(1);
    H_0 = 50 + 150;

    %Initial state - IROS_2018
    posx_0 = -300;
    posy_0 = 0;
    theta_0 = pi/6;
    H_0 = 200;
    
    %Initial state - IROS_2018 - video
    posx_0 = 200;
    posy_0 = 50;
    theta_0 = -7*pi/8;
    H_0 = 200;
    
    %Equilibrium velocity and shift on z
    Heq = 200; %m
    Veq = 23; %m/s
    
    U_z = 0.3/1; %U_z = 0;
    U_theta = 0.06/1; %U_theta = 0;       %OBS: U_theta = ErroMedicaoTheta/tau_theta
    U_v = 0.3; %U_v = 0;
   
    
elseif(curve == 3)% Random ----------
    
    %Curve and parameters
    a = 1.8;
    b = 12;
    c = 3;
    Ksad = 0; d = 0;
    a = 1.5;
    b = 8;
    c = 2.5;
    
    scale_fac = 1000; %scale factor
    
    LIM_CENTER = 0.20;
    
    gain_G = 1.5;
    g1 = 4; % weigth of the function alpha_1 on P
    
    T_sim = 300+100;
    
    %Guidance controller gains
    Kp = 0.18; % orientation control gain
    Kv = 1; % velocity control gain to stablish a desired time conatant

     
    %Initial state
    posx_0 = 500*(2*rand(1)-1);
    posy_0 = 500*(2*rand(1)-1);
    theta_0 = 200*rand(1);

    % Losangle for CBA video
    posx_0 = 500*(2*rand(1)-1); posx_0 = 100;
    posy_0 = 500*(2*rand(1)-1); posy_0 = 350;
    theta_0 = 200*rand(1); theta_0 = -pi/2;
    
    H_0 = 200;
    
    %Equilibrium velocity and shift on z
    Heq = 200; %m
    Veq = 23; %m/s
    
    U_z = 0.3/1; %U_z = 0;
    U_theta = 0.06/1; %U_theta = 0;       %OBS: U_theta = ErroMedicaoTheta/tau_theta
    U_v = 0.2; %U_v = 0;
    
    
elseif(curve == 4)% Polynomial ----------

    %ALPHA = a*X.^3 + b*X.^2 + c*X.^1 + d*X.^0 - yn;
    
    %Curve and parameters
    a = 0.85;
    b = 0.08;
    c = -0.13;
    d = -0.04;
    Ksad = 0;
    
    scale_fac = 1000; %scale factor
    
    LIM_CENTER = 0.2;

    gain_G = 2.5;
    g1 = 4; % weigth of the
    T_sim = 120;
    
    %Guidance controller gains
    Kp = 0.18; % orientation control gain
    Kv = 1; % velocity control gain to stablish a desired time conatant

     
    %Initial state
    posx_0 = -750 + 250*(2*rand(1)-1);
    posy_0 = -600 + 200*(2*rand(1)-1);
    theta_0 = pi*(2*rand(1)-1);
  
    H_0 = 200;
    
    %Equilibrium velocity and shift on z
    Heq = 200; %m
    Veq = 23; %m/s
    
    U_z = 0.3/1; %U_z = 0;
    U_theta = 0.06/1; %U_theta = 0;       %OBS: U_theta = ErroMedicaoTheta/tau_theta
    U_v = 0.2; %U_v = 0;
    
end
% ----------  ----------

%Uncertainty

freq_divisor = 7;

%Measurement disturbace
% For theta
amp_theta = ((1/4)*[1 1 1]/6)*U_theta*tau_theta; %rad %OBS only 1/2 of U_theta is applied
f_theta = ([5 3 1]+rand(1,3))*0.05*2*pi/freq_divisor; %frequency %rad/s
p_theta = rand(1,3)*2*pi; %phase %rad
%For x
amp_x = [5 5 5]; %m
f_x = ([6 4 2]+rand(1,3))*0.05*2*pi/freq_divisor;
p_x = rand(1,3)*2*pi;
%For y
amp_y = [5 5 5]; %m
f_y = ([6 4 2]+rand(1,3))*0.05*2*pi/freq_divisor;
p_y = rand(1,3)*2*pi;
%For z
amp_z = [1 1 1]*(tau_z*U_z)/6/2; %m
f_z = ([6 4 2]+rand(1,3))*0.05*2*pi/freq_divisor;
p_z = rand(1,3)*2*pi;

amp_theta = amp_theta*2;
amp_z = amp_z*2;

%Actuation disturbace
% For theta_c
amp_theta_c = ((1/4)*[1 1 1]/6)*U_theta*tau_theta; %rad %OBS only 1/2 of U_theta is applied
f_theta_c = ([5 3 1]+rand(1,3))*0.05*2*pi/freq_divisor;
p_theta_c = rand(1,3)*2*pi;
%For v_c
amp_v_c = [1 1 1]*(tau_v*U_v)/6; %m/s
f_v_c = ([4 2 0.5]+rand(1,3))*0.05*2*pi/freq_divisor;
p_v_c = rand(1,3)*2*pi;
%For z_c
amp_z_c = [1 1 1]*(tau_z*U_z)/6/2; %m
f_z_c = ([4 2 0.5]+rand(1,3))*0.05*2*pi/freq_divisor;
p_z_c = rand(1,3)*2*pi;
