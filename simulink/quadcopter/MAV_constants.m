



% Curve example
curva = 9;
% ----- Options ----- %
% 1 - Horizontal circle
% 2 - Vertical circle
% 3 - Spiral
% 4 - Eight like curve
% 4 - Eight like curve
% 5 - Smooth square
% 6 - Saddle
% 7 - Crazy curve
% 8 - Straight line
% 9 - Knot

Delta_tau = 1.0;
Delta_omega = 0.5;

% Simulation parameters
T_sim = 12; % simulation duration
dt_sim = 0.025; % time step

% Model parameters
g = 9.81; % gravity
m = 0.5; % mass
L = 0.2; % arm length
Ix = 4.85e-3; % inertial moment on x axis
Iy = 4.85e-3; % inertial moment on y axis
Iz = 8.81e-3; % inertial moment on z axis
IR = 3.36e-5; % inertial moment of the proppelers
b = 2.92e-6; % 
d = 1.12e-7; % 

%Temporary drag
drag_coef = 0.05;

% Desired velocity
vr = 4;


%Controoller gains
Kf = 1; % field
Kv = 2; % velocity
Kr = 5; % orientation

%Initial conditions
xyz_0 = [-2 -2 -2] + 0.001; %position
xyz_d_0 = [0 1 0] + 0.001; %velocity
Rot_0 = eul2rotm(fliplr([pi/4 pi/2 pi])); % orientation (roll, pitch, yaw)
