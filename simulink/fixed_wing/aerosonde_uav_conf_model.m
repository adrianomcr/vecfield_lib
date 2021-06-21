
% ----------------- PARAMETERS OF THE UAV MODEL ----------------------------------
% -------------------------------------------------------------------------

degrees2rad = pi/180;

%vari�veis globais do modelo da aeronave
global delta_CD delta_CY delta_CL delta_Cm delta_Cn delta_Cl delta_FT  

global perc_min_CD perc_max_CD perc_min_CY perc_max_CY perc_min_CL perc_max_CL
global perc_min_Cl perc_max_Cl perc_min_Cm perc_max_Cm perc_min_Cn perc_max_Cn
global perc_min_FT perc_max_FT
global w GP;
global enable_latero_dynamics enable_long_dynamics enable_controller_freq;
global A_i B_i C_i D_i A_ext_i B_ext_i C_ext_i D_ext_i; % modelos incertos 

w = logspace(-3,3,64);

%vari�vies globais de perturba��o
global gain_xw num_xw den_xw;
global gain_yw num_yw den_yw;
global gain_zw num_zw den_zw;
global file_pareto_inner_loops file_pareto_outer_loops;

%vari�veis globais diversas
global x0_uav u0_uav De_long Du_long Du_vt Dd_long De_lat Du_lat Dd_lat name_uav Nmodels;
global De Du Dd De_h De_vt;
global u0_uav_uncertain x0_uav_uncertain;
name_uav = 'AeroSonde';

%Number of uncertain models
if (enable_latero_dynamics & enable_long_dynamics)
    Nmodels = 128;
elseif(enable_long_dynamics)
    Nmodels = 16;
end;

%gr�ficos das tabelas aerodin�micas
plot_aero_tables = 0;

%elimina os estados P_N (m) e P_E (m) para possibilitar a trimagem do modelo 
%isTrimming = 1;
isTrimming = 0;

%aeronave c/ ou s/ cauda
global tailless
tailless = 0;

% estrat�gia de controle throttle from altitude 
global altitude_thro
altitude_thro = 0;

%Condi��o de trimagem do modelo
global Heq Veq;
Veq = 23;
Heq = 200;

%x0_uav_uncertain = x0_uav;        
        
%mean chord (m)
Cw = 0.189941;

%wing span (m)
Bw = 2.8956; % m

%wing area (m)
Sw = 0.55; % m^2

%aspect ratio
AR = (Bw^2)/Sw;

% Oswald's coefficient
osw = 0.75;

%gravidade (m/s^2)     
gravity = 9.807;

%massa da aeronave (Kg)
M = 8.5;

%componentes de in�rcia da aeronave
Jx = 0.7795;
Jy = 1.122;
Jz = 1.752;
Jxz = 0.1211;

%posi��o do c.a. no eixo estrutural (m)
r_aero_struct =    [0.1425 0 0];

%posi��o do c.g. no eixo estrutural (m)
r_cg_struct =      [0.156 0 0];

%posi��o do motor no eixo estrutural (m)
r_motor_struct =   [0.23 0 0]; % m    

%posi��o da empenagem no eixo estrutural (m) 
r_tail_struct =    r_cg_struct;

%convers�o do eixo estrutural para o ABC (m)
r_aero_abc  = r_aero_struct*[-1 0 0;0 1 0;0 0 -1];
r_cg_abc    = r_cg_struct*[-1 0 0;0 1 0;0 0 -1];  
r_motor_abc = r_motor_struct*[-1 0 0;0 1 0;0 0 -1];        
r_tail_abc  = r_aero_struct*[-1 0 0;0 1 0;0 0 -1];

% -------------------------------------------------------------------------
% ------------------------ TAIL -------------------------------------------
% -------------------------------------------------------------------------

% "bra�o de alavanca" da empenagem (m)
xt = 0;

% efeito de down wash
d_wash = 0;

% �rea da empenagem horizontal  (m^2)
Sh = 0;
% -------------------------------------------------------------------------
% ------------------------ SERVOS------------------------------------------
% -------------------------------------------------------------------------

global elevator_max;

% LImita��o na atua��es dos servos (rad)
elevator_limits = [-15*pi/180;15*pi/180];
throttle_limits = [0;1.0];
rudder_limits = [-20*pi/180;20*pi/180];
aileron_limits = [-20*pi/180;20*pi/180];
elevator_max = elevator_limits(2);


% -------------------------------------------------------------------------
% ------------------------ AERODIN�MICOS-----------------------------------
% -------------------------------------------------------------------------

%range das vari�veis \alpha (rad) e \beta (rad)
range_alpha = -2:16;
AlphaBnd = range_alpha.*degrees2rad;
range_beta = -10:10;
BetaBnd = range_beta.*degrees2rad;
range_P = -30:30;
PBnd = range_P.*degrees2rad;
range_Q = -30:30;
QBnd = range_Q.*degrees2rad;
range_R = -30:30;
RBnd = range_R.*degrees2rad;
range_elev = -20:20;
ElevBnd = range_elev.*degrees2rad;
range_ail = -20:20;
AilBnd = range_ail.*degrees2rad;
range_rud = -20:20;
RudBnd = range_rud.*degrees2rad;

%percentuais de incerteza dos par�metros do modelo
perc_min_CD = - 0.15;  
perc_max_CD = + 0.15;
perc_min_CY = - 0.15;
perc_max_CY = + 0.15;
perc_min_CL = - 0.15;
perc_max_CL = + 0.15;
perc_min_Cl = - 0.15;
perc_max_Cl = + 0.15;
perc_min_Cm = - 0.15;
perc_max_Cm = + 0.15;
perc_min_Cn = - 0.15;
perc_max_Cn = + 0.15;
perc_min_FT = - 0.15;
perc_max_FT = + 0.15;

%---------------------------Drag Force-------------------------------------
%coeficientes aerodin�micos de arrasto
CD_alpha_beta = zeros(length(AlphaBnd),length(BetaBnd));  %tabela
CD_aux_elev = abs(0.0135.*ElevBnd);
CD_elev = ones(length(AlphaBnd),1)*CD_aux_elev;
CD_aux_ail = abs(0.0302.*AilBnd);
CD_ail = ones(length(AlphaBnd),1)*CD_aux_ail;
CD_aux_rud = abs(0.0303.*RudBnd);
CD_rud = ones(length(AlphaBnd),1)*CD_aux_rud;
CD_mind = 0.23;
CD_0 = 0.0434;
induced_drag = 1/(pi*AR*osw);

if (plot_aero_tables)
    
    figure;
    surf(range_beta,range_alpha,CD_alpha_beta);
    xlabel('\beta (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_D ( \alpha, \beta)');
    
    figure;
    surf(range_elev,range_alpha,CD_elev);
    xlabel('\delta_{elev} (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_D ( \alpha, \delta_{elev})');
    
    figure;
    surf(range_ail,range_alpha,CD_ail);
    xlabel('\delta_{ail} (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_D ( \alpha, \delta_{ail})');
    
end


%---------------------------Side Force-------------------------------------
%coeficientes aerodin�micos de for�a lateral
CY_beta = BetaBnd.*-0.83; 
CY_alpha_beta = ones(length(AlphaBnd),1)*CY_beta; 
CY_aux_ail = -0.075.*AilBnd;
CY_ail = ones(length(AlphaBnd),1)*CY_aux_ail;
CY_aux_rud = 0.1914.*RudBnd;
CY_rud = ones(length(AlphaBnd),1)*CY_aux_rud;
CY_P = zeros(length(AlphaBnd),length(PBnd));
CY_R = zeros(length(AlphaBnd),length(RBnd));

if (plot_aero_tables)
    
    figure;
    surf(range_beta,range_alpha,CY_alpha_beta);
    xlabel('\beta (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_Y ( \alpha, \beta)');

    figure;
    surf(range_ail,range_alpha,CY_ail);
    xlabel('\delta_{ail}  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_Y ( \alpha, \delta_{ail} )');

    figure;
    surf(range_P,range_alpha,CY_P);
    xlabel('P  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_Y ( \alpha, P )');

    figure;
    surf(range_R,range_alpha,CY_R);
    xlabel('R  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_Y ( \alpha, R )');    
    
end

%---------------------------Lift Force-------------------------------------

%----------------------------Asa-------------------------------------------
%coeficientes aerodin�micos da for�a de sustenta��o
CL_alpha = (AlphaBnd.*5.6106)'; 
CL_alpha_beta = CL_alpha*ones(1,length(BetaBnd));
CL_0 = 0.23;
CL_aux_elev = 0.13.*ElevBnd;
CL_elev = ones(length(AlphaBnd),1)*CL_aux_elev;
CL_alphadot =  1.9724; 
CL_Q = 7.9543*QBnd;

if (plot_aero_tables)
    
    figure;
    surf(range_beta,range_alpha,CL_alpha_beta);
    xlabel('\beta (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_L ( \alpha, \beta)');  
  
    figure;
    surf(range_elev,range_alpha,CL_elev);
    xlabel('\delta_{elev}  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_L ( \alpha, \delta_{elev})');  
    
    figure;
    plot(range_Q,CL_Q);
    xlabel('Q (graus)');
    ylabel('C_L ( Q)');  
 
end

%----------------------------Empenagem-------------------------------------
% coeficientes aerodin�micos da cauda
CLE_alphat = zeros(1,length(AlphaBnd));    %tabela
CLE_elev = 0;

%--------------------------Rolling Moment----------------------------------
% coeficientes aerodin�micos de rolagem
Cl_beta = BetaBnd.*-0.13; 
Cl_alpha_beta = ones(length(AlphaBnd),1)*Cl_beta; 
Cl_aux_ail = -0.1695.*AilBnd;
Cl_ail = ones(length(AlphaBnd),1)*Cl_aux_ail;
Cl_aux_rud = 0.0024.*RudBnd;
Cl_rud = ones(length(AlphaBnd),1)*Cl_aux_rud;
Cl_aux_P = -0.5051.*PBnd;
Cl_P = ones(length(AlphaBnd),1)*Cl_aux_P;
Cl_aux_R = 0.2519.*RBnd;
Cl_R = ones(length(AlphaBnd),1)*Cl_aux_R;

if (plot_aero_tables)

    figure;
    surf(range_beta,range_alpha,Cl_alpha_beta);
    xlabel('\beta (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_l ( \alpha, \beta)');

    figure;
    surf(range_ail,range_alpha,Cl_ail);
    xlabel('\delta_{ail}  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_l ( \alpha, \delta_{ail} )');

    figure;
    surf(range_P,range_alpha,Cl_P);
    xlabel('P  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_l ( \alpha, P )');

    figure;
    surf(range_R,range_alpha,Cl_R);
    xlabel('R  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_l ( \alpha, R )'); 

end

%--------------------------Pitching Moment---------------------------------
% coeficientes aerodin�micos de arfagem
Cm0 = 0.135;
Cm_alpha = (AlphaBnd.*-2.7397)';  
Cm_alpha_beta = Cm_alpha*ones(1,length(BetaBnd));
Cm_aux_elev = ElevBnd.*-0.9918;    
Cm_elev = ones(length(AlphaBnd),1)*Cm_aux_elev;
Cm_alphadot = -10.3796;
Cm_Q = -38.2067*QBnd;                   

if (plot_aero_tables)
    
    figure;
    surf(range_beta,range_alpha,Cm_alpha_beta);
    xlabel('\beta (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_m ( \alpha, \beta)');  
  
    figure;
    surf(range_elev,range_alpha,Cm_elev);
    xlabel('\delta_{elev}  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_m ( \alpha, \delta_{elev})');  
    
    figure;
    plot(range_Q,Cm_Q);
    xlabel('Q (graus)');
    ylabel('C_m ( Q)');  
 
end


%--------------------------Yawing Moment-----------------------------------
%coeficientes aerodin�micos de guinada
Cn_beta = BetaBnd.*0.0726; 
Cn_alpha_beta = ones(length(AlphaBnd),1)*Cn_beta; 
Cn_aux_ail = 0.0108.*AilBnd;
Cn_ail = ones(length(AlphaBnd),1)*Cn_aux_ail;
Cn_aux_rud = -0.0693.*RudBnd;
Cn_rud = ones(length(AlphaBnd),1)*Cn_aux_rud;
Cn_aux_P = -0.069.*PBnd;
Cn_P = ones(length(AlphaBnd),1)*Cn_aux_P;
Cn_aux_R = -0.0946.*RBnd;
Cn_R = ones(length(AlphaBnd),1)*Cn_aux_R;

if (plot_aero_tables)

    figure;
    surf(range_beta,range_alpha,Cn_alpha_beta);
    xlabel('\beta (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_n ( \alpha, \beta)');

    figure;
    surf(range_ail,range_alpha,Cn_ail);
    xlabel('\delta_{ail}  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_n ( \alpha, \delta_{ail} )');

    figure;
    surf(range_P,range_alpha,Cn_P);
    xlabel('P  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_n ( \alpha, P )');

    figure;
    surf(range_R,range_alpha,Cn_R);
    xlabel('R  (graus)');
    ylabel('\alpha (graus)');
    zlabel('C_n ( \alpha, R )'); 
    
end;

% -------------------------------------------------------------------------
% ------------------------ MOTOR ------------------------------------------
% -------------------------------------------------------------------------

% variav�l presente no simulador chavea o pot�ncia do motor obtido via
% constante ou modelo completo
simple_motor_model = 1;

F_T = 19; %N

%di�metro da h�lice
D_hel = 0.15;   

%obt�m par�metros do motor utilizado
load('helice.mat');

CT_save = CT_save.*10; 
rpm_save = rpm_save.*2;
J_save = J_save.*1.5;

Delta_V_J  = [ 0 0.4 0.7 1.4 2.1];
Delta_V_EH = [ 2 1.7365 1.2985 1.0890 1.0205];

% ------------------ CONTROLE - NORMALIZA��O FTs --------------------------
global theta_max phi_max vt_max h_max R_max psi_max;

%Limites das vari�veis de controle
theta_max = 10*pi/180; %em rad
theta_min = -5*pi/180; %em rad
phi_max = 20*pi/180; %em rad
phi_min = -20*pi/180; %em rad
vt_max = 4;  %em m/s
yacc_max = 2;  %em m/s^2
R_max = 2;  %em m/s^2
h_max = 10;

%taxas m�ximas de varia��es na vari�veis
servo_rate = 60*degrees2rad;
motor_rate = 1;%0.305;
theta_rate = 1;
roll_rate = 30*(pi/180);

%valores m�ximos dos integradores
perc_int_theta_max = 0.8;%1;
perc_int_phi_max = 0.8;%0.5;
perc_int_vt_max = 0.8;%0.8;
perc_int_h_max = 0.8;%0.72;
perc_int_yacc_max = 0.8;%0.5;
motor_min = 0.1;

%erro m�ximo de velocidade da aeronave
turb_max = 6;     % considerando que a turbulencia n�o pode passar de 40% da velocidade nominal.  
  
%------------matrizes limites m�ximos din�mica completa--------------------

%aeronave sem cauda
if(tailless)

    if(altitude_thro)
        
        De = [  2*theta_max 0 0;    
                0 2*phi_max 0; 
                0  0 h_max ]; 
            
    else
        
        De = [  2*theta_max 0 0;    
                0 2*phi_max 0; 
                0  0 vt_max ]; 
        
    end

    Du = [  elevator_limits(2) 0 0;    
            0 aileron_limits(2) 0; 
            0  0 throttle_limits(2) ]; 

    Dd = [  turb_max 0 0;    
            0 turb_max 0; 
            0  0 turb_max ];     
        
    De_vt = De(3,3);
    Du_vt = Du(3,3);

%aeronave com cauda     
else
    
    if(altitude_thro)
    
        De = [  2*theta_max 0 0 0;    
                0 2*phi_max 0 0; 
                0 0 R_max 0; 
                0 0 0 h_max ]; 
    else
            
        De = [  2*theta_max 0 0 0;    
                0 2*phi_max 0 0; 
                0 0 R_max 0; 
                0 0 0 vt_max ]; 
    end;

    Du = [  elevator_limits(2) 0 0 0;    
            0 aileron_limits(2) 0 0; 
            0 0 rudder_limits(2) 0;
            0 0 0 throttle_limits(2) ];     

    Dd = [  turb_max 0 0;    
            0 turb_max 0; 
            0  0 turb_max ];     
        
    De_vt = De(4,4);
    Du_vt = Du(4,4);
    
    De_h = De(4,4);
    
end
    
%---------------------matrizes limites m�ximos long------------------------
%valores de erros m�ximos

if(altitude_thro)
    De_long = [ 2*theta_max 0;
                0  h_max];    
else
    De_long = [ 2*theta_max 0;
                0  vt_max ];  
end;
%entradas m�ximas
Du_long = [ elevator_limits(2) 0;
            0 throttle_limits(2) ];

%entrada de perturba��o
Dd_long =  [ turb_max 0; 
            0 turb_max ];
    
%---------------------matrizes limites m�ximos latero----------------------    
%valores de erros m�ximos

if(tailless)
    De_lat = [ 2*phi_max];
    %entradas m�ximas
    Du_lat = [ aileron_limits(2)];
else
    De_lat = [ 2*phi_max 0;
                0  R_max];
    %entradas m�ximas
    Du_lat = [ aileron_limits(2) 0;
                0 rudder_limits(2)];    
end;
%entrada de perturba��o
Dd_lat =  [ turb_max ];     
    
% -------------- CONTROLE - DESEMPENHO M�NIMO MALHAS ----------------------
global time_sim_external_loops;
global ts_sim time_sim_internal_loops  time_degrau_theta time_degrau_vt time_degrau_h time_degrau_phi time_degrau_psi;
global t_a_p_c_theta t_a_p_c_vt t_a_r_d_vt t_a_r_d_theta t_a_r_d_h t_a_r_d_phi t_a_p_c_phi t_a_r_d_psi; 
global overshoot_vt_max overshoot_theta_max overshoot_h_max overshoot_phi_max overshoot_psi_max;
global KC_theta TI_theta TD_theta KC_vt TI_vt KC_h TD_h TI_h KT rho;
global KC_phi TI_phi TD_phi KC_r TI_r TD_r KR;
global KC_psi TD_psi KE;

% ts_sim = 0.001;   % periodo de amostragem da simula�ao  
ts_sim = 0.01;

if(altitude_thro)
    t_a_r_d_vt = 60;
    t_a_p_c_vt = 50;
    t_a_r_d_theta = 20;
    t_a_p_c_theta = 30;
else
    t_a_r_d_vt = 20;
    t_a_p_c_vt = 20;
    t_a_r_d_theta = 12;
    t_a_p_c_theta = 12;
end;

t_a_r_d_phi = 12;
t_a_p_c_phi = 12;

t_a_r_d_h = 15;
t_a_r_d_psi = 25;
overshoot_vt_max = 15;
overshoot_h_max = 10;
overshoot_psi_max = 10;
overshoot_theta_max = 20;
overshoot_phi_max = 10;

time_sinc = 15;
%internal
time_sim_internal_loops = time_sinc + 4*t_a_r_d_theta + 4*t_a_r_d_vt + 4*t_a_r_d_phi + time_sinc;   
time_degrau_theta = time_sinc + 4*t_a_r_d_vt + 4*t_a_r_d_phi;
time_degrau_phi = time_sinc + 4*t_a_r_d_vt;
time_degrau_vt = time_sinc;

%
time_sim_external_loops = time_sinc + 4*t_a_r_d_h + 4*t_a_r_d_psi + time_sinc; 
time_degrau_h = time_sinc;
time_degrau_psi = time_sinc + 4*t_a_r_d_h;


global amp_degrau_theta amp_degrau_vt amp_degrau_h_vt_ext amp_degrau_phi amp_degrau_psi amp_degrau_h;
amp_degrau_theta = 0.25;        % 25% da faixa total de excurs�o
amp_degrau_phi = 0.25; 
amp_degrau_vt = 0.25;           % 25% da faixa total de excurs�o
amp_degrau_h = 10;
amp_degrau_vt_ext = 1;
amp_degrau_psi = 30;

if(altitude_thro)
    amp_degrau_h_vt_ext = amp_degrau_vt_ext;
else
    amp_degrau_h_vt_ext = amp_degrau_h;
end;

h_max = amp_degrau_h;
psi_max = amp_degrau_psi;

% simula��o avaliacao PID malhas externas e internas 
time_sim_avaliacao_loops = time_sinc + 4*t_a_r_d_h + 4*t_a_r_d_vt + 4*t_a_r_d_psi + time_sinc;
time_degrau_vt_h = time_sinc + 4*t_a_r_d_h + 4*t_a_r_d_psi;

time_sim_external_loops = time_sinc + 4*t_a_r_d_h + 4*t_a_r_d_vt + 4*t_a_r_d_psi + time_sinc;

if(altitude_thro)
    amp_degrau_vt_h_ext = amp_degrau_h;
else
    amp_degrau_vt_h_ext = amp_degrau_vt_ext;    
end;

% par�metros de incertezas default para o modelo nominal
%valores para o modelo nominal
delta_CD = 1;
delta_CY = 1;
delta_CL = 1;
delta_Cl = 1;
delta_Cm = 1;
delta_Cn = 1;
delta_FT = 1;

        

%% Acrescimos Tales, Leo e Lucas em fev./marco/abril de 2013.
% Tempo total 
time_sim_external_loops = 300;


delta_raio = 30;
phi_smd = 0.2;
Kf = atanh(tan(phi_smd))/delta_raio;

U_0 = Veq;          % Veloc. inicial do VANT (m/s).


amp_degrau_h_vt_ext = 0;
amp_degrau_psi = 0;

%Define the parameters for the vector field controler
vector_field_parameters


theta_0 = wrapToPi(theta_0);
if (theta_0 == -pi)
    theta_0 = pi;
end


x0_refmodel = [ Veq;            % v     -> norma do vetor velocidade no plano XY (m/s)
                theta_0;        % theta -> proa inicial (rad).
                posx_0;         % x     -> posicao leste.
                posy_0;         % y     -> posicao norte.
                ];

% Valores iniciais do problema
P_N_0 = posy_0;        % Posicao Norte inicial (m).
P_E_0 = posx_0;        % Posicao Leste inicial (m).

% Ang. de Guinada inicial (rad), referencial NED.
psi_0 = mod((pi/2 - theta_0) + pi, 2*pi) - pi;



% Condicoes iniciais para o VANT
x0_uav = [  P_N_0;          % P_N (m)  
            P_E_0;          % P_E (m) 
            H_0;            % altitude (m)
            0;              % phi    (rad)
            0;              % theta  (rad) 
            psi_0;          % psi    (rad)
            Veq;            % U  (m/s)
            0;              % V  (m/s)
            0;              % W  (m/s)
            0;              % P  (rad/s)
            0;              % Q  (rad/s)
            0   ];          % R  (rad/s)


%u0_uav = [ 0 0 0 (2*motor_min + (1 - motor_min))/2]';
u0_uav = [0 0 0 0.4];




rho = 0.1;
perc_int_max = 0.8;


centro_curva = [0 0];
vd = Veq;


%% Parametros controlador sintonizado Thums
KC_vt = 0.9948*1;
TI_vt = 0.2128*1;
KC_theta = -1.1482;
TI_theta = 4.7607;
TD_theta = 0.0640;
KC_h = 0.28;
TI_h = 5.7;
TD_h = 0.14;
KC_phi = -1.18;
TI_phi = 5.85;
TD_phi = 0.0646;

KC_psi = 0.03; 
TD_psi = 0.24;
% Integral action introduced after all the other gains
% were obtained through optimization.
% The objective is to have zero error in the tracking
% of ramp like reference signals (necessary to circulate
% a circle).
TI_psi = 0.8*1e8;

KR = 0.0130;
KE = -0.1562;
KT = 0.0792;
KC_r = -0.0073;
TI_r = 3.4131;
TD_r = 0.1076;
