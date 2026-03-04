%% Simscape_LQR_v41.m
%  TESTBENCH COMPLETO — Segway Gemelo Digital | 6 Estados
%  ─ Reconstruye Segway_Testbench_v41.slx desde cero en cada ejecucion
%  ─ K1 (1x4): LQR avance  [theta, dtheta, x, dx]
%  ─ K2 (1x2): LQR giro    [alpha, dalpha]
%  ─ Controlador compacto: un bloque MATLAB Function (LQR_Controller)
%  ─ Mezcla: VR = Va+Vd  |  VL = Va-Vd  (Vd>0 = giro derecha)
%  ─ Saturaciones: Va +-24V | Vd +-24V | VR/VL +-24V final
%  ─ Sin back-EMF — consistencia planta<->controlador
%  ─ MATLAB/Simulink R2025b
% =========================================================================
clear; clc; close all; bdclose all;

%% =========================================================================
%  1. PARAMETROS — MODIFICA AQUI TUS VALORES REALES
% =========================================================================
M      = 80;      % masa cuerpo [kg]
r      = 0.20;    % radio rueda [m]
d      = 0.60;    % separacion entre ruedas [m]
l      = 0.90;    % dist. eje ruedas -> CM cuerpo [m]
g      = 9.81;
m      = 2;       % masa cada rueda [kg]
Icy    = 10;      % inercia cuerpo pitch [kg·m^2]
Icz    = 12;      % inercia cuerpo yaw   [kg·m^2]
Icx    = 12;      % inercia cuerpo roll  [kg·m^2]
Iw     = 0.08;    % inercia rueda spin   [kg·m^2]
Iwz    = 0.04;    % inercia rueda transversal [kg·m^2]
alm    = 2.0;     % ganancia par motor [N·m/V]

% -- Geometria visual --
body_W = 0.40;
body_D = 0.20;
body_H = 1.60;

% -- Condiciones iniciales --
theta0_deg = 5;      % perturbacion balanceo [deg]
alpha0_deg = 10;     % perturbacion giro     [deg]  <- valida K2
theta0     = theta0_deg * pi/180;
alpha0     = alpha0_deg * pi/180;

% -- Simulacion --
t_sim   = 15;    % duracion [s]
V_sat_a = 24;    % saturacion Va (avance)  [V]
V_sat_d = 24;    % saturacion Vd (giro)    [V]
V_sat_f = 24;    % saturacion final VR,VL  [V]

% -- RUTINA AVANCE: theta_ref --
theta_ref_deg = 0;
t_avance      = 999;
t_stop        = 1000;

% -- RUTINA GIRO: alpha_ref --
alpha_ref_deg = 0;
t_avance_a    = 999;
t_stop_a      = 1000;

fprintf('=========================================================\n');
fprintf(' TESTBENCH Segway v4.1 | 6 Estados | K1+K2 desacoplados\n');
fprintf(' theta0=%.1f  alpha0=%.1f  t_sim=%gs\n', theta0_deg, alpha0_deg, t_sim);
fprintf('=========================================================\n');

if abs(alpha0_deg) < 5
    fprintf('[ADVERTENCIA] alpha0=%.1f deg es pequeno — Vd inicial sera %.2f V.\n', ...
        alpha0_deg, abs(alpha0_deg)*pi/180 * 28.28);
    fprintf('              El giro puede ser imperceptible visualmente.\n');
    fprintf('              Usa alpha0_deg >= 5 para validar K2 visualmente.\n');
end

%% =========================================================================
%  2. LINEALIZACION Y LQR
%  K1: avance  — [theta, dtheta, x, dx]   entrada Va
%  K2: giro    — [alpha, dalpha]           entrada Vd
%  Sin back-EMF: tau = alm*V
% =========================================================================
M11  = Icy + M*l^2;
M12  = M*l;
M22  = M + 2*m + 2*Iw/r^2;
M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
det0 = M11*M22 - M12^2;

% ── K1: subsistema avance ─────────────────────────────────────────────
A1 = zeros(4);
A1(1,2) = 1;
A1(2,1) = M22*M*g*l / det0;
A1(3,4) = 1;
A1(4,1) = -M12*M*g*l / det0;

b21_a = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41_a = ( M12*(2*alm) + M11*(2*alm/r)) / det0;
B1    = [0; b21_a; 0; b41_a];

rango1 = rank(ctrb(A1, B1));
fprintf('[K1] Controlabilidad: %d/4', rango1);
if rango1==4, fprintf(' OK\n'); else, fprintf(' REVISAR\n'); end

Q1 = diag([2000, 100, 50, 50]);
R1 = 1;
[K1, ~, eigs1] = lqr(A1, B1, Q1, R1);
fprintf('[K1] K1 = [%.3f  %.3f  %.3f  %.3f]\n', K1);
fprintf('[K1] Polos: '); fprintf('%.3f  ', real(eigs1)); fprintf('\n');

% ── K2: subsistema giro ───────────────────────────────────────────────
A2 = [0 1; 0 0];
B2 = [0; d*alm/(r*M33)];

rango2 = rank(ctrb(A2, B2));
fprintf('[K2] Controlabilidad: %d/2', rango2);
if rango2==2, fprintf(' OK\n'); else, fprintf(' REVISAR\n'); end

Q2 = diag([800, 50]);
R2 = 1;
[K2, ~, eigs2] = lqr(A2, B2, Q2, R2);
fprintf('[K2] K2 = [%.3f  %.3f]\n', K2);
fprintf('[K2] Polos: '); fprintf('%.3f  ', real(eigs2)); fprintf('\n');

%% =========================================================================
%  3. ODE NO LINEAL 6 ESTADOS + K1/K2
%  X = [theta, dtheta, x, dx, alpha, dalpha]
% =========================================================================
theta_ref_rad = theta_ref_deg * pi/180;
alpha_ref_rad = alpha_ref_deg * pi/180;
X0_ode = [theta0; 0; 0; 0; alpha0; 0];

ode_fn = @(t,X) ode_nl_6(t, X, K1, K2, ...
    V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    theta_ref_rad, alpha_ref_rad, ...
    t_avance, t_stop, t_avance_a, t_stop_a);

[t_ode, X_ode] = ode45(ode_fn, [0 t_sim], X0_ode, odeset('RelTol',1e-6));

Va_ode = zeros(length(t_ode),1);
Vd_ode = zeros(length(t_ode),1);
VR_ode = zeros(length(t_ode),1);
VL_ode = zeros(length(t_ode),1);
for i = 1:length(t_ode)
    tref_th = get_ref(t_ode(i), theta_ref_rad, t_avance,   t_stop);
    tref_al = get_ref(t_ode(i), alpha_ref_rad, t_avance_a, t_stop_a);
    Xe1 = [X_ode(i,1)-tref_th; X_ode(i,2); X_ode(i,3); X_ode(i,4)];
    Xe2 = [X_ode(i,5)-tref_al; X_ode(i,6)];
    Va_ode(i) = max(min(-K1*Xe1, V_sat_a), -V_sat_a);
    Vd_ode(i) = max(min(-K2*Xe2, V_sat_d), -V_sat_d);
    VR_ode(i) = max(min(Va_ode(i)+Vd_ode(i), V_sat_f), -V_sat_f);
    VL_ode(i) = max(min(Va_ode(i)-Vd_ode(i), V_sat_f), -V_sat_f);
end

fprintf('[ODE] theta_final=%.3f  x_final=%.3f m  alpha_final=%.3f\n', ...
    rad2deg(X_ode(end,1)), X_ode(end,3), rad2deg(X_ode(end,5)));
fprintf('[ODE] theta %s  |  alpha %s\n', ...
    iif(abs(rad2deg(X_ode(end,1)))<1,'OK','REVISAR'), ...
    iif(abs(rad2deg(X_ode(end,5)))<1,'OK','REVISAR'));

%% =========================================================================
%  4. CONSTRUIR MODELO SIMSCAPE — siempre desde cero
% =========================================================================
modelName = 'Segway_Testbench_v41';
if bdIsLoaded(modelName),            close_system(modelName, 0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName);
open_system(modelName);
set_param(modelName,'Solver','ode23t','StopTime',num2str(t_sim),...
    'RelTol','1e-4','AbsTol','1e-6');

add_block('nesl_utility/Solver Configuration',        [modelName '/Solver_Config'],'Position',[42 105 192 135]);
add_block('sm_lib/Utilities/Mechanism Configuration', [modelName '/Mech_Config'],  'Position',[227 130 417 160]);
set_param([modelName '/Mech_Config'],'GravityVector','[0; 0; -9.81]');
add_block('sm_lib/Frames and Transforms/World Frame', [modelName '/World'],         'Position',[92 40 192 70]);
add_line(modelName,'World/RConn1','Solver_Config/RConn1');
add_line(modelName,'World/RConn1','Mech_Config/RConn1');

% Cadena cinematica
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[477 130 597 160]);
set_param([modelName '/RT_AxleHeight'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Z','TranslationStandardOffset',num2str(r));

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_PrismaticAlign'],'Position',[647 130 767 160]);
set_param([modelName '/RT_PrismaticAlign'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','90','TranslationMethod','None');

add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[817 130 937 190]);
set_param([modelName '/Joint_x'],'MotionActuationMode','ComputedMotion','TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign'],'Position',[997 120 1117 160]);
set_param([modelName '/RT_Realign'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','-90','TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[1167 125 1287 185]);
set_param([modelName '/Joint_alpha'],...
    'TorqueActuationMode','InputTorque',...
    'MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0',...
    'SensePosition','on','SenseVelocity','on',...
    'PositionTargetSpecify','on',...
    'PositionTargetValue',num2str(alpha0_deg),...
    'PositionTargetValueUnits','deg',...
    'PositionTargetPriority','High');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[1377 240 1497 270]);
set_param([modelName '/RT_ThetaAxis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90','TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[1547 225 1667 285]);
set_param([modelName '/Joint_theta'],...
    'PositionTargetSpecify','on','PositionTargetValue',num2str(theta0_deg),...
    'PositionTargetValueUnits','deg','PositionTargetPriority','High',...
    'TorqueActuationMode','NoTorque','MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1727 205 1847 265]);
set_param([modelName '/RT_BodyCM'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(l));

add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1935 -50 2055 10]);
set_param([modelName '/Body_Solid'],'BrickDimensions',mat2str([body_D,body_H,body_W]),'InertiaType','Custom','Mass',num2str(M),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Icx,Icy,Icz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.2 0.5 0.8]','Orientation','left');

add_line(modelName,'World/RConn1',            'RT_AxleHeight/LConn1');
add_line(modelName,'RT_AxleHeight/RConn1',    'RT_PrismaticAlign/LConn1');
add_line(modelName,'RT_PrismaticAlign/RConn1','Joint_x/LConn1');
add_line(modelName,'Joint_x/RConn1',          'RT_Realign/LConn1');
add_line(modelName,'RT_Realign/RConn1',        'Joint_alpha/LConn1');
add_line(modelName,'Joint_alpha/RConn1',       'RT_ThetaAxis/LConn1');
add_line(modelName,'RT_ThetaAxis/RConn1',      'Joint_theta/LConn1');
add_line(modelName,'Joint_theta/RConn1',       'RT_BodyCM/LConn1');
add_line(modelName,'RT_BodyCM/RConn1',         'Body_Solid/RConn1');

% Rueda derecha
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[1377 120 1497 150]);
set_param([modelName '/RT_WheelR'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Axis'],'Position',[1952 120 2072 150]);
set_param([modelName '/RT_WheelR_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[2545 245 2665 305]);
set_param([modelName '/Joint_phi_R'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[2600 550 2720 610]);
set_param([modelName '/Wheel_R'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]','Orientation','left');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelR/LConn1');
add_line(modelName,'RT_WheelR/RConn1',      'RT_WheelR_Axis/LConn1');
add_line(modelName,'RT_WheelR_Axis/RConn1', 'Joint_phi_R/LConn1');
add_line(modelName,'Joint_phi_R/RConn1',    'Wheel_R/RConn1');

% Rueda izquierda
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL'],'Position',[1377 180 1497 210]);
set_param([modelName '/RT_WheelL'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL_Axis'],'Position',[1952 180 2072 210]);
set_param([modelName '/RT_WheelL_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_L'],'Position',[2545 360 2665 420]);
set_param([modelName '/Joint_phi_L'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_L'],'Position',[2600 110 2720 170]);
set_param([modelName '/Wheel_L'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]','Orientation','left');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelL/LConn1');
add_line(modelName,'RT_WheelL/RConn1',      'RT_WheelL_Axis/LConn1');
add_line(modelName,'RT_WheelL_Axis/RConn1', 'Joint_phi_L/LConn1');
add_line(modelName,'Joint_phi_L/RConn1',    'Wheel_L/RConn1');

% Suelo visual
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Ground'],'Position',[477 190 597 220]);
set_param([modelName '/RT_Ground'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Z','TranslationStandardOffset','0.025');
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Ground'],'Position',[297 230 417 260]);
set_param([modelName '/Ground'],'BrickDimensions','[10 10 0.05]','InertiaType','Custom','Mass','1e6','CenterOfMass','[0 0 0]','MomentsOfInertia','[1e6 1e6 1e6]','ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.6 0.6 0.6]');
add_line(modelName,'World/RConn1','RT_Ground/LConn1');
add_line(modelName,'RT_Ground/RConn1','Ground/RConn1');

fprintf('[Build] Estructura fisica OK\n');

%% =========================================================================
%  5. UPDATE → verificar port handles
% =========================================================================
set_param(modelName,'SimulationCommand','update');
pH_Jt  = get_param([modelName '/Joint_theta'], 'PortHandles');
pH_Jx  = get_param([modelName '/Joint_x'],     'PortHandles');
pH_Ja  = get_param([modelName '/Joint_alpha'],  'PortHandles');
pH_JpR = get_param([modelName '/Joint_phi_R'],  'PortHandles');

fprintf('[Ports] Joint_theta=%d | Joint_x=%d | Joint_alpha=%d | Joint_phi_R=%d\n',...
    numel(pH_Jt.RConn), numel(pH_Jx.RConn), numel(pH_Ja.RConn), numel(pH_JpR.RConn));

if numel(pH_Ja.RConn) < 3
    error('[ERROR] Joint_alpha RConn=%d — SensePosition/SenseVelocity no activos', numel(pH_Ja.RConn));
end
fprintf('[Ports] Joint_alpha RConn(2)=alpha  RConn(3)=dalpha  OK\n');

%% =========================================================================
%  6. REFERENCIAS — theta_ref y alpha_ref como Steps
% =========================================================================
tref_str = num2str(theta_ref_deg*pi/180,'%.6f');
add_block('simulink/Sources/Step',[modelName '/Step_th_on'], 'Position',[1637 715 1707 745]);
set_param([modelName '/Step_th_on'],'Time',num2str(t_avance),'Before','0','After',tref_str);
add_block('simulink/Sources/Step',[modelName '/Step_th_off'],'Position',[1637 775 1707 805]);
set_param([modelName '/Step_th_off'],'Time',num2str(t_stop),'Before','0','After',['-' tref_str]);
add_block('simulink/Math Operations/Sum',[modelName '/Sum_tref'],'Position',[1795 745 1835 805]);
set_param([modelName '/Sum_tref'],'Inputs','++','IconShape','rectangular');
pH_Stref = get_param([modelName '/Sum_tref'],'PortHandles');
add_line(modelName,get_param([modelName '/Step_th_on'], 'PortHandles').Outport(1), pH_Stref.Inport(1),'autorouting','on');
add_line(modelName,get_param([modelName '/Step_th_off'],'PortHandles').Outport(1), pH_Stref.Inport(2),'autorouting','on');

aref_str = num2str(alpha_ref_deg*pi/180,'%.6f');
add_block('simulink/Sources/Step',[modelName '/Step_al_on'], 'Position',[1637 835 1707 865]);
set_param([modelName '/Step_al_on'],'Time',num2str(t_avance_a),'Before','0','After',aref_str);
add_block('simulink/Sources/Step',[modelName '/Step_al_off'],'Position',[1637 895 1707 925]);
set_param([modelName '/Step_al_off'],'Time',num2str(t_stop_a),'Before','0','After',['-' aref_str]);
add_block('simulink/Math Operations/Sum',[modelName '/Sum_aref'],'Position',[1795 865 1835 925]);
set_param([modelName '/Sum_aref'],'Inputs','++','IconShape','rectangular');
pH_Saref = get_param([modelName '/Sum_aref'],'PortHandles');
add_line(modelName,get_param([modelName '/Step_al_on'], 'PortHandles').Outport(1), pH_Saref.Inport(1),'autorouting','on');
add_line(modelName,get_param([modelName '/Step_al_off'],'PortHandles').Outport(1), pH_Saref.Inport(2),'autorouting','on');

fprintf('[Refs] theta_ref=%.1f @ t=%.0f  |  alpha_ref=%.1f @ t=%.0f\n',...
    theta_ref_deg, t_avance, alpha_ref_deg, t_avance_a);

%% =========================================================================
%  7. CONTROLADOR LQR — MATLAB Function compacto
%  Entradas (8): theta, dtheta, x, dx, alpha, dalpha, theta_ref, alpha_ref
%  Salidas  (4): VR, VL, Va, Vd
%  K1/K2 hardcodeados con los valores calculados en seccion 2
% =========================================================================
ps_names = {'PS2SL_th','PS2SL_dth','PS2SL_x','PS2SL_dx','PS2SL_al','PS2SL_dal'};
ps_src   = {pH_Jt.RConn(2), pH_Jt.RConn(3), pH_Jx.RConn(2), pH_Jx.RConn(3), pH_Ja.RConn(2), pH_Ja.RConn(3)};
ps_pos = {[1767 400 1877 430],[1767 465 1877 495],[1767 530 1877 560],[1767 595 1877 625],[1767 660 1877 690],[1767 725 1877 755]};
for i = 1:6
    add_block('nesl_utility/PS-Simulink Converter',[modelName '/' ps_names{i}],'Position',ps_pos{i});
    add_line(modelName, ps_src{i}, get_param([modelName '/' ps_names{i}],'PortHandles').LConn(1),'autorouting','on');
end

add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/LQR_Controller'],'Position',[1960 303 2105 812]);

func_str = sprintf([...
'function [VR, VL, Va, Vd] = LQR_Controller(theta, dtheta, x, dx, alpha, dalpha, theta_ref, alpha_ref)\n'...
'%% LQR K1+K2 desacoplados — ganancias hardcodeadas\n'...
'%%  K1: avance [theta,dtheta,x,dx]  K2: giro [alpha,dalpha]\n'...
'%%  Salidas: VR, VL (a motores)  Va, Vd (para F_x y tau_alpha)\n'...
'K1 = %s;\n'...
'K2 = %s;\n'...
'V_sat_a = %.4f;\n'...
'V_sat_d = %.4f;\n'...
'V_sat_f = %.4f;\n'...
'\n'...
'Xe1 = [theta - theta_ref; dtheta; x; dx];\n'...
'Va  = max(min(-K1 * Xe1, V_sat_a), -V_sat_a);\n'...
'\n'...
'Xe2 = [alpha - alpha_ref; dalpha];\n'...
'Vd  = max(min(-K2 * Xe2, V_sat_d), -V_sat_d);\n'...
'\n'...
'VR = max(min(Va + Vd, V_sat_f), -V_sat_f);\n'...
'VL = max(min(Va - Vd, V_sat_f), -V_sat_f);\n'...
'end\n'], mat2str(K1,6), mat2str(K2,6), V_sat_a, V_sat_d, V_sat_f);

rt = sfroot();
ctrl_chart = rt.find('-isa','Stateflow.EMChart','Path',[modelName '/LQR_Controller']);
ctrl_chart.Script = func_str;

pH_LQR = get_param([modelName '/LQR_Controller'],'PortHandles');
for i = 1:6
    add_line(modelName, get_param([modelName '/' ps_names{i}],'PortHandles').Outport(1), pH_LQR.Inport(i),'autorouting','on');
end
add_line(modelName, pH_Stref.Outport(1), pH_LQR.Inport(7),'autorouting','on');
add_line(modelName, pH_Saref.Outport(1), pH_LQR.Inport(8),'autorouting','on');

fprintf('[LQR_Controller] MATLAB Function OK — K1/K2 hardcodeados\n');
fprintf('[LQR_Controller] Salidas: port1=VR  port2=VL  port3=Va  port4=Vd\n');

%% =========================================================================
%  8. TORQUES A JOINTS
%  port1=VR  port2=VL  port3=Va  port4=Vd
% =========================================================================
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauR'],'Position',[2225 405 2255 435]);
set_param([modelName '/Gain_tauR'],'Gain',num2str(alm));
pH_GtR = get_param([modelName '/Gain_tauR'],'PortHandles');
add_line(modelName, pH_LQR.Outport(1), pH_GtR.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauR'],'Position',[2318 405 2398 435]);
add_line(modelName, pH_GtR.Outport(1), get_param([modelName '/SL2PS_tauR'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauR/RConn1','Joint_phi_R/LConn2');

add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauL'],'Position',[2225 470 2255 500]);
set_param([modelName '/Gain_tauL'],'Gain',num2str(alm));
pH_GtL = get_param([modelName '/Gain_tauL'],'PortHandles');
add_line(modelName, pH_LQR.Outport(2), pH_GtL.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauL'],'Position',[2318 470 2398 500]);
add_line(modelName, pH_GtL.Outport(1), get_param([modelName '/SL2PS_tauL'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauL/RConn1','Joint_phi_L/LConn2');

add_block('simulink/Math Operations/Gain',[modelName '/Gain_Fx'],'Position',[2225 590 2255 620]);
set_param([modelName '/Gain_Fx'],'Gain',num2str(2*alm/r));
pH_GFx = get_param([modelName '/Gain_Fx'],'PortHandles');
add_line(modelName, pH_LQR.Outport(3), pH_GFx.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fx'],'Position',[687 190 767 220]);
add_line(modelName, pH_GFx.Outport(1), get_param([modelName '/SL2PS_Fx'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_Fx/RConn1','Joint_x/LConn2');

add_block('simulink/Math Operations/Gain',[modelName '/Gain_tau_al'],'Position',[2225 530 2255 560]);
set_param([modelName '/Gain_tau_al'],'Gain',num2str(d*alm/r));
pH_Gtal = get_param([modelName '/Gain_tau_al'],'PortHandles');
add_line(modelName, pH_LQR.Outport(4), pH_Gtal.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tau_al'],'Position',[1170 280 1250 310]);
add_line(modelName, pH_Gtal.Outport(1), get_param([modelName '/SL2PS_tau_al'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tau_al/RConn1','Joint_alpha/LConn2');

fprintf('[Torques] tau_R | tau_L | F_x | tau_alpha -> joints OK\n');

%% =========================================================================
%  9. LOGGING
% =========================================================================
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VRVL'],'Position',[2040 702 2045 773]);
set_param([modelName '/Mux_VRVL'],'Inputs','2');
pH_MVRVL = get_param([modelName '/Mux_VRVL'],'PortHandles');
add_line(modelName, pH_LQR.Outport(1), pH_MVRVL.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(2), pH_MVRVL.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VRVL'],'Position',[1975 790 2035 810]);
set_param([modelName '/ToWS_VRVL'],'VariableName','VRVL_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVRVL.Outport(1), get_param([modelName '/ToWS_VRVL'],'PortHandles').Inport(1),'autorouting','on');

add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VaVd'],'Position',[2040 612 2045 683]);
set_param([modelName '/Mux_VaVd'],'Inputs','2');
pH_MVaVd = get_param([modelName '/Mux_VaVd'],'PortHandles');
add_line(modelName, pH_LQR.Outport(3), pH_MVaVd.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(4), pH_MVaVd.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VaVd'],'Position',[1940 615 2000 635]);
set_param([modelName '/ToWS_VaVd'],'VariableName','VaVd_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVaVd.Outport(1), get_param([modelName '/ToWS_VaVd'],'PortHandles').Inport(1),'autorouting','on');

fprintf('[Logging] VRVL_log | VaVd_log OK\n');

%% =========================================================================
%  10. AUTO-LAYOUT — reordena bloques y elimina cables diagonales
% =========================================================================
Simulink.BlockDiagram.arrangeSystem(modelName);
fprintf('[Layout] Bloques reorganizados automaticamente\n');

%% =========================================================================
%  12. GUARDAR Y SIMULAR
% =========================================================================
save_system(modelName,[modelName '.slx']);
fprintf('[Sim] Iniciando simulacion de %g s...\n', t_sim);
try
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimscapeLogType','all','SimscapeLogName','simlog');
    out   = sim(simIn);
    fprintf('[Sim] OK  t_final=%.3f s\n', out.tout(end));
catch e
    fprintf('[Sim] FALLO: %s\n', e.message);
    return
end

%% =========================================================================
%  13. DIAGNOSTICO + GRAFICAS
% =========================================================================
try
    t_sm     = out.simlog.Joint_theta.Rz.q.series.time;
    theta_sm = out.simlog.Joint_theta.Rz.q.series.values('rad');
    alpha_sm = out.simlog.Joint_alpha.Rz.q.series.values('rad');

    fprintf('\n[Diagnostico Simscape]\n');
    fprintf('  theta(0) = %.4f rad (esperado %.4f)  %s\n', theta_sm(1), theta0, iif(abs(theta_sm(1)-theta0)<0.01,'OK','REVISAR'));
    fprintf('  alpha(0) = %.4f rad (esperado %.4f)  %s\n', alpha_sm(1), alpha0, iif(abs(alpha_sm(1)-alpha0)<0.01,'OK','REVISAR'));
    if abs(alpha_sm(1)) < 0.01 && abs(alpha0) > 0.01
        fprintf('  [ADVERTENCIA] alpha(0) Simscape ~ 0 pero alpha0=%.2f rad.\n', alpha0);
        fprintf('                Verificar PositionTargetSpecify en Joint_alpha.\n');
    end
    fprintf('  theta_final = %.3f  %s\n', rad2deg(theta_sm(end)), iif(abs(rad2deg(theta_sm(end)))<1,'OK','REVISAR'));
    fprintf('  alpha_final = %.3f  %s\n', rad2deg(alpha_sm(end)), iif(abs(rad2deg(alpha_sm(end)))<1,'OK','REVISAR'));

    t_U  = out.tout;
    VRVL = out.get('VRVL_log');
    VaVd = out.get('VaVd_log');

    figure('Name','v4.1 — Resultados','Position',[50 50 1200 800]);

    % Subplot 1: theta
    subplot(3,1,1); hold on; grid on;
    plot(t_ode, rad2deg(X_ode(:,1)),'r--','LineWidth',2,'DisplayName','ODE + K1/K2');
    plot(t_sm,  rad2deg(theta_sm),  'b',  'LineWidth',2,'DisplayName','Simscape + K1/K2');
    yline(0,'--k','Equilibrio');
    xlabel('t [s]'); ylabel('\theta [deg]');
    title(sprintf('\\theta(t)  |  \\theta_0=%.0f°  \\alpha_0=%.0f°', theta0_deg, alpha0_deg));
    legend('Location','best'); xlim([0 t_sim]);

    % Subplot 2: VR y VL
    subplot(3,1,2); hold on; grid on;
    if ~isempty(VRVL) && size(VRVL,2)>=2
        plot(t_U, VRVL(:,1),'b',  'LineWidth',1.5,'DisplayName','V_R Simscape');
        plot(t_U, VRVL(:,2),'b--','LineWidth',1.5,'DisplayName','V_L Simscape');
    end
    plot(t_ode, VR_ode,'r',  'LineWidth',1.2,'DisplayName','V_R ODE');
    plot(t_ode, VL_ode,'r--','LineWidth',1.2,'DisplayName','V_L ODE');
    yline( V_sat_f,'--k'); yline(-V_sat_f,'--k');
    xlabel('t [s]'); ylabel('V [V]');
    title('VR y VL  |  VR~=VL mientras K2 corrige alpha');
    legend('Location','best'); xlim([0 t_sim]);

    % Subplot 3: Va y Vd
    subplot(3,1,3); hold on; grid on;
    plot(t_ode, Va_ode,'g',  'LineWidth',2,'DisplayName','Va (avance) ODE');
    plot(t_ode, Vd_ode,'m--','LineWidth',2,'DisplayName','Vd (giro)   ODE');
    if ~isempty(VaVd) && size(VaVd,2)>=2
        plot(t_U, VaVd(:,1),'g:','LineWidth',1.2,'DisplayName','Va Simscape');
        plot(t_U, VaVd(:,2),'m:','LineWidth',1.2,'DisplayName','Vd Simscape');
    end
    yline(0,'--k');
    xlabel('t [s]'); ylabel('V [V]');
    title('Desacoplamiento K1/K2  |  Va=avance  Vd=giro');
    legend('Location','best'); xlim([0 t_sim]);

catch e
    fprintf('[Graficas] FALLO: %s\n', e.message);
    disp(fieldnames(out));
end

fprintf('\n=========================================================\n');
fprintf(' LISTO v4.1\n');
fprintf(' Modificar perturbacion: theta0_deg | alpha0_deg\n');
fprintf(' Activar avance:  theta_ref_deg | t_avance | t_stop\n');
fprintf(' Activar giro:    alpha_ref_deg | t_avance_a | t_stop_a\n');
fprintf('=========================================================\n');

%% =========================================================================
%  FUNCIONES LOCALES
% =========================================================================
function s = iif(cond, a, b)
    if cond, s = a; else, s = b; end
end

function ref = get_ref(t, val, t_on, t_off)
    if t >= t_on && t < t_off
        ref = val;
    else
        ref = 0;
    end
end

function dX = ode_nl_6(t, X, K1, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    theta_ref_rad, alpha_ref_rad, t_avance, t_stop, t_avance_a, t_stop_a)

    tref_th = get_ref(t, theta_ref_rad, t_avance,   t_stop);
    tref_al = get_ref(t, alpha_ref_rad, t_avance_a, t_stop_a);

    Xe1 = [X(1)-tref_th; X(2); X(3); X(4)];
    Va  = max(min(-K1*Xe1, V_sat_a), -V_sat_a);

    Xe2 = [X(5)-tref_al; X(6)];
    Vd  = max(min(-K2*Xe2, V_sat_d), -V_sat_d);

    VR = max(min(Va+Vd, V_sat_f), -V_sat_f);
    VL = max(min(Va-Vd, V_sat_f), -V_sat_f);

    tau_R = alm*VR;
    tau_L = alm*VL;

    theta = X(1); dtheta = X(2);
    M11  = Icy + M*l^2;
    M12  = M*l*cos(theta);
    M22  = M + 2*m + 2*Iw/r^2;
    M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    detM = M11*M22 - M12^2;

    F1 = M*g*l*sin(theta) - (tau_R + tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R + tau_L)/r;
    F3 = d*(tau_R - tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / detM;
    ddx     = (-M12*F1 + M11*F2) / detM;
    ddalpha = F3 / M33;

    dX = [dtheta; ddtheta; X(4); ddx; X(6); ddalpha];
end
