%% Simscape_LQR_v4.m
%  TESTBENCH — Segway Gemelo Digital | 6 Estados
%  ─ K1 (1x4): LQR avance  [theta, dtheta, x, dx]
%  ─ K2 (1x2): LQR giro    [alpha, dalpha]
%  ─ Mezcla: VR = Va+Vd  |  VL = Va-Vd  (Vd>0 = giro derecha)
%  ─ Saturaciones: Va +-24V | Vd +-24V | VR/VL +-24V final
%  ─ F_x       = alm*2*Va/r      → Joint_x
%  ─ tau_alpha = d*alm*Vd/r      → Joint_alpha  [N·m] check: [m][N·m/V][V]/[m]=[N·m]
%  ─ Sin back-EMF — consistencia planta↔controlador
%  ─ CI: theta0=5°, alpha0=2°  → valida K1 y K2 simultaneamente
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
alpha0_deg = 10;      % perturbacion giro     [deg]  <- valida K2
theta0     = theta0_deg * pi/180;
alpha0     = alpha0_deg * pi/180;

% -- Simulacion --
t_sim   = 15;    % duracion [s]
V_sat_a = 24;    % saturacion Va (avance)  [V]
V_sat_d = 24;    % saturacion Vd (giro)    [V]
V_sat_f = 24;    % saturacion final VR,VL  [V]

% -- RUTINA AVANCE: theta_ref --
% Para activar avance: cambia theta_ref_deg y t_avance/t_stop
theta_ref_deg = 0;
t_avance      = 999;
t_stop        = 1000;

% -- RUTINA GIRO: alpha_ref --
% Para activar giro: cambia alpha_ref_deg y t_avance_a/t_stop_a
alpha_ref_deg = 0;
t_avance_a    = 999;
t_stop_a      = 1000;

fprintf('=========================================================\n');
fprintf(' TESTBENCH Segway v4 | 6 Estados | K1+K2 desacoplados\n');
fprintf(' theta0=%.1f  alpha0=%.1f  t_sim=%gs\n', theta0_deg, alpha0_deg, t_sim);
fprintf('=========================================================\n');

% Advertencia: alpha0 pequeno -> Vd imperceptible en animacion 3D
if abs(alpha0_deg) < 5
    fprintf('[ADVERTENCIA] alpha0=%.1f deg es pequeno — Vd inicial sera %.2f V.\n', ...
        alpha0_deg, abs(alpha0_deg)*pi/180 * 28.28);  % estimacion K2*alpha0
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
% Va actua simetricamente: tau_R=tau_L=alm*Va → tau_sum=2*alm*Va
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
% Vd diferencial: tau_R-tau_L=2*alm*Vd → F3=d*alm*Vd/r → ddalpha=F3/M33
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

% Reconstruir Va, Vd, VR, VL para graficas
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
%  4. CONSTRUIR MODELO SIMSCAPE
% =========================================================================
modelName = 'Segway_Testbench_v4';
if bdIsLoaded(modelName),            close_system(modelName, 0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName);
open_system(modelName);
set_param(modelName,'Solver','ode23t','StopTime',num2str(t_sim),...
    'RelTol','1e-4','AbsTol','1e-6');

add_block('nesl_utility/Solver Configuration',        [modelName '/Solver_Config'],'Position',[30 30 180 60]);
add_block('sm_lib/Utilities/Mechanism Configuration', [modelName '/Mech_Config'],  'Position',[30 90 220 120]);
set_param([modelName '/Mech_Config'],'GravityVector','[0; 0; -9.81]');
add_block('sm_lib/Frames and Transforms/World Frame', [modelName '/World'],         'Position',[30 160 130 190]);
add_line(modelName,'World/RConn1','Solver_Config/RConn1');
add_line(modelName,'World/RConn1','Mech_Config/RConn1');

% Cadena cinematica
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[220 160 340 190]);
set_param([modelName '/RT_AxleHeight'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Z','TranslationStandardOffset',num2str(r));

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_PrismaticAlign'],'Position',[400 160 520 190]);
set_param([modelName '/RT_PrismaticAlign'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','90','TranslationMethod','None');

add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[580 145 700 205]);
set_param([modelName '/Joint_x'],'MotionActuationMode','ComputedMotion','TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign'],'Position',[740 145 860 185]);
set_param([modelName '/RT_Realign'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','-90','TranslationMethod','None');

% Joint_alpha: ahora InputTorque + Sensing + CI alpha0
% DampingCoefficient=0 — K2 controla activamente, sin parches
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[920 145 1040 205]);
set_param([modelName '/Joint_alpha'],...
    'TorqueActuationMode','InputTorque',...
    'MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0',...
    'SensePosition','on','SenseVelocity','on',...
    'PositionTargetSpecify','on',...
    'PositionTargetValue',num2str(alpha0_deg),...
    'PositionTargetValueUnits','deg',...
    'PositionTargetPriority','High');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[960 60 1080 90]);
set_param([modelName '/RT_ThetaAxis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90','TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[1140 45 1260 105]);
set_param([modelName '/Joint_theta'],...
    'PositionTargetSpecify','on','PositionTargetValue',num2str(theta0_deg),...
    'PositionTargetValueUnits','deg','PositionTargetPriority','High',...
    'TorqueActuationMode','NoTorque','MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1320 45 1440 105]);
set_param([modelName '/RT_BodyCM'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(l));

add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1500 45 1620 105]);
set_param([modelName '/Body_Solid'],'BrickDimensions',mat2str([body_D,body_H,body_W]),'InertiaType','Custom','Mass',num2str(M),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Icx,Icy,Icz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.2 0.5 0.8]');

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
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[960 160 1080 190]);
set_param([modelName '/RT_WheelR'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Axis'],'Position',[1140 160 1260 190]);
set_param([modelName '/RT_WheelR_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[1320 145 1440 205]);
set_param([modelName '/Joint_phi_R'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[1500 145 1620 205]);
set_param([modelName '/Wheel_R'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelR/LConn1');
add_line(modelName,'RT_WheelR/RConn1',      'RT_WheelR_Axis/LConn1');
add_line(modelName,'RT_WheelR_Axis/RConn1', 'Joint_phi_R/LConn1');
add_line(modelName,'Joint_phi_R/RConn1',    'Wheel_R/RConn1');

% Rueda izquierda
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL'],'Position',[960 250 1080 280]);
set_param([modelName '/RT_WheelL'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL_Axis'],'Position',[1140 250 1260 280]);
set_param([modelName '/RT_WheelL_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_L'],'Position',[1320 235 1440 295]);
set_param([modelName '/Joint_phi_L'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_L'],'Position',[1500 235 1620 295]);
set_param([modelName '/Wheel_L'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelL/LConn1');
add_line(modelName,'RT_WheelL/RConn1',      'RT_WheelL_Axis/LConn1');
add_line(modelName,'RT_WheelL_Axis/RConn1', 'Joint_phi_L/LConn1');
add_line(modelName,'Joint_phi_L/RConn1',    'Wheel_L/RConn1');

% Suelo visual
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Ground'],'Position',[30 280 150 310]);
set_param([modelName '/RT_Ground'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Z','TranslationStandardOffset','0.025');
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Ground'],'Position',[200 280 320 310]);
set_param([modelName '/Ground'],'BrickDimensions','[10 10 0.05]','InertiaType','Custom','Mass','1e6','CenterOfMass','[0 0 0]','MomentsOfInertia','[1e6 1e6 1e6]','ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.6 0.6 0.6]');
add_line(modelName,'World/RConn1','RT_Ground/LConn1');
add_line(modelName,'RT_Ground/RConn1','Ground/RConn1');

fprintf('[Build] Estructura fisica OK\n');

%% =========================================================================
%  5. UPDATE → verificar port handles (leccion aprendida v3)
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
%  6. REFERENCIAS — theta_ref y alpha_ref como Steps (identico patron)
% =========================================================================
% theta_ref
tref_str = num2str(theta_ref_deg*pi/180,'%.6f');
add_block('simulink/Sources/Step',[modelName '/Step_th_on'], 'Position',[30 360 100 390]);
set_param([modelName '/Step_th_on'],'Time',num2str(t_avance),'Before','0','After',tref_str);
add_block('simulink/Sources/Step',[modelName '/Step_th_off'],'Position',[30 405 100 435]);
set_param([modelName '/Step_th_off'],'Time',num2str(t_stop),'Before','0','After',['-' tref_str]);
add_block('simulink/Math Operations/Sum',[modelName '/Sum_tref'],'Position',[140 368 180 427]);
set_param([modelName '/Sum_tref'],'Inputs','++');
pH_Stref = get_param([modelName '/Sum_tref'],'PortHandles');
add_line(modelName,get_param([modelName '/Step_th_on'], 'PortHandles').Outport(1), pH_Stref.Inport(1),'autorouting','on');
add_line(modelName,get_param([modelName '/Step_th_off'],'PortHandles').Outport(1), pH_Stref.Inport(2),'autorouting','on');

% alpha_ref
aref_str = num2str(alpha_ref_deg*pi/180,'%.6f');
add_block('simulink/Sources/Step',[modelName '/Step_al_on'], 'Position',[30 450 100 480]);
set_param([modelName '/Step_al_on'],'Time',num2str(t_avance_a),'Before','0','After',aref_str);
add_block('simulink/Sources/Step',[modelName '/Step_al_off'],'Position',[30 495 100 525]);
set_param([modelName '/Step_al_off'],'Time',num2str(t_stop_a),'Before','0','After',['-' aref_str]);
add_block('simulink/Math Operations/Sum',[modelName '/Sum_aref'],'Position',[140 458 180 517]);
set_param([modelName '/Sum_aref'],'Inputs','++');
pH_Saref = get_param([modelName '/Sum_aref'],'PortHandles');
add_line(modelName,get_param([modelName '/Step_al_on'], 'PortHandles').Outport(1), pH_Saref.Inport(1),'autorouting','on');
add_line(modelName,get_param([modelName '/Step_al_off'],'PortHandles').Outport(1), pH_Saref.Inport(2),'autorouting','on');

fprintf('[Refs] theta_ref=%.1f @ t=%.0f  |  alpha_ref=%.1f @ t=%.0f\n',...
    theta_ref_deg, t_avance, alpha_ref_deg, t_avance_a);

%% =========================================================================
%  7. CADENA K1 — AVANCE
%  [theta-theta_ref, dtheta, x, dx] → Gain_K1 → Sat_Va
% =========================================================================
ps_av = {'PS2SL_th','PS2SL_dth','PS2SL_x','PS2SL_dx'};
sp_av = {pH_Jt.RConn(2), pH_Jt.RConn(3), pH_Jx.RConn(2), pH_Jx.RConn(3)};
yp_av = [560 600 640 680];
for i = 1:4
    add_block('nesl_utility/PS-Simulink Converter',[modelName '/' ps_av{i}],'Position',[220 yp_av(i)-15 330 yp_av(i)+15]);
    add_line(modelName, sp_av{i}, get_param([modelName '/' ps_av{i}],'PortHandles').LConn(1),'autorouting','on');
end

% Error theta
add_block('simulink/Math Operations/Sum',[modelName '/Sum_err_th'],'Position',[370 548 410 582]);
set_param([modelName '/Sum_err_th'],'Inputs','+-');
pH_Eth = get_param([modelName '/Sum_err_th'],'PortHandles');
add_line(modelName, get_param([modelName '/PS2SL_th'],'PortHandles').Outport(1), pH_Eth.Inport(1),'autorouting','on');
add_line(modelName, pH_Stref.Outport(1), pH_Eth.Inport(2),'autorouting','on');

% Mux4 K1
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_K1'],'Position',[450 548 480 682]);
set_param([modelName '/Mux_K1'],'Inputs','4');
pH_MK1 = get_param([modelName '/Mux_K1'],'PortHandles');
add_line(modelName, pH_Eth.Outport(1), pH_MK1.Inport(1),'autorouting','on');
for i = 2:4
    add_line(modelName, get_param([modelName '/' ps_av{i}],'PortHandles').Outport(1), pH_MK1.Inport(i),'autorouting','on');
end

% Gain K1 (1x4)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_K1'],'Position',[510 605 610 635]);
set_param([modelName '/Gain_K1'],'Gain',mat2str(-K1),'Multiplication','Matrix(K*u)');
pH_GK1 = get_param([modelName '/Gain_K1'],'PortHandles');
add_line(modelName, pH_MK1.Outport(1), pH_GK1.Inport(1),'autorouting','on');

% Sat Va
add_block('simulink/Discontinuities/Saturation',[modelName '/Sat_Va'],'Position',[640 605 710 635]);
set_param([modelName '/Sat_Va'],'UpperLimit',num2str(V_sat_a),'LowerLimit',num2str(-V_sat_a));
pH_SVa = get_param([modelName '/Sat_Va'],'PortHandles');
add_line(modelName, pH_GK1.Outport(1), pH_SVa.Inport(1),'autorouting','on');

fprintf('[K1] Cadena avance OK\n');

%% =========================================================================
%  8. CADENA K2 — GIRO
%  [alpha-alpha_ref, dalpha] → Gain_K2 → Sat_Vd
%  Convencion: Vd>0 → giro derecha (tau_R > tau_L → VR > VL)
% =========================================================================
add_block('nesl_utility/PS-Simulink Converter',[modelName '/PS2SL_al'], 'Position',[220 725 330 755]);
add_block('nesl_utility/PS-Simulink Converter',[modelName '/PS2SL_dal'],'Position',[220 765 330 795]);
add_line(modelName, pH_Ja.RConn(2), get_param([modelName '/PS2SL_al'], 'PortHandles').LConn(1),'autorouting','on');
add_line(modelName, pH_Ja.RConn(3), get_param([modelName '/PS2SL_dal'],'PortHandles').LConn(1),'autorouting','on');

% Error alpha
add_block('simulink/Math Operations/Sum',[modelName '/Sum_err_al'],'Position',[370 723 410 757]);
set_param([modelName '/Sum_err_al'],'Inputs','+-');
pH_Eal = get_param([modelName '/Sum_err_al'],'PortHandles');
add_line(modelName, get_param([modelName '/PS2SL_al'],'PortHandles').Outport(1), pH_Eal.Inport(1),'autorouting','on');
add_line(modelName, pH_Saref.Outport(1), pH_Eal.Inport(2),'autorouting','on');

% Mux2 K2
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_K2'],'Position',[450 723 480 797]);
set_param([modelName '/Mux_K2'],'Inputs','2');
pH_MK2 = get_param([modelName '/Mux_K2'],'PortHandles');
add_line(modelName, pH_Eal.Outport(1), pH_MK2.Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/PS2SL_dal'],'PortHandles').Outport(1), pH_MK2.Inport(2),'autorouting','on');

% Gain K2 (1x2)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_K2'],'Position',[510 745 610 775]);
set_param([modelName '/Gain_K2'],'Gain',mat2str(-K2),'Multiplication','Matrix(K*u)');
pH_GK2 = get_param([modelName '/Gain_K2'],'PortHandles');
add_line(modelName, pH_MK2.Outport(1), pH_GK2.Inport(1),'autorouting','on');

% Sat Vd
add_block('simulink/Discontinuities/Saturation',[modelName '/Sat_Vd'],'Position',[640 745 710 775]);
set_param([modelName '/Sat_Vd'],'UpperLimit',num2str(V_sat_d),'LowerLimit',num2str(-V_sat_d));
pH_SVd = get_param([modelName '/Sat_Vd'],'PortHandles');
add_line(modelName, pH_GK2.Outport(1), pH_SVd.Inport(1),'autorouting','on');

fprintf('[K2] Cadena giro OK  (Vd>0 = giro derecha)\n');

%% =========================================================================
%  9. MEZCLA Va/Vd → VR, VL + SATURACION FINAL +-24V
% =========================================================================
% VR = Va + Vd
add_block('simulink/Math Operations/Sum',[modelName '/Sum_VR'],'Position',[750 600 790 630]);
set_param([modelName '/Sum_VR'],'Inputs','++');
pH_SVR = get_param([modelName '/Sum_VR'],'PortHandles');
add_line(modelName, pH_SVa.Outport(1), pH_SVR.Inport(1),'autorouting','on');
add_line(modelName, pH_SVd.Outport(1), pH_SVR.Inport(2),'autorouting','on');

% VL = Va - Vd
add_block('simulink/Math Operations/Sum',[modelName '/Sum_VL'],'Position',[750 645 790 675]);
set_param([modelName '/Sum_VL'],'Inputs','+-');
pH_SVL = get_param([modelName '/Sum_VL'],'PortHandles');
add_line(modelName, pH_SVa.Outport(1), pH_SVL.Inport(1),'autorouting','on');
add_line(modelName, pH_SVd.Outport(1), pH_SVL.Inport(2),'autorouting','on');

% Sat final VR
add_block('simulink/Discontinuities/Saturation',[modelName '/Sat_VR'],'Position',[820 600 890 630]);
set_param([modelName '/Sat_VR'],'UpperLimit',num2str(V_sat_f),'LowerLimit',num2str(-V_sat_f));
pH_SatVR = get_param([modelName '/Sat_VR'],'PortHandles');
add_line(modelName, pH_SVR.Outport(1), pH_SatVR.Inport(1),'autorouting','on');

% Sat final VL
add_block('simulink/Discontinuities/Saturation',[modelName '/Sat_VL'],'Position',[820 645 890 675]);
set_param([modelName '/Sat_VL'],'UpperLimit',num2str(V_sat_f),'LowerLimit',num2str(-V_sat_f));
pH_SatVL = get_param([modelName '/Sat_VL'],'PortHandles');
add_line(modelName, pH_SVL.Outport(1), pH_SatVL.Inport(1),'autorouting','on');

fprintf('[Mezcla] VR=Va+Vd | VL=Va-Vd | Sat +-%.0fV OK\n', V_sat_f);

%% =========================================================================
%  10. TORQUES A JOINTS
%  tau_R     = alm*VR          → Joint_phi_R
%  tau_L     = alm*VL          → Joint_phi_L
%  F_x       = alm*2*Va/r      → Joint_x   (VR+VL=2*Va, Vd cancela)
%  tau_alpha = d*alm*Vd/r      → Joint_alpha  [N·m] check OK arriba
% =========================================================================
% tau_R → Joint_phi_R
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauR'],'Position',[930 595 1010 625]);
set_param([modelName '/Gain_tauR'],'Gain',num2str(alm));
pH_GtR = get_param([modelName '/Gain_tauR'],'PortHandles');
add_line(modelName, pH_SatVR.Outport(1), pH_GtR.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauR'],'Position',[1040 595 1120 625]);
add_line(modelName, pH_GtR.Outport(1), get_param([modelName '/SL2PS_tauR'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauR/RConn1','Joint_phi_R/LConn2');

% tau_L → Joint_phi_L
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauL'],'Position',[930 640 1010 670]);
set_param([modelName '/Gain_tauL'],'Gain',num2str(alm));
pH_GtL = get_param([modelName '/Gain_tauL'],'PortHandles');
add_line(modelName, pH_SatVL.Outport(1), pH_GtL.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauL'],'Position',[1040 640 1120 670]);
add_line(modelName, pH_GtL.Outport(1), get_param([modelName '/SL2PS_tauL'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauL/RConn1','Joint_phi_L/LConn2');

% F_x = alm*2*Va/r → Joint_x
add_block('simulink/Math Operations/Gain',[modelName '/Gain_Fx'],'Position',[750 545 850 575]);
set_param([modelName '/Gain_Fx'],'Gain',num2str(2*alm/r));
pH_GFx = get_param([modelName '/Gain_Fx'],'PortHandles');
add_line(modelName, pH_SVa.Outport(1), pH_GFx.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fx'],'Position',[870 545 950 575]);
add_line(modelName, pH_GFx.Outport(1), get_param([modelName '/SL2PS_Fx'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_Fx/RConn1','Joint_x/LConn2');

% tau_alpha = d*alm*Vd/r → Joint_alpha  (Vd>0 = giro derecha)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tau_al'],'Position',[750 790 850 820]);
set_param([modelName '/Gain_tau_al'],'Gain',num2str(d*alm/r));
pH_Gtal = get_param([modelName '/Gain_tau_al'],'PortHandles');
add_line(modelName, pH_SVd.Outport(1), pH_Gtal.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tau_al'],'Position',[870 790 950 820]);
add_line(modelName, pH_Gtal.Outport(1), get_param([modelName '/SL2PS_tau_al'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tau_al/RConn1','Joint_alpha/LConn2');

fprintf('[Torques] tau_R | tau_L | F_x | tau_alpha → joints OK\n');

%% =========================================================================
%  11. LOGGING
% =========================================================================
% VR/VL log
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VRVL'],'Position',[920 685 950 715]);
set_param([modelName '/Mux_VRVL'],'Inputs','2');
pH_MVRVL = get_param([modelName '/Mux_VRVL'],'PortHandles');
add_line(modelName, pH_SatVR.Outport(1), pH_MVRVL.Inport(1),'autorouting','on');
add_line(modelName, pH_SatVL.Outport(1), pH_MVRVL.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VRVL'],'Position',[970 690 1070 710]);
set_param([modelName '/ToWS_VRVL'],'VariableName','VRVL_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVRVL.Outport(1), get_param([modelName '/ToWS_VRVL'],'PortHandles').Inport(1),'autorouting','on');

% Va/Vd log
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VaVd'],'Position',[750 690 780 720]);
set_param([modelName '/Mux_VaVd'],'Inputs','2');
pH_MVaVd = get_param([modelName '/Mux_VaVd'],'PortHandles');
add_line(modelName, pH_SVa.Outport(1), pH_MVaVd.Inport(1),'autorouting','on');
add_line(modelName, pH_SVd.Outport(1), pH_MVaVd.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VaVd'],'Position',[800 693 900 713]);
set_param([modelName '/ToWS_VaVd'],'VariableName','VaVd_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVaVd.Outport(1), get_param([modelName '/ToWS_VaVd'],'PortHandles').Inport(1),'autorouting','on');

fprintf('[Logging] VRVL_log | VaVd_log OK\n');

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

    % Figura 1: theta ODE vs Simscape
    figure('Name','v4 — Angulo de inclinacion','Position',[50 430 900 340]);
    hold on; grid on;
    plot(t_ode, rad2deg(X_ode(:,1)),'r--','LineWidth',2,'DisplayName','ODE + K1/K2');
    plot(t_sm,  rad2deg(theta_sm),  'b',  'LineWidth',2,'DisplayName','Simscape + K1/K2');
    yline(0,'--k','Equilibrio');
    xlabel('t [s]'); ylabel('\theta [deg]');
    title(sprintf('\\theta(t)  |  \\theta_0=%.0f  \\alpha_0=%.0f  |  Suelo plano', theta0_deg, alpha0_deg));
    legend('Location','best'); xlim([0 t_sim]);

    % Figura 2: VR y VL ODE vs Simscape
    figure('Name','v4 — VR y VL (motores)','Position',[50 60 900 310]);
    hold on; grid on;
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

    % Figura 3: Va y Vd — evidencia desacoplamiento
    figure('Name','v4 — Va y Vd (desacoplamiento)','Position',[960 60 900 310]);
    hold on; grid on;
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
fprintf(' LISTO v4\n');
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

    % Referencias activas
    tref_th = get_ref(t, theta_ref_rad, t_avance,   t_stop);
    tref_al = get_ref(t, alpha_ref_rad, t_avance_a, t_stop_a);

    % K1 avance
    Xe1 = [X(1)-tref_th; X(2); X(3); X(4)];
    Va  = max(min(-K1*Xe1, V_sat_a), -V_sat_a);

    % K2 giro
    Xe2 = [X(5)-tref_al; X(6)];
    Vd  = max(min(-K2*Xe2, V_sat_d), -V_sat_d);

    % Mezcla + saturacion final
    VR = max(min(Va+Vd, V_sat_f), -V_sat_f);
    VL = max(min(Va-Vd, V_sat_f), -V_sat_f);

    % Torques sin back-EMF
    tau_R = alm*VR;
    tau_L = alm*VL;

    % Matrices de masa (no lineales)
    theta = X(1); dtheta = X(2);
    M11  = Icy + M*l^2;
    M12  = M*l*cos(theta);
    M22  = M + 2*m + 2*Iw/r^2;
    M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    detM = M11*M22 - M12^2;

    % Fuerzas generalizadas (ecuaciones de Kane, sin back-EMF, sin Td)
    F1 = M*g*l*sin(theta) - (tau_R + tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R + tau_L)/r;
    F3 = d*(tau_R - tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / detM;
    ddx     = (-M12*F1 + M11*F2) / detM;
    ddalpha = F3 / M33;

    dX = [dtheta; ddtheta; X(4); ddx; X(6); ddalpha];
end
