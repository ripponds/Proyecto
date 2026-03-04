%% Simscape_LQR_v3.m
%  TESTBENCH — Segway Gemelo Digital
%  ─ LQR 4 estados: [theta, dtheta, x, dx]
%  ─ Rutina de avance: theta_ref escalón (t=2s adelante, t=6s stop)
%  ─ Rampa de acera simulada via gravedad inclinada
%  ─ Salidas: theta ODE vs Simscape  |  esfuerzo de control U(t)
%  ─ Sin back-EMF (consistencia planta lineal ↔ Simscape)
%  ─ Un solo script — MATLAB/Simulink R2025b
% =========================================================================
clear; clc; close all; bdclose all;

%% ══════════════════════════════════════════════════════════════════════════
%  1. PARÁMETROS — MODIFICA AQUÍ TUS VALORES REALES
% ══════════════════════════════════════════════════════════════════════════
% ── Vehículo ──
M        = 80;      % masa cuerpo [kg]          ← placeholder Segway comercial
r        = 0.20;    % radio rueda [m]
d        = 0.60;    % separación entre ruedas [m]
l        = 0.90;    % dist. eje ruedas → CM cuerpo [m]
g        = 9.81;
m        = 2;       % masa cada rueda [kg]
Icy      = 10;      % inercia cuerpo eje pitch [kg·m²]
Icz      = 12;      % inercia cuerpo eje yaw [kg·m²]
Icx      = 12;      % inercia cuerpo eje roll [kg·m²]
Iw       = 0.08;    % inercia rueda spin [kg·m²]
Iwz      = 0.04;    % inercia rueda transversal [kg·m²]
alm      = 2.0;     % ganancia par motor [N·m/V]

% ── Geometría visual ──
body_W   = 0.40;
body_D   = 0.20;
body_H   = 1.60;

% ── Condición inicial ──
theta0_deg = 14;
theta0     = theta0_deg * pi/180;

% ── Simulación ──
t_sim    = 15;      % duración [s]
V_sat    = 12;      % saturación voltaje motor [V]

% ── RUTINA DE AVANCE — modifica estos dos parámetros para cambiar la maniobra ──
theta_ref_deg = 0;          % inclinación de referencia para avanzar [°]
t_avance      = 999;          % tiempo en que arranca el avance [s]
t_stop        = 1000;          % tiempo en que se detiene [s]

% ── ESCENARIO DE TERRENO ──
%   ramp_deg = 0   → suelo plano
%   ramp_deg = 4   → rampa de acera típica (~4%)
%   ramp_deg = 8   → rampa pronunciada
ramp_deg = 0;               % ← CAMBIA AQUÍ para el escenario que quieras
ramp_rad = ramp_deg * pi/180;

% Componentes de gravedad en rampa:
%   g_t  → componente a lo largo de la superficie (frena el avance)
%   g_n  → componente normal a la superficie (peso efectivo del péndulo)
g_t = g * sin(ramp_rad);    % perturbación traslacional [m/s²]
g_n = g * cos(ramp_rad);    % gravedad efectiva péndulo [m/s²]

fprintf('═══════════════════════════════════════════════════\n');
fprintf(' TESTBENCH Segway v3\n');
fprintf(' Escenario: rampa %.1f°  |  Avance: %.1f° ref\n', ramp_deg, theta_ref_deg);
fprintf('═══════════════════════════════════════════════════\n');

%% ══════════════════════════════════════════════════════════════════════════
%  2. LINEALIZACIÓN Y LQR — planta lineal en rampa (g_n)
% ══════════════════════════════════════════════════════════════════════════
%  Estados: X = [theta, dtheta, x, dx]
%  Control: U = -K * [theta - theta_ref;  dtheta;  x;  dx]
%  tau = alm * V  (sin back-EMF)

M11  = Icy + M*l^2;
M12  = M*l;
M22  = M + 2*m + 2*Iw/r^2;
det0 = M11*M22 - M12^2;

A = zeros(4);
A(1,2) = 1;
A(2,1) = M22 * M*g_n*l / det0;     % usa g_n (pendiente)
A(3,4) = 1;
A(4,1) = -M12 * M*g_n*l / det0;

b21 = (M22*(-alm) - M12*(alm/r)) / det0;
b41 = (M12*alm    + M11*(alm/r)) / det0;
B      = zeros(4,2);
B(2,1) = b21;  B(2,2) = b21;
B(4,1) = b41;  B(4,2) = b41;

% Verificar controlabilidad
rango_ctrl = rank(ctrb(A,B));
fprintf('[LQR] Controlabilidad: %d/4', rango_ctrl);
if rango_ctrl == 4, fprintf(' ✓\n'); else, fprintf(' ✗ — REVISAR\n'); end

% Sintonía LQR — ajusta Q para cambiar agresividad
Q = diag([2000, 100, 50, 50]);  % [theta, dtheta, x, dx]
R = diag([1, 1]);
[K, ~, eigs_cl] = lqr(A, B, Q, R);

fprintf('[LQR] K = [%.2f  %.2f  %.2f  %.2f]\n', K(1,:));
fprintf('[LQR] Polos lazo cerrado: ');
fprintf('%.3f  ', real(eigs_cl)); fprintf('\n');

%% ══════════════════════════════════════════════════════════════════════════
%  3. ODE NO LINEAL + LQR (referencia — incluye rampa y theta_ref)
% ══════════════════════════════════════════════════════════════════════════
theta_ref_rad = theta_ref_deg * pi/180;
X0 = [theta0; 0; 0; 0];

ode_fn = @(t,X) ode_nl_lqr(t, X, K, V_sat, M, m, r, l, ...
    g_n, g_t, Icy, Iw, alm, ...
    theta_ref_rad, t_avance, t_stop);

[t_ode, X_ode] = ode45(ode_fn, [0 t_sim], X0, odeset('RelTol',1e-6));

% Reconstruir U para graficar esfuerzo de control ODE
U_ode = zeros(length(t_ode), 2);
for i = 1:length(t_ode)
    tref = get_thetaref(t_ode(i), theta_ref_rad, t_avance, t_stop);
    Xerr = X_ode(i,:)'; Xerr(1) = Xerr(1) - tref;
    U_ode(i,:) = max(min((-K*Xerr)', V_sat), -V_sat);
end

fprintf('[ODE] theta_final = %.3f°  |  x_final = %.3f m\n', ...
    rad2deg(X_ode(end,1)), X_ode(end,3));

%% ══════════════════════════════════════════════════════════════════════════
%  4. CONSTRUIR MODELO SIMSCAPE
% ══════════════════════════════════════════════════════════════════════════
modelName = 'Segway_Testbench_v3';
if bdIsLoaded(modelName),            close_system(modelName, 0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver','ode23t', 'StopTime',num2str(t_sim), ...
    'RelTol','1e-4', 'AbsTol','1e-6');

% ── Gravedad inclinada para simular rampa ──
grav_str = sprintf('[-%.6f; 0; -%.6f]', g*sin(ramp_rad), g*cos(ramp_rad));  % rampa: -X=avance, -Z=vertical

% ── Bloques de utilidad ──
add_block('nesl_utility/Solver Configuration',        [modelName '/Solver_Config'],'Position',[30 30 180 60]);
add_block('sm_lib/Utilities/Mechanism Configuration', [modelName '/Mech_Config'],  'Position',[30 90 220 120]);
set_param([modelName '/Mech_Config'], 'GravityVector', grav_str);
add_block('sm_lib/Frames and Transforms/World Frame', [modelName '/World'],         'Position',[30 160 130 190]);
add_line(modelName,'World/RConn1','Solver_Config/RConn1');
add_line(modelName,'World/RConn1','Mech_Config/RConn1');

% ── Cadena: World → RT_AxleHeight → RT_PrismaticAlign → Joint_x ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[220 160 340 190]);
set_param([modelName '/RT_AxleHeight'], ...
    'RotationMethod','None','TranslationMethod','StandardAxis', ...
    'TranslationStandardAxis','+Z','TranslationStandardOffset',num2str(r));

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_PrismaticAlign'],'Position',[400 160 520 190]);
set_param([modelName '/RT_PrismaticAlign'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','90', ...
    'TranslationMethod','None');

% Joint_x — traslación + InputTorque (fuerza de tracción desde LQR)
add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[580 145 700 205]);
set_param([modelName '/Joint_x'], ...
    'MotionActuationMode','ComputedMotion', ...
    'TorqueActuationMode','InputTorque', ...
    'SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign'],'Position',[740 145 860 185]);
set_param([modelName '/RT_Realign'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','-90', ...
    'TranslationMethod','None');

% Joint_alpha — guiñada libre (sin control por ahora)
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[920 145 1040 205]);
set_param([modelName '/Joint_alpha'], ...
    'TorqueActuationMode','NoTorque','MotionActuationMode','ComputedMotion', ...
    'DampingCoefficient','5');  % amortiguamiento para evitar giro libre en avance recto

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[960 60 1080 90]);
set_param([modelName '/RT_ThetaAxis'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90', ...
    'TranslationMethod','None');

% Joint_theta — CI + sensing
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[1140 45 1260 105]);
set_param([modelName '/Joint_theta'], ...
    'PositionTargetSpecify','on', ...
    'PositionTargetValue',num2str(theta0_deg), ...
    'PositionTargetValueUnits','deg', ...
    'PositionTargetPriority','High', ...
    'TorqueActuationMode','NoTorque','MotionActuationMode','ComputedMotion', ...
    'DampingCoefficient','0','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1320 45 1440 105]);
set_param([modelName '/RT_BodyCM'], ...
    'RotationMethod','None','TranslationMethod','StandardAxis', ...
    'TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(l));

add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1500 45 1620 105]);
set_param([modelName '/Body_Solid'], ...
    'BrickDimensions',mat2str([body_D, body_H, body_W]),'InertiaType','Custom', ...
    'Mass',num2str(M),'CenterOfMass','[0 0 0]', ...
    'MomentsOfInertia',mat2str([Icx, Icy, Icz]),'ProductsOfInertia','[0 0 0]', ...
    'GraphicDiffuseColor','[0.2 0.5 0.8]');

% Conexiones cadena principal
add_line(modelName,'World/RConn1',            'RT_AxleHeight/LConn1');
add_line(modelName,'RT_AxleHeight/RConn1',    'RT_PrismaticAlign/LConn1');
add_line(modelName,'RT_PrismaticAlign/RConn1','Joint_x/LConn1');
add_line(modelName,'Joint_x/RConn1',          'RT_Realign/LConn1');
add_line(modelName,'RT_Realign/RConn1',        'Joint_alpha/LConn1');
add_line(modelName,'Joint_alpha/RConn1',       'RT_ThetaAxis/LConn1');
add_line(modelName,'RT_ThetaAxis/RConn1',      'Joint_theta/LConn1');
add_line(modelName,'Joint_theta/RConn1',       'RT_BodyCM/LConn1');
add_line(modelName,'RT_BodyCM/RConn1',         'Body_Solid/RConn1');

% ── Rueda derecha ──
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

% ── Rueda izquierda ──
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

% ── Suelo visual ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Ground'],'Position',[30 280 150 310]);
set_param([modelName '/RT_Ground'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Z','TranslationStandardOffset','0.025');
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Ground'],'Position',[200 280 320 310]);
set_param([modelName '/Ground'],'BrickDimensions','[10 10 0.05]','InertiaType','Custom','Mass','1e6','CenterOfMass','[0 0 0]','MomentsOfInertia','[1e6 1e6 1e6]','ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.6 0.6 0.6]');
add_line(modelName,'World/RConn1',     'RT_Ground/LConn1');
add_line(modelName,'RT_Ground/RConn1', 'Ground/RConn1');

fprintf('[Build] Estructura física OK — gravedad: %s\n', grav_str);

%% ══════════════════════════════════════════════════════════════════════════
%  5. UPDATE → port handles
% ══════════════════════════════════════════════════════════════════════════
set_param(modelName,'SimulationCommand','update');
pH_Jt  = get_param([modelName '/Joint_theta'], 'PortHandles');
pH_Jx  = get_param([modelName '/Joint_x'],     'PortHandles');
pH_JpR = get_param([modelName '/Joint_phi_R'], 'PortHandles');
pH_JpL = get_param([modelName '/Joint_phi_L'], 'PortHandles');
fprintf('[Ports] Joint_theta RConn=%d | Joint_x RConn=%d | Joint_phi_R RConn=%d\n', ...
    numel(pH_Jt.RConn), numel(pH_Jx.RConn), numel(pH_JpR.RConn));

%% ══════════════════════════════════════════════════════════════════════════
%  6. RUTINA DE AVANCE: theta_ref(t) via Step blocks
%     t < t_avance  → theta_ref = 0
%     t_avance ≤ t < t_stop  → theta_ref = +theta_ref_rad
%     t ≥ t_stop   → theta_ref = 0
%  Implementado como: Step1(t_avance, +ref) + Step2(t_stop, -ref)
% ══════════════════════════════════════════════════════════════════════════
theta_ref_rad_str = num2str(theta_ref_deg * pi/180, '%.6f');

add_block('simulink/Sources/Step',[modelName '/Step_avance'],'Position',[30 380 100 410]);
set_param([modelName '/Step_avance'], ...
    'Time',num2str(t_avance), ...
    'Before',num2str(0), ...
    'After', theta_ref_rad_str);

add_block('simulink/Sources/Step',[modelName '/Step_stop'],'Position',[30 430 100 460]);
set_param([modelName '/Step_stop'], ...
    'Time',num2str(t_stop), ...
    'Before',num2str(0), ...
    'After', ['-' theta_ref_rad_str]);

add_block('simulink/Math Operations/Sum',[modelName '/Sum_tref'],'Position',[150 390 190 450]);
set_param([modelName '/Sum_tref'],'Inputs','++');
pH_Stref = get_param([modelName '/Sum_tref'],'PortHandles');
pH_Sav   = get_param([modelName '/Step_avance'],'PortHandles');
pH_Sst   = get_param([modelName '/Step_stop'],  'PortHandles');
add_line(modelName, pH_Sav.Outport(1), pH_Stref.Inport(1), 'autorouting','on');
add_line(modelName, pH_Sst.Outport(1), pH_Stref.Inport(2), 'autorouting','on');

fprintf('[Rutina] theta_ref: 0 → %.1f° @ t=%.1fs  → 0 @ t=%.1fs\n', ...
    theta_ref_deg, t_avance, t_stop);

%% ══════════════════════════════════════════════════════════════════════════
%  7. CADENA LQR: sensing → error(theta - theta_ref) → Mux → K → Sat → tau
% ══════════════════════════════════════════════════════════════════════════
%  Orden estados: [theta, dtheta, x, dx]
%  Joint_theta: RConn(2)=theta, RConn(3)=dtheta
%  Joint_x:     RConn(2)=x,     RConn(3)=dx

ps2sl_names   = {'PS2SL_th','PS2SL_dth','PS2SL_x','PS2SL_dx'};
sensing_ports = {pH_Jt.RConn(2), pH_Jt.RConn(3), ...
                 pH_Jx.RConn(2), pH_Jx.RConn(3)};
ypos_ps = [510 550 590 630];

for i = 1:4
    add_block('nesl_utility/PS-Simulink Converter', ...
        [modelName '/' ps2sl_names{i}],'Position',[230 ypos_ps(i)-15 340 ypos_ps(i)+15]);
    pH_ps = get_param([modelName '/' ps2sl_names{i}],'PortHandles');
    add_line(modelName, sensing_ports{i}, pH_ps.LConn(1),'autorouting','on');
end

% Restar theta_ref a theta: Sum_err = theta - theta_ref
add_block('simulink/Math Operations/Sum',[modelName '/Sum_err'],'Position',[380 505 420 545]);
set_param([modelName '/Sum_err'],'Inputs','+-');
pH_Serr = get_param([modelName '/Sum_err'],'PortHandles');
pH_th   = get_param([modelName '/PS2SL_th'],'PortHandles');
add_line(modelName, pH_th.Outport(1),      pH_Serr.Inport(1),'autorouting','on');
add_line(modelName, pH_Stref.Outport(1),   pH_Serr.Inport(2),'autorouting','on');

% Mux 4→1: [theta_err, dtheta, x, dx]
add_block('simulink/Signal Routing/Mux',[modelName '/Mux4'],'Position',[460 505 490 645]);
set_param([modelName '/Mux4'],'Inputs','4');
pH_Mux = get_param([modelName '/Mux4'],'PortHandles');
add_line(modelName, pH_Serr.Outport(1), pH_Mux.Inport(1),'autorouting','on');
for i = 2:4
    pH_ps = get_param([modelName '/' ps2sl_names{i}],'PortHandles');
    add_line(modelName, pH_ps.Outport(1), pH_Mux.Inport(i),'autorouting','on');
end

% Gain: -K (2×4)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_K'],'Position',[530 565 630 595]);
set_param([modelName '/Gain_K'],'Gain',mat2str(-K),'Multiplication','Matrix(K*u)');
pH_GainK = get_param([modelName '/Gain_K'],'PortHandles');
add_line(modelName, pH_Mux.Outport(1), pH_GainK.Inport(1),'autorouting','on');

% Saturación ±V_sat
add_block('simulink/Discontinuities/Saturation',[modelName '/Sat'],'Position',[660 565 730 595]);
set_param([modelName '/Sat'],'UpperLimit',num2str(V_sat),'LowerLimit',num2str(-V_sat));
pH_Sat = get_param([modelName '/Sat'],'PortHandles');
add_line(modelName, pH_GainK.Outport(1), pH_Sat.Inport(1),'autorouting','on');

% Demux 1→2 (VR, VL)
add_block('simulink/Signal Routing/Demux',[modelName '/Demux2'],'Position',[760 570 790 590]);
set_param([modelName '/Demux2'],'Outputs','2');
pH_Dmx = get_param([modelName '/Demux2'],'PortHandles');
add_line(modelName, pH_Sat.Outport(1), pH_Dmx.Inport(1),'autorouting','on');

% tau = alm * V → SL2PS → ruedas
add_block('simulink/Math Operations/Gain',[modelName '/Gain_almR'],'Position',[820 555 900 575]);
add_block('simulink/Math Operations/Gain',[modelName '/Gain_almL'],'Position',[820 585 900 605]);
set_param([modelName '/Gain_almR'],'Gain',num2str(alm));
set_param([modelName '/Gain_almL'],'Gain',num2str(alm));
pH_GaR = get_param([modelName '/Gain_almR'],'PortHandles');
pH_GaL = get_param([modelName '/Gain_almL'],'PortHandles');
add_line(modelName, pH_Dmx.Outport(1), pH_GaR.Inport(1),'autorouting','on');
add_line(modelName, pH_Dmx.Outport(2), pH_GaL.Inport(1),'autorouting','on');

add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauR'],'Position',[930 555 1010 575]);
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauL'],'Position',[930 585 1010 605]);
pH_SL_R = get_param([modelName '/SL2PS_tauR'],'PortHandles');
pH_SL_L = get_param([modelName '/SL2PS_tauL'],'PortHandles');
add_line(modelName, pH_GaR.Outport(1), pH_SL_R.Inport(1),'autorouting','on');
add_line(modelName, pH_GaL.Outport(1), pH_SL_L.Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauR/RConn1','Joint_phi_R/LConn2');
add_line(modelName,'SL2PS_tauL/RConn1','Joint_phi_L/LConn2');

fprintf('[LQR] Cadena control OK — error = theta - theta_ref\n');

%% ══════════════════════════════════════════════════════════════════════════
%  8. FUERZA DE TRACCIÓN → Joint_x:  F_x = alm*(VR+VL)/r
% ══════════════════════════════════════════════════════════════════════════
add_block('simulink/Math Operations/Sum',[modelName '/Sum_VV'],'Position',[820 650 860 690]);
set_param([modelName '/Sum_VV'],'Inputs','++');
pH_SVV = get_param([modelName '/Sum_VV'],'PortHandles');
add_line(modelName, pH_Dmx.Outport(1), pH_SVV.Inport(1),'autorouting','on');
add_line(modelName, pH_Dmx.Outport(2), pH_SVV.Inport(2),'autorouting','on');

add_block('simulink/Math Operations/Gain',[modelName '/Gain_Fx'],'Position',[890 655 970 685]);
set_param([modelName '/Gain_Fx'],'Gain',num2str(alm/r));
pH_GFx = get_param([modelName '/Gain_Fx'],'PortHandles');
add_line(modelName, pH_SVV.Outport(1), pH_GFx.Inport(1),'autorouting','on');

add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fx'],'Position',[1000 655 1080 685]);
pH_SL_Fx = get_param([modelName '/SL2PS_Fx'],'PortHandles');
add_line(modelName, pH_GFx.Outport(1), pH_SL_Fx.Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_Fx/RConn1','Joint_x/LConn2');

fprintf('[Tracción] F_x = alm*(VR+VL)/r → Joint_x OK\n');

%% ══════════════════════════════════════════════════════════════════════════
%  9. LOGGING — guardar U(t) para gráfica de esfuerzo de control
% ══════════════════════════════════════════════════════════════════════════
% Estados completos al workspace
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_state'],'Position',[460 700 560 730]);
set_param([modelName '/ToWS_state'],'VariableName','state_log','SaveFormat','Array','SampleTime','-1');
pH_TWS = get_param([modelName '/ToWS_state'],'PortHandles');
add_line(modelName, pH_Mux.Outport(1), pH_TWS.Inport(1),'autorouting','on');

% Control effort U al workspace
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_U'],'Position',[660 620 760 650]);
set_param([modelName '/ToWS_U'],'VariableName','U_log','SaveFormat','Array','SampleTime','-1');
pH_TWU = get_param([modelName '/ToWS_U'],'PortHandles');
add_line(modelName, pH_Sat.Outport(1), pH_TWU.Inport(1),'autorouting','on');

%% ══════════════════════════════════════════════════════════════════════════
%  10. GUARDAR Y SIMULAR
% ══════════════════════════════════════════════════════════════════════════
save_system(modelName,[modelName '.slx']);
fprintf('[Sim] Iniciando simulación de %g s...\n', t_sim);

try
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimscapeLogType','all','SimscapeLogName','simlog');
    out   = sim(simIn);
    fprintf('[Sim] OK  t_final=%.3f s\n', out.tout(end));
catch e
    fprintf('[Sim] FALLO: %s\n', e.message);
    return
end

%% ══════════════════════════════════════════════════════════════════════════
%  11. DIAGNÓSTICO + GRÁFICAS
% ══════════════════════════════════════════════════════════════════════════
try
    t_sm     = out.simlog.Joint_theta.Rz.q.series.time;
    theta_sm = out.simlog.Joint_theta.Rz.q.series.values('rad');
    fprintf('[Diagnóstico] theta(0)=%.4f rad (esperado %.4f)\n', theta_sm(1), theta0);
    if abs(theta_sm(1)) < 1e-3
        fprintf('  >> ADVERTENCIA: CI theta ≈ 0, revisar PositionTarget\n');
    else
        fprintf('  >> CI theta OK\n');
    end
    fprintf('[Simlog] theta_final = %.3f°\n', rad2deg(theta_sm(end)));

    % Recuperar U de Simscape desde workspace log
    t_U  = out.tout;
    U_sm = out.get('U_log');

    % ── Figura 1: theta ODE vs Simscape ──────────────────────────────────
    figure('Name','Testbench v3 — Ángulo de inclinación','Position',[50 400 900 380]);
    hold on; grid on;
    plot(t_ode, rad2deg(X_ode(:,1)), 'r--','LineWidth',2,'DisplayName','ODE No Lineal + LQR');
    plot(t_sm,  rad2deg(theta_sm),   'b',  'LineWidth',2,'DisplayName','Simscape 3D + LQR');
    xline(t_avance,'--g','Avance','LabelHorizontalAlignment','left','LineWidth',1.2);
    xline(t_stop,  '--m','Stop',  'LabelHorizontalAlignment','left','LineWidth',1.2);
    yline(0,'--k','Equilibrio');
    xlabel('t [s]'); ylabel('\theta [°]');
    title(sprintf('\\theta(t)  |  Rampa %.1f°  |  \\theta_{ref}=%.1f° (t=%.0f–%.0fs)', ...
        ramp_deg, theta_ref_deg, t_avance, t_stop));
    legend('Location','best');
    xlim([0 t_sim]);

    % ── Figura 2: esfuerzo de control ─────────────────────────────────────
    figure('Name','Testbench v3 — Esfuerzo de control','Position',[50 30 900 350]);
    hold on; grid on;
    if ~isempty(U_sm) && size(U_sm,2) >= 2
        plot(t_U, U_sm(:,1),'b','LineWidth',1.5,'DisplayName','V_R Simscape');
        plot(t_U, U_sm(:,2),'b--','LineWidth',1.5,'DisplayName','V_L Simscape');
    end
    plot(t_ode, U_ode(:,1),'r','LineWidth',1.2,'DisplayName','V_R ODE');
    plot(t_ode, U_ode(:,2),'r--','LineWidth',1.2,'DisplayName','V_L ODE');
    yline( V_sat,'--k','V_{sat}'); yline(-V_sat,'--k');
    xline(t_avance,'--g','Avance','LabelHorizontalAlignment','left');
    xline(t_stop,  '--m','Stop',  'LabelHorizontalAlignment','left');
    xlabel('t [s]'); ylabel('V [V]');
    title(sprintf('Esfuerzo de control  |  Rampa %.1f°', ramp_deg));
    legend('Location','best');
    xlim([0 t_sim]);

catch e
    fprintf('[Gráficas] FALLO: %s\n', e.message);
    disp(fieldnames(out));
end

fprintf('\n═══════════════════════════════════════════════════\n');
fprintf(' LISTO — Para cambiar escenario modifica:\n');
fprintf('   ramp_deg      (rampa actual: %.1f°)\n', ramp_deg);
fprintf('   theta_ref_deg (avance actual: %.1f°)\n', theta_ref_deg);
fprintf('   t_avance / t_stop\n');
fprintf('═══════════════════════════════════════════════════\n');

%% ══════════════════════════════════════════════════════════════════════════
%  FUNCIONES LOCALES
% ══════════════════════════════════════════════════════════════════════════
function ref = get_thetaref(t, theta_ref_rad, t_avance, t_stop)
    if t >= t_avance && t < t_stop
        ref = theta_ref_rad;
    else
        ref = 0;
    end
end

function dX = ode_nl_lqr(t, X, K, V_sat, M, m, r, l, ...
    g_n, g_t, Icy, Iw, alm, theta_ref_rad, t_avance, t_stop)
    % Referencia de ángulo activa según rutina de avance
    tref  = get_thetaref(t, theta_ref_rad, t_avance, t_stop);
    Xerr  = X;  Xerr(1) = X(1) - tref;
    U     = max(min((-K*Xerr)', V_sat), -V_sat);

    theta  = X(1);
    dtheta = X(2);
    dx     = X(4);
    VR = U(1);  VL = U(2);
    tau_sum = alm*(VR + VL);

    M11   = Icy + M*l^2;
    M12   = M*l*cos(theta);
    M22   = M + 2*m + 2*Iw/r^2;
    det_c = M11*M22 - M12^2;

    % g_n: gravedad efectiva del péndulo (componente normal a la superficie)
    % g_t: perturbación de rampa (componente tangencial, frena el avance)
    F1 = M*g_n*l*sin(theta) - tau_sum;
    F2 = tau_sum/r + M*l*dtheta^2*sin(theta) - M22*g_t;

    ddtheta = ( M22*F1 - M12*F2) / det_c;
    ddx     = (-M12*F1 + M11*F2) / det_c;

    dX = [dtheta; ddtheta; dx; ddx];
end