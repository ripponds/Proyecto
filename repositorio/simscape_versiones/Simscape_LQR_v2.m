%% Simscape_LQR_v2.m
%  Segway — Gemelo Digital con LQR en lazo cerrado
%  Restricción cinemática dura: x = r * phi_R  (Joint_x InputMotion)
%  Sin back-EMF: tau = alm*V  (consistencia planta lineal ↔ Simscape)
%  LQR sobre 4 estados: [theta, dtheta, x, dx]  (sin giro alpha)
%  MATLAB/Simulink R2025b
% ─────────────────────────────────────────────────────────────────────────
clear; clc; close all; bdclose all;

%% ── 1. PARÁMETROS ────────────────────────────────────────────────────────
M        = 80;      % masa cuerpo [kg]
r        = 0.20;    % radio rueda [m]
d        = 0.60;    % separación entre ruedas [m]
l        = 0.90;    % dist. eje ruedas → CM cuerpo [m]
g        = 9.81;
m        = 2;       % masa cada rueda [kg]
Icy      = 10;      % inercia cuerpo eje Y (pitch) [kg·m²]
Icz      = 12;      % inercia cuerpo eje Z [kg·m²]
Icx      = 12;      % inercia cuerpo eje X [kg·m²]
Iw       = 0.08;    % inercia rueda eje de spin [kg·m²]
Iwz      = 0.04;    % inercia rueda eje transversal [kg·m²]
alm      = 2.0;     % constante de par motor [N·m/V]
body_W   = 0.40;    % ancho cuerpo [m]
body_D   = 0.20;    % profundidad cuerpo [m]
body_H   = 1.60;    % altura cuerpo [m]
theta0_deg = 5;
theta0     = theta0_deg * pi/180;
t_sim      = 5;
V_sat      = 12;    % saturación de voltaje [V]

%% ── 2. LINEALIZACIÓN Y LQR (4 estados, sin back-EMF) ────────────────────
%  Estados X = [theta, dtheta, x, dx]
%  Entradas U = [V_R, V_L]
%  tau = alm*V  (directo, sin back-EMF)
%  F1 = M*g*l*theta - alm*(VR+VL)         → eq. rotación
%  F2 = alm*(VR+VL)/r                     → eq. traslación

M11  = Icy + M*l^2;
M12  = M*l;             % linealizado: cos(0) = 1
M22  = M + 2*m + 2*Iw/r^2;
det0 = M11*M22 - M12^2;

A = zeros(4);
A(1,2) = 1;
A(2,1) = M22 * M*g*l / det0;
A(3,4) = 1;
A(4,1) = -M12 * M*g*l / det0;

% B: VR y VL tienen mismo efecto (movimiento simétrico)
b21 = (M22*(-alm) - M12*(alm/r)) / det0;
b41 = (M12*alm    + M11*(alm/r)) / det0;
B = zeros(4,2);
B(2,1) = b21;  B(2,2) = b21;
B(4,1) = b41;  B(4,2) = b41;

% Verificar controlabilidad
Co = ctrb(A, B);
fprintf('[LQR] Rango controlabilidad: %d / 4\n', rank(Co));

Q = diag([2000, 100, 50, 50]);
R = diag([1, 1]);
[K, ~, ~] = lqr(A, B, Q, R);
fprintf('[LQR] K calculado OK\n');
disp(K);

%% ── 3. ODE NO LINEAL + LQR (referencia) ─────────────────────────────────
X0     = [theta0; 0; 0; 0];
ode_fn = @(t,X) ode_nl_lqr(t, X, K, V_sat, M, m, r, l, g, Icy, Iw, alm);
[t_ode, X_ode] = ode45(ode_fn, [0 t_sim], X0);
fprintf('[ODE] theta_final = %.3f°\n', rad2deg(X_ode(end,1)));

%% ── 4. CONSTRUIR MODELO SIMSCAPE ─────────────────────────────────────────
modelName = 'Segway_LQR_v2';
if bdIsLoaded(modelName),        close_system(modelName, 0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver','ode23t', 'StopTime',num2str(t_sim), ...
    'RelTol','1e-4', 'AbsTol','1e-6');

% ── Bloques de utilidad ──
add_block('nesl_utility/Solver Configuration',       [modelName '/Solver_Config'], 'Position',[30 30 180 60]);
add_block('sm_lib/Utilities/Mechanism Configuration',[modelName '/Mech_Config'],   'Position',[30 90 220 120]);
set_param([modelName '/Mech_Config'], 'GravityVector','[0;0;-9.81]');
add_block('sm_lib/Frames and Transforms/World Frame',[modelName '/World'],          'Position',[30 160 130 190]);
add_line(modelName,'World/RConn1','Solver_Config/RConn1');
add_line(modelName,'World/RConn1','Mech_Config/RConn1');

% ── RT_AxleHeight: eleva el sistema al radio de la rueda ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[220 160 340 190]);
set_param([modelName '/RT_AxleHeight'], ...
    'RotationMethod','None', ...
    'TranslationMethod','StandardAxis', ...
    'TranslationStandardAxis','+Z', ...
    'TranslationStandardOffset',num2str(r));

% ── RT_PrismaticAlign: rota para que Joint_x deslice en Y (avance) ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_PrismaticAlign'],'Position',[400 160 520 190]);
set_param([modelName '/RT_PrismaticAlign'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','90', ...
    'TranslationMethod','None');

% ── Joint_x: InputMotion (x = r*phi_R) + Sensing posición y velocidad ──
%  ForceActuationMode = NoForce  (corrección del error #2 del diagnóstico)
add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[580 145 700 205]);
set_param([modelName '/Joint_x'], ...
    'MotionActuationMode','ComputedMotion', ...
    'TorqueActuationMode','InputTorque', ...
    'SensePosition','on', ...
    'SenseVelocity','on');

% ── RT_Realign: deshace la rotación para la cadena hacia el cuerpo ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign'],'Position',[740 145 860 185]);
set_param([modelName '/RT_Realign'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','-90', ...
    'TranslationMethod','None');

% ── Joint_alpha: rotación de guiñada — libre, no controlada ──
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[920 145 1040 205]);
set_param([modelName '/Joint_alpha'], ...
    'TorqueActuationMode','NoTorque', ...
    'MotionActuationMode','ComputedMotion');

% ── RT_ThetaAxis: cambia eje para que theta sea rotación correcta ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[960 60 1080 90]);
set_param([modelName '/RT_ThetaAxis'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90', ...
    'TranslationMethod','None');

% ── Joint_theta: CI = theta0 + Sensing ──
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[1140 45 1260 105]);
set_param([modelName '/Joint_theta'], ...
    'PositionTargetSpecify','on', ...
    'PositionTargetValue',num2str(theta0_deg), ...
    'PositionTargetValueUnits','deg', ...
    'PositionTargetPriority','High', ...
    'TorqueActuationMode','NoTorque', ...
    'MotionActuationMode','ComputedMotion', ...
    'DampingCoefficient','0', ...
    'SensePosition','on', ...
    'SenseVelocity','on');

% ── RT_BodyCM: desplaza desde eje de ruedas al CM del cuerpo ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1320 45 1440 105]);
set_param([modelName '/RT_BodyCM'], ...
    'RotationMethod','None', ...
    'TranslationMethod','StandardAxis','TranslationStandardAxis','-Y', ...
    'TranslationStandardOffset',num2str(l));

% ── Cuerpo sólido ──
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1500 45 1620 105]);
set_param([modelName '/Body_Solid'], ...
    'BrickDimensions',mat2str([body_D, body_H, body_W]), ...
    'InertiaType','Custom', ...
    'Mass',num2str(M), ...
    'CenterOfMass','[0 0 0]', ...
    'MomentsOfInertia',mat2str([Icx, Icy, Icz]), ...
    'ProductsOfInertia','[0 0 0]', ...
    'GraphicDiffuseColor','[0.2 0.5 0.8]');

% ── Conexiones cadena principal ──
add_line(modelName,'World/RConn1',            'RT_AxleHeight/LConn1');
add_line(modelName,'RT_AxleHeight/RConn1',    'RT_PrismaticAlign/LConn1');
add_line(modelName,'RT_PrismaticAlign/RConn1','Joint_x/LConn1');
add_line(modelName,'Joint_x/RConn1',          'RT_Realign/LConn1');
add_line(modelName,'RT_Realign/RConn1',        'Joint_alpha/LConn1');
add_line(modelName,'Joint_alpha/RConn1',       'RT_ThetaAxis/LConn1');
add_line(modelName,'RT_ThetaAxis/RConn1',      'Joint_theta/LConn1');
add_line(modelName,'Joint_theta/RConn1',       'RT_BodyCM/LConn1');
add_line(modelName,'RT_BodyCM/RConn1',         'Body_Solid/RConn1');

% ── Rueda derecha: InputTorque + SensePosition + SenseVelocity ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[960 160 1080 190]);
set_param([modelName '/RT_WheelR'], ...
    'RotationMethod','None', ...
    'TranslationMethod','StandardAxis','TranslationStandardAxis','+Y', ...
    'TranslationStandardOffset',num2str(d/2));

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Axis'],'Position',[1140 160 1260 190]);
set_param([modelName '/RT_WheelR_Axis'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90', ...
    'TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[1320 145 1440 205]);
set_param([modelName '/Joint_phi_R'], ...
    'TorqueActuationMode','InputTorque', ...
    'MotionActuationMode','ComputedMotion', ...
    'SensePosition','on', ...
    'SenseVelocity','on');

add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[1500 145 1620 205]);
set_param([modelName '/Wheel_R'], ...
    'CylinderRadius',num2str(r),'CylinderLength','0.08', ...
    'InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]', ...
    'MomentsOfInertia',mat2str([Iwz, Iw, Iwz]),'ProductsOfInertia','[0 0 0]', ...
    'GraphicDiffuseColor','[0.15 0.15 0.15]');

add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelR/LConn1');
add_line(modelName,'RT_WheelR/RConn1',      'RT_WheelR_Axis/LConn1');
add_line(modelName,'RT_WheelR_Axis/RConn1', 'Joint_phi_R/LConn1');
add_line(modelName,'Joint_phi_R/RConn1',    'Wheel_R/RConn1');

% ── Rueda izquierda: InputTorque ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL'],'Position',[960 250 1080 280]);
set_param([modelName '/RT_WheelL'], ...
    'RotationMethod','None', ...
    'TranslationMethod','StandardAxis','TranslationStandardAxis','-Y', ...
    'TranslationStandardOffset',num2str(d/2));

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL_Axis'],'Position',[1140 250 1260 280]);
set_param([modelName '/RT_WheelL_Axis'], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90', ...
    'TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_L'],'Position',[1320 235 1440 295]);
set_param([modelName '/Joint_phi_L'], ...
    'TorqueActuationMode','InputTorque', ...
    'MotionActuationMode','ComputedMotion', ...
    'SenseVelocity','on');

add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_L'],'Position',[1500 235 1620 295]);
set_param([modelName '/Wheel_L'], ...
    'CylinderRadius',num2str(r),'CylinderLength','0.08', ...
    'InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]', ...
    'MomentsOfInertia',mat2str([Iwz, Iw, Iwz]),'ProductsOfInertia','[0 0 0]', ...
    'GraphicDiffuseColor','[0.15 0.15 0.15]');

add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelL/LConn1');
add_line(modelName,'RT_WheelL/RConn1',      'RT_WheelL_Axis/LConn1');
add_line(modelName,'RT_WheelL_Axis/RConn1', 'Joint_phi_L/LConn1');
add_line(modelName,'Joint_phi_L/RConn1',    'Wheel_L/RConn1');

% ── Suelo visual ──
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Ground'],'Position',[30 280 150 310]);
set_param([modelName '/RT_Ground'], ...
    'RotationMethod','None', ...
    'TranslationMethod','StandardAxis','TranslationStandardAxis','-Z', ...
    'TranslationStandardOffset','0.025');
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Ground'],'Position',[200 280 320 310]);
set_param([modelName '/Ground'], ...
    'BrickDimensions','[6 6 0.05]','InertiaType','Custom','Mass','1e6', ...
    'CenterOfMass','[0 0 0]','MomentsOfInertia','[1e6 1e6 1e6]','ProductsOfInertia','[0 0 0]', ...
    'GraphicDiffuseColor','[0.55 0.55 0.55]');
add_line(modelName,'World/RConn1',     'RT_Ground/LConn1');
add_line(modelName,'RT_Ground/RConn1', 'Ground/RConn1');

fprintf('[Build] Estructura física OK\n');

%% ── 5. UPDATE → obtener port handles ────────────────────────────────────
set_param(modelName,'SimulationCommand','update');
pH_Jt  = get_param([modelName '/Joint_theta'], 'PortHandles');
pH_Jx  = get_param([modelName '/Joint_x'],     'PortHandles');
pH_JpR = get_param([modelName '/Joint_phi_R'], 'PortHandles');
pH_JpL = get_param([modelName '/Joint_phi_L'], 'PortHandles');
fprintf('[Ports] Joint_theta RConn=%d | Joint_x RConn=%d | Joint_phi_R RConn=%d | Joint_phi_L RConn=%d\n', ...
    numel(pH_Jt.RConn), numel(pH_Jx.RConn), numel(pH_JpR.RConn), numel(pH_JpL.RConn));

%% ── 6. CADENA LQR: sensing → Mux → Gain(-K) → Sat → tau ────────────────
%  Orden estados: [theta, dtheta, x, dx]
%  Joint_theta: RConn(2)=theta, RConn(3)=dtheta
%  Joint_x:     RConn(2)=x,     RConn(3)=dx   (x llega desde la restricción)

ps2sl_names   = {'PS2SL_th','PS2SL_dth','PS2SL_x','PS2SL_dx'};
sensing_ports = {pH_Jt.RConn(2), pH_Jt.RConn(3), ...
                 pH_Jx.RConn(2), pH_Jx.RConn(3)};
ypos_ps = [500 540 580 620];

for i = 1:4
    add_block('nesl_utility/PS-Simulink Converter', ...
        [modelName '/' ps2sl_names{i}], 'Position',[200 ypos_ps(i)-15 310 ypos_ps(i)+15]);
    pH_ps = get_param([modelName '/' ps2sl_names{i}], 'PortHandles');
    add_line(modelName, sensing_ports{i}, pH_ps.LConn(1), 'autorouting','on');
end

% Mux 4→1
add_block('simulink/Signal Routing/Mux',[modelName '/Mux4'],'Position',[360 505 390 635]);
set_param([modelName '/Mux4'],'Inputs','4');
pH_Mux = get_param([modelName '/Mux4'],'PortHandles');
for i = 1:4
    pH_ps = get_param([modelName '/' ps2sl_names{i}],'PortHandles');
    add_line(modelName, pH_ps.Outport(1), pH_Mux.Inport(i), 'autorouting','on');
end

% Gain: -K  (2×4)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_K'],'Position',[430 555 530 585]);
set_param([modelName '/Gain_K'],'Gain',mat2str(-K),'Multiplication','Matrix(K*u)');
pH_GainK = get_param([modelName '/Gain_K'],'PortHandles');
add_line(modelName, pH_Mux.Outport(1), pH_GainK.Inport(1), 'autorouting','on');

% Saturación ±V_sat
add_block('simulink/Discontinuities/Saturation',[modelName '/Sat'],'Position',[570 555 640 585]);
set_param([modelName '/Sat'],'UpperLimit',num2str(V_sat),'LowerLimit',num2str(-V_sat));
pH_Sat = get_param([modelName '/Sat'],'PortHandles');
add_line(modelName, pH_GainK.Outport(1), pH_Sat.Inport(1), 'autorouting','on');

% Demux 1→2 (VR, VL)
add_block('simulink/Signal Routing/Demux',[modelName '/Demux2'],'Position',[680 560 710 580]);
set_param([modelName '/Demux2'],'Outputs','2');
pH_Dmx = get_param([modelName '/Demux2'],'PortHandles');
add_line(modelName, pH_Sat.Outport(1), pH_Dmx.Inport(1), 'autorouting','on');

% tau = alm * V  (sin back-EMF — consistente con linealización)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_almR'],'Position',[750 545 830 575]);
set_param([modelName '/Gain_almR'],'Gain',num2str(alm));
add_block('simulink/Math Operations/Gain',[modelName '/Gain_almL'],'Position',[750 585 830 615]);
set_param([modelName '/Gain_almL'],'Gain',num2str(alm));
pH_GaR = get_param([modelName '/Gain_almR'],'PortHandles');
pH_GaL = get_param([modelName '/Gain_almL'],'PortHandles');
add_line(modelName, pH_Dmx.Outport(1), pH_GaR.Inport(1), 'autorouting','on');
add_line(modelName, pH_Dmx.Outport(2), pH_GaL.Inport(1), 'autorouting','on');

% SL2PS → Joint_phi_R/LConn2, Joint_phi_L/LConn2
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauR'],'Position',[870 545 960 575]);
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauL'],'Position',[870 585 960 615]);
pH_SL_R = get_param([modelName '/SL2PS_tauR'],'PortHandles');
pH_SL_L = get_param([modelName '/SL2PS_tauL'],'PortHandles');
add_line(modelName, pH_GaR.Outport(1), pH_SL_R.Inport(1), 'autorouting','on');
add_line(modelName, pH_GaL.Outport(1), pH_SL_L.Inport(1), 'autorouting','on');
add_line(modelName,'SL2PS_tauR/RConn1','Joint_phi_R/LConn2');
add_line(modelName,'SL2PS_tauL/RConn1','Joint_phi_L/LConn2');

fprintf('[LQR] Cadena de control OK — tau = alm*V (sin back-EMF)\n');

%% ── 7. FUERZA DIRECTA A Joint_x: F_x = alm*(VR+VL)/r ──────────────────────
%  La fuerza de tracción neta que ejercen las ruedas sobre el suelo es:
%  F_x = (tau_R + tau_L) / r = alm*(VR+VL) / r
%  Se suma VR+VL del Demux, se multiplica por alm/r, y se conecta a Joint_x/LConn2.
%  Esto cierra el lazo físico sin restricción cinemática ni back-EMF.

% Suma VR + VL
add_block('simulink/Math Operations/Sum',[modelName '/Sum_VV'],'Position',[750 680 790 720]);
set_param([modelName '/Sum_VV'],'Inputs','++');
pH_SVV = get_param([modelName '/Sum_VV'],'PortHandles');
add_line(modelName, pH_Dmx.Outport(1), pH_SVV.Inport(1), 'autorouting','on');
add_line(modelName, pH_Dmx.Outport(2), pH_SVV.Inport(2), 'autorouting','on');

% Gain: alm/r
add_block('simulink/Math Operations/Gain',[modelName '/Gain_Fx'],'Position',[820 680 900 710]);
set_param([modelName '/Gain_Fx'],'Gain',num2str(alm/r));
pH_GFx = get_param([modelName '/Gain_Fx'],'PortHandles');
add_line(modelName, pH_SVV.Outport(1), pH_GFx.Inport(1), 'autorouting','on');

% SL2PS → Joint_x/LConn2
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fx'],'Position',[920 680 1010 710]);
pH_SL_Fx = get_param([modelName '/SL2PS_Fx'],'PortHandles');
add_line(modelName, pH_GFx.Outport(1), pH_SL_Fx.Inport(1), 'autorouting','on');
add_line(modelName,'SL2PS_Fx/RConn1','Joint_x/LConn2');

fprintf('[Force] F_x = alm*(VR+VL)/r → Joint_x OK\n');

%% ── 8. LOGGING ───────────────────────────────────────────────────────────
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_state'],'Position',[430 650 530 680]);
set_param([modelName '/ToWS_state'],'VariableName','state_log','SaveFormat','Array','SampleTime','-1');
pH_TWS = get_param([modelName '/ToWS_state'],'PortHandles');
add_line(modelName, pH_Mux.Outport(1), pH_TWS.Inport(1), 'autorouting','on');

%% ── 9. GUARDAR Y SIMULAR ─────────────────────────────────────────────────
save_system(modelName,[modelName '.slx']);

fprintf('[Sim] Simulando %g s...\n', t_sim);
try
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimscapeLogType','all','SimscapeLogName','simlog');
    out   = sim(simIn);
    fprintf('[Sim] OK  t_final=%.3f s\n', out.tout(end));
catch e
    fprintf('[Sim] FALLO: %s\n', e.message);
    return
end

%% ── 10. DIAGNÓSTICO + GRÁFICA ────────────────────────────────────────────
try
    t_sm     = out.simlog.Joint_theta.Rz.q.series.time;
    theta_sm = out.simlog.Joint_theta.Rz.q.series.values('rad');
    dth0     = out.simlog.Joint_theta.Rz.w.series.values('rad/s');
    fprintf('\n[Diagnóstico] theta(0) = %.4f rad (esperado +%.4f)  |  dtheta(0) = %.4f rad/s\n', ...
        theta_sm(1), theta0, dth0(1));
    if abs(theta_sm(1) + theta0) < 0.01
        fprintf('  >> ADVERTENCIA: theta viene con signo invertido\n');
    elseif abs(theta_sm(1)) < 1e-3
        fprintf('  >> ADVERTENCIA: CI theta ≈ 0, no se aplicó correctamente\n');
    else
        fprintf('  >> CI theta OK\n');
    end
    fprintf('[Simlog] theta_final = %.3f°\n', rad2deg(theta_sm(end)));

    figure('Name','Segway LQR v2 | Restricción Cinemática x = r·phi_R','Position',[100 100 900 420]);
    plot(t_ode, rad2deg(X_ode(:,1)), 'r--', 'LineWidth',2, 'DisplayName','ODE No Lineal + LQR');
    hold on;
    plot(t_sm,  rad2deg(theta_sm),   'b',   'LineWidth',2, 'DisplayName','Simscape 3D + LQR (cerrado)');
    yline(0,'--k','Equilibrio','LabelHorizontalAlignment','left');
    grid on;
    xlabel('t [s]');
    ylabel('\theta [°]');
    title('LQR lazo cerrado  |  x = r \cdot \phi_R  |  \theta_0 = 5°');
    legend('Location','best');
catch e
    fprintf('[Gráfica] FALLO: %s\n', e.message);
    fprintf('  Campos disponibles en out: ');
    disp(fieldnames(out));
end

fprintf('\n=== LISTO ===\n');

%% ── FUNCIÓN ODE NO LINEAL + LQR (4 estados) ─────────────────────────────
function dX = ode_nl_lqr(~, X, K, V_sat, M, m, r, l, g, Icy, Iw, alm)
    U      = max(min(-K*X, V_sat), -V_sat);
    theta  = X(1);
    dtheta = X(2);
    dx     = X(4);
    VR = U(1);  VL = U(2);

    M11   = Icy + M*l^2;
    M12   = M*l*cos(theta);
    M22   = M + 2*m + 2*Iw/r^2;
    det_c = M11*M22 - M12^2;

    tau_sum = alm*(VR + VL);
    F1 = M*g*l*sin(theta) - tau_sum;
    F2 = tau_sum/r + M*l*dtheta^2*sin(theta);   % incluye término centrípeto

    ddtheta = (M22*F1 - M12*F2) / det_c;
    ddx     = (-M12*F1 + M11*F2) / det_c;

    dX = [dtheta; ddtheta; dx; ddx];
end