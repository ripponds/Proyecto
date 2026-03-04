%% ========================================================================
%  segway_simscape.m — Modelo Simscape Multibody del Segway
%  Construcción automática del modelo 3D físico — Lazo Abierto
%  MATLAB 2025b | Simscape Multibody
% =========================================================================
%
%  Árbol cinemático:
%
%  World ──► RT_AxleHeight ──► RT_PrismaticAlign ──► Joint_x (Prismatic, X)
%              ──► Joint_alpha (Revolute, Z)
%                    ├──► RT_ThetaAxis ──► Joint_theta (Revolute, Y) ──► RT_BodyCM ──► Brick Solid
%                    ├──► RT_WheelR ──► RT_WheelR_Axis ──► Joint_phi_R ──► Cylindrical Solid
%                    └──► RT_WheelL ──► RT_WheelL_Axis ──► Joint_phi_L ──► Cylindrical Solid
%
%  Convención de puertos (Simscape Multibody 2025b):
%    LConn(1) = marco base (entrada/parent)
%    RConn(1) = marco follower (salida/child)
%    Solids y World Frame solo tienen RConn(1)
%
%  Lazo abierto: tau_R = tau_L = 0 [N·m]  — placeholder para controlador v2
%  [v1] Sin rolling constraint ni contacto dinámico suelo-rueda
% =========================================================================

clear; clc; close all;

%% ── 1. PARÁMETROS FÍSICOS ───────────────────────────────────────────────
M   = 80;     % [kg]    Masa cuerpo + persona
m   = 2;      % [kg]    Masa por rueda
r   = 0.20;   % [m]     Radio de rueda
d   = 0.60;   % [m]     Separación entre ruedas (vía)
l   = 0.90;   % [m]     Distancia eje ruedas → CM del cuerpo

Icy = 10;     % [kg·m²] Inercia cabeceo (eje Y)
Icz = 12;     % [kg·m²] Inercia giro    (eje Z)
Icx = 12;     % [kg·m²] Inercia alabeo  (eje X)
Iw  = 0.08;   % [kg·m²] Inercia axial rueda   (spin, eje Y)
Iwz = 0.04;   % [kg·m²] Inercia transversal rueda

body_W = 0.40;  % [m] ancho  (Y)
body_D = 0.20;  % [m] prof   (X)
body_H = 1.60;  % [m] alto   (Z)

theta0_deg = 5;
theta0     = theta0_deg * pi/180;   % [rad] CI cabeceo

% PLACEHOLDER v2: load('segway_modelo_resultados.mat','K')
% u = -K*[theta;dtheta;x;dx;alpha;dalpha];  tau_R=u(1); tau_L=u(2);

%% ── 2. CREAR MODELO ─────────────────────────────────────────────────────
modelName = 'segway_sm';
fprintf('Construyendo %s ...\n', modelName);
if bdIsLoaded(modelName), close_system(modelName, 0); end
new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver','ode23t', 'StopTime','3', ...
    'RelTol','1e-4', 'AbsTol','1e-6');

%% ── 3. UTILIDADES SIMSCAPE ──────────────────────────────────────────────
add_block('nesl_utility/Solver Configuration', ...
    [modelName '/Solver_Config'], 'Position', [30 30 180 60]);

add_block('sm_lib/Utilities/Mechanism Configuration', ...
    [modelName '/Mech_Config'], 'Position', [30 90 220 120]);
set_param([modelName '/Mech_Config'], 'GravityVector', '[0; 0; -9.81]');

add_block('sm_lib/Frames and Transforms/World Frame', ...
    [modelName '/World'], 'Position', [30 160 130 190]);

pH_world  = get_param([modelName '/World'],        'PortHandles');
pH_solver = get_param([modelName '/Solver_Config'],'PortHandles');
pH_mech   = get_param([modelName '/Mech_Config'],  'PortHandles');
add_line(modelName, pH_world.RConn(1),  pH_solver.RConn(1), 'autorouting','on');
add_line(modelName, pH_world.RConn(1),  pH_mech.RConn(1),   'autorouting','on');

%% ── 4. RT_AxleHeight — eleva el marco al eje de ruedas (z = r) ──────────
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_AxleHeight'], 'Position', [220 160 340 190]);
set_param([modelName '/RT_AxleHeight'], ...
    'RotationMethod',            'None', ...
    'TranslationMethod',         'StandardAxis', ...
    'TranslationStandardAxis',   '+Z', ...
    'TranslationStandardOffset', num2str(r));

pH_RT_Axle = get_param([modelName '/RT_AxleHeight'], 'PortHandles');
add_line(modelName, pH_world.RConn(1), pH_RT_Axle.LConn(1), 'autorouting','on');

%% ── 5. RT_PrismaticAlign — rota +90°Y: eje prisma Z→X ───────────────────
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_PrismaticAlign'], 'Position', [400 160 520 190]);
set_param([modelName '/RT_PrismaticAlign'], ...
    'RotationMethod',       'StandardAxis', ...
    'RotationStandardAxis', '+Y', ...
    'RotationAngle',        '90', ...
    'TranslationMethod',    'None');

pH_RT_Prism = get_param([modelName '/RT_PrismaticAlign'], 'PortHandles');
add_line(modelName, pH_RT_Axle.RConn(1), pH_RT_Prism.LConn(1), 'autorouting','on');

%% ── 6. Joint_x — avance lineal x [m] ────────────────────────────────────
add_block('sm_lib/Joints/Prismatic Joint', ...
    [modelName '/Joint_x'], 'Position', [580 145 700 205]);
set_param([modelName '/Joint_x'], ...
    'TorqueActuationMode', 'NoTorque', ...
    'MotionActuationMode', 'ComputedMotion', ...
    'SensePosition',       'on',  ...   % Outport(1) = x [m]
    'SenseVelocity',       'on');       % Outport(2) = dx [m/s]

pH_Joint_x = get_param([modelName '/Joint_x'], 'PortHandles');
add_line(modelName, pH_RT_Prism.RConn(1), pH_Joint_x.LConn(1), 'autorouting','on');

%% ── 7. Joint_alpha — giro α [rad], eje Z ────────────────────────────────
add_block('sm_lib/Joints/Revolute Joint', ...
    [modelName '/Joint_alpha'], 'Position', [760 145 880 205]);
set_param([modelName '/Joint_alpha'], ...
    'TorqueActuationMode', 'NoTorque', ...
    'MotionActuationMode', 'ComputedMotion', ...
    'SensePosition',       'on',  ...   % Outport(1) = alpha [rad]
    'SenseVelocity',       'on');       % Outport(2) = dalpha [rad/s]

pH_Joint_alpha = get_param([modelName '/Joint_alpha'], 'PortHandles');
add_line(modelName, pH_Joint_x.RConn(1), pH_Joint_alpha.LConn(1), 'autorouting','on');

%% ── 8. Rama cabeceo ──────────────────────────────────────────────────────

% RT_ThetaAxis — rota -90°X: eje revoluta Z→Y (cabeceo)
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_ThetaAxis'], 'Position', [960 60 1080 90]);
set_param([modelName '/RT_ThetaAxis'], ...
    'RotationMethod',       'StandardAxis', ...
    'RotationStandardAxis', '+X', ...
    'RotationAngle',        '-90', ...
    'TranslationMethod',    'None');

pH_RT_ThetaAxis = get_param([modelName '/RT_ThetaAxis'], 'PortHandles');
add_line(modelName, pH_Joint_alpha.RConn(1), pH_RT_ThetaAxis.LConn(1), 'autorouting','on');

% Joint_theta — cabeceo θ [rad], eje Y (tras rotación)
add_block('sm_lib/Joints/Revolute Joint', ...
    [modelName '/Joint_theta'], 'Position', [1140 45 1260 105]);
% CI primero — cambia modo interno de la junta antes de fijar actuación
set_param([modelName '/Joint_theta'], ...
    'PositionTargetSpecify',  'on', ...
    'PositionTargetValue',    num2str(theta0), ...   % [rad] = 5°
    'PositionTargetPriority', 'High');
% Actuación y sensing en llamada separada
set_param([modelName '/Joint_theta'], ...
    'TorqueActuationMode', 'NoTorque', ...
    'MotionActuationMode', 'ComputedMotion', ...
    'SensePosition',       'on',  ...   % Outport(1) = theta [rad]
    'SenseVelocity',       'on');       % Outport(2) = dtheta [rad/s]

pH_Joint_theta = get_param([modelName '/Joint_theta'], 'PortHandles');
add_line(modelName, pH_RT_ThetaAxis.RConn(1), pH_Joint_theta.LConn(1), 'autorouting','on');

% RT_BodyCM — sube l metros al CM del cuerpo
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_BodyCM'], 'Position', [1320 45 1440 105]);
set_param([modelName '/RT_BodyCM'], ...
    'RotationMethod',            'None', ...
    'TranslationMethod',         'StandardAxis', ...
    'TranslationStandardAxis',   '+Z', ...
    'TranslationStandardOffset', num2str(l));

pH_RT_BodyCM = get_param([modelName '/RT_BodyCM'], 'PortHandles');
add_line(modelName, pH_Joint_theta.RConn(1), pH_RT_BodyCM.LConn(1), 'autorouting','on');

% Body_Solid — cuerpo M=80 kg, caja
add_block('sm_lib/Body Elements/Brick Solid', ...
    [modelName '/Body_Solid'], 'Position', [1500 45 1620 105]);
set_param([modelName '/Body_Solid'], ...
    'BrickDimensions',    mat2str([body_D, body_W, body_H]), ...  % [m] X Y Z
    'InertiaType',        'Custom', ...
    'Mass',               num2str(M), ...
    'CenterOfMass',       '[0 0 0]', ...
    'MomentsOfInertia',   mat2str([Icx, Icy, Icz]), ...   % [kg·m²] Ixx Iyy Izz
    'ProductsOfInertia',  '[0 0 0]', ...                  % [kg·m²] Ixy Ixz Iyz
    'GraphicDiffuseColor','[0.2 0.5 0.8]');

pH_Body = get_param([modelName '/Body_Solid'], 'PortHandles');
add_line(modelName, pH_RT_BodyCM.RConn(1), pH_Body.RConn(1), 'autorouting','on');

%% ── 9. Rama rueda derecha ────────────────────────────────────────────────

% RT_WheelR — desplaza +d/2 en Y
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_WheelR'], 'Position', [960 160 1080 190]);
set_param([modelName '/RT_WheelR'], ...
    'RotationMethod',            'None', ...
    'TranslationMethod',         'StandardAxis', ...
    'TranslationStandardAxis',   '+Y', ...
    'TranslationStandardOffset', num2str(d/2));

pH_RT_WR = get_param([modelName '/RT_WheelR'], 'PortHandles');
add_line(modelName, pH_Joint_alpha.RConn(1), pH_RT_WR.LConn(1), 'autorouting','on');

% RT_WheelR_Axis — rota +90°X: cilindro eje Y
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_WheelR_Axis'], 'Position', [1140 160 1260 190]);
set_param([modelName '/RT_WheelR_Axis'], ...
    'RotationMethod',       'StandardAxis', ...
    'RotationStandardAxis', '+X', ...
    'RotationAngle',        '90', ...
    'TranslationMethod',    'None');

pH_RT_WR_ax = get_param([modelName '/RT_WheelR_Axis'], 'PortHandles');
add_line(modelName, pH_RT_WR.RConn(1), pH_RT_WR_ax.LConn(1), 'autorouting','on');

% Joint_phi_R — spin rueda derecha (libre en lazo abierto, tau=0)
% [v2] cambiar a 'InputTorque' y conectar señal del controlador
add_block('sm_lib/Joints/Revolute Joint', ...
    [modelName '/Joint_phi_R'], 'Position', [1320 145 1440 205]);
set_param([modelName '/Joint_phi_R'], ...
    'TorqueActuationMode', 'NoTorque', ...
    'MotionActuationMode', 'ComputedMotion', ...
    'SensePosition',       'off', ...
    'SenseVelocity',       'off');

pH_Joint_phiR = get_param([modelName '/Joint_phi_R'], 'PortHandles');
add_line(modelName, pH_RT_WR_ax.RConn(1), pH_Joint_phiR.LConn(1), 'autorouting','on');

% Wheel_R — cilindro m=2 kg
add_block('sm_lib/Body Elements/Cylindrical Solid', ...
    [modelName '/Wheel_R'], 'Position', [1500 145 1620 205]);
set_param([modelName '/Wheel_R'], ...
    'CylinderRadius',     num2str(r), ...
    'CylinderLength',     num2str(0.08), ...
    'InertiaType',        'Custom', ...
    'Mass',               num2str(m), ...
    'CenterOfMass',       '[0 0 0]', ...
    'MomentsOfInertia',   mat2str([Iwz, Iw, Iwz]), ...
    'ProductsOfInertia',  '[0 0 0]', ...
    'GraphicDiffuseColor','[0.15 0.15 0.15]');

pH_WheelR = get_param([modelName '/Wheel_R'], 'PortHandles');
add_line(modelName, pH_Joint_phiR.RConn(1), pH_WheelR.RConn(1), 'autorouting','on');

%% ── 10. Rama rueda izquierda ─────────────────────────────────────────────

% RT_WheelL — desplaza -d/2 en Y
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_WheelL'], 'Position', [960 250 1080 280]);
set_param([modelName '/RT_WheelL'], ...
    'RotationMethod',            'None', ...
    'TranslationMethod',         'StandardAxis', ...
    'TranslationStandardAxis',   '-Y', ...
    'TranslationStandardOffset', num2str(d/2));

pH_RT_WL = get_param([modelName '/RT_WheelL'], 'PortHandles');
add_line(modelName, pH_Joint_alpha.RConn(1), pH_RT_WL.LConn(1), 'autorouting','on');

% RT_WheelL_Axis — rota +90°X
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [modelName '/RT_WheelL_Axis'], 'Position', [1140 250 1260 280]);
set_param([modelName '/RT_WheelL_Axis'], ...
    'RotationMethod',       'StandardAxis', ...
    'RotationStandardAxis', '+X', ...
    'RotationAngle',        '90', ...
    'TranslationMethod',    'None');

pH_RT_WL_ax = get_param([modelName '/RT_WheelL_Axis'], 'PortHandles');
add_line(modelName, pH_RT_WL.RConn(1), pH_RT_WL_ax.LConn(1), 'autorouting','on');

% Joint_phi_L — spin rueda izquierda (libre en lazo abierto, tau=0)
% [v2] cambiar a 'InputTorque' y conectar señal del controlador
add_block('sm_lib/Joints/Revolute Joint', ...
    [modelName '/Joint_phi_L'], 'Position', [1320 235 1440 295]);
set_param([modelName '/Joint_phi_L'], ...
    'TorqueActuationMode', 'NoTorque', ...
    'MotionActuationMode', 'ComputedMotion', ...
    'SensePosition',       'off', ...
    'SenseVelocity',       'off');

pH_Joint_phiL = get_param([modelName '/Joint_phi_L'], 'PortHandles');
add_line(modelName, pH_RT_WL_ax.RConn(1), pH_Joint_phiL.LConn(1), 'autorouting','on');

% Wheel_L — cilindro m=2 kg
add_block('sm_lib/Body Elements/Cylindrical Solid', ...
    [modelName '/Wheel_L'], 'Position', [1500 235 1620 295]);
set_param([modelName '/Wheel_L'], ...
    'CylinderRadius',     num2str(r), ...
    'CylinderLength',     num2str(0.08), ...
    'InertiaType',        'Custom', ...
    'Mass',               num2str(m), ...
    'CenterOfMass',       '[0 0 0]', ...
    'MomentsOfInertia',   mat2str([Iwz, Iw, Iwz]), ...
    'ProductsOfInertia',  '[0 0 0]', ...
    'GraphicDiffuseColor','[0.15 0.15 0.15]');

pH_WheelL = get_param([modelName '/Wheel_L'], 'PortHandles');
add_line(modelName, pH_Joint_phiL.RConn(1), pH_WheelL.RConn(1), 'autorouting','on');

%% ── 11. ACTUADORES (lazo abierto — juntas libres, tau = 0 implícito) ─────
% ┌──────────────────────────────────────────────────────────────────────┐
% │  PLACEHOLDER v2: activar InputTorque en Joint_phi_R y Joint_phi_L   │
% │  y conectar:  tau_R = u(1),  tau_L = u(2)  del controlador LQR      │
% └──────────────────────────────────────────────────────────────────────┘
% En lazo abierto NoTorque = tau implícitamente cero, sin bloque externo

%% ── 12. SENSORES → WORKSPACE ─────────────────────────────────────────────
% Forzar actualización del diagrama para que los Outports de sensing existan
set_param(modelName, 'SimulationCommand', 'update');

% theta [rad] — cabeceo (Outport 1 = posición)
add_block('simulink/Sinks/To Workspace', [modelName '/out_theta'], ...
    'Position', [1460 -10 1580 20]);
set_param([modelName '/out_theta'], 'VariableName','sm_theta', 'SaveFormat','Array');
add_line(modelName, 'Joint_theta/1', 'out_theta/1', 'autorouting','on');

% x [m] — avance (Outport 1 = posición)
add_block('simulink/Sinks/To Workspace', [modelName '/out_x'], ...
    'Position', [820 110 940 140]);
set_param([modelName '/out_x'], 'VariableName','sm_x', 'SaveFormat','Array');
add_line(modelName, 'Joint_x/1', 'out_x/1', 'autorouting','on');

% alpha [rad] — giro (Outport 1 = posición)
add_block('simulink/Sinks/To Workspace', [modelName '/out_alpha'], ...
    'Position', [820 160 940 190]);
set_param([modelName '/out_alpha'], 'VariableName','sm_alpha', 'SaveFormat','Array');
add_line(modelName, 'Joint_alpha/1', 'out_alpha/1', 'autorouting','on');

%% ── 13. GUARDAR Y SIMULAR ────────────────────────────────────────────────
fprintf('Guardando modelo...\n');
save_system(modelName, [modelName '.slx']);

fprintf('Simulando %s s  (lazo abierto, theta0 = %d°)...\n', ...
    get_param(modelName,'StopTime'), theta0_deg);
out = sim(modelName);

t     = out.tout;
theta = out.sm_theta;
x_pos = out.sm_x;
alpha = out.sm_alpha;

%% ── 14. GRÁFICAS ──────────────────────────────────────────────────────────
figure('Name','Simscape Multibody — Lazo Abierto','Position',[80 80 1100 500]);

subplot(1,3,1);
plot(t, rad2deg(theta), 'b', 'LineWidth', 1.8);
yline(0,'--k','Equilibrio'); grid on;
xlabel('t  [s]'); ylabel('Cabeceo  [°]');
title(sprintf('Cabeceo libre  (\\theta_0 = %d°)', theta0_deg));

subplot(1,3,2);
plot(t, x_pos, 'g', 'LineWidth', 1.8);
yline(0,'--k'); grid on;
xlabel('t  [s]'); ylabel('Avance  [m]');
title('Posición de avance  [v1: sin rolling constraint]');

subplot(1,3,3);
plot(t, rad2deg(alpha), 'r', 'LineWidth', 1.8);
yline(0,'--k'); grid on;
xlabel('t  [s]'); ylabel('Giro  [°]');
title('Giro  (\alpha_0 = 0°)');

sgtitle(sprintf('Simscape Multibody — Lazo Abierto  |  \\theta_0 = %d°  |  \\tau = 0 N·m', ...
    theta0_deg), 'FontSize', 12);

fprintf('\n=== SIMULACIÓN COMPLETADA ===\n');
fprintf('theta final = %.2f°\n', rad2deg(theta(end)));
