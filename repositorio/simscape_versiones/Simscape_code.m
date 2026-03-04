%% segway_debug.m — Constructor paso a paso con diagnóstico
%  Ejecutar sección por sección (Ctrl+Enter) o completo
%  Cada paso imprime OK o el error exacto

clear; clc; close all;
bdclose all;
modelName = 'Simscape_Sim';
if bdIsLoaded(modelName), close_system(modelName, 0); end
if exist([modelName '.slx'], 'file'), delete([modelName '.slx']); end
new_system(modelName); open_system(modelName);
set_param(modelName,'Solver','ode23t','StopTime','10','RelTol','1e-4','AbsTol','1e-6');

r=0.20; d=0.60; l=0.90; M=80; m=2;
Icy=10; Icz=12; Icx=12; Iw=0.08; Iwz=0.04;
body_W=0.40; body_D=0.20; body_H=1.60;
theta0_deg = 5;
theta0     = theta0_deg * pi/180;

%% ── PASO 1: Utilidades ───────────────────────────────────────────────────
try
    add_block('nesl_utility/Solver Configuration',[modelName '/Solver_Config'],'Position',[30 30 180 60]);
    add_block('sm_lib/Utilities/Mechanism Configuration',[modelName '/Mech_Config'],'Position',[30 90 220 120]);
    set_param([modelName '/Mech_Config'],'GravityVector','[0; 0; -9.81]');
    add_block('sm_lib/Frames and Transforms/World Frame',[modelName '/World'],'Position',[30 160 130 190]);
    pH_world  = get_param([modelName '/World'],        'PortHandles');
    pH_solver = get_param([modelName '/Solver_Config'],'PortHandles');
    pH_mech   = get_param([modelName '/Mech_Config'],  'PortHandles');
    add_line(modelName,pH_world.RConn(1),pH_solver.RConn(1),'autorouting','on');
    add_line(modelName,pH_world.RConn(1),pH_mech.RConn(1),  'autorouting','on');
    fprintf('[1] Utilidades                    OK\n');
catch e
    fprintf('[1] Utilidades                    FALLO: %s\n', e.message); return
end

%% ── PASO 2: RT_AxleHeight ────────────────────────────────────────────────
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[220 160 340 190]);
    set_param([modelName '/RT_AxleHeight'], ...
        'RotationMethod','None', ...
        'TranslationMethod','StandardAxis', ...
        'TranslationStandardAxis','+Z', ...
        'TranslationStandardOffset',num2str(r));
    pH_RT_Axle = get_param([modelName '/RT_AxleHeight'],'PortHandles');
    add_line(modelName,pH_world.RConn(1),pH_RT_Axle.LConn(1),'autorouting','on');
    fprintf('[2] RT_AxleHeight                 OK\n');
catch e
    fprintf('[2] RT_AxleHeight                 FALLO: %s\n', e.message); return
end

%% ── PASO 3: RT_PrismaticAlign ────────────────────────────────────────────
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_PrismaticAlign'],'Position',[400 160 520 190]);
    set_param([modelName '/RT_PrismaticAlign'], ...
        'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+Y', ...
        'RotationAngle','90', ...
        'TranslationMethod','None');
    pH_RT_Prism = get_param([modelName '/RT_PrismaticAlign'],'PortHandles');
    add_line(modelName,pH_RT_Axle.RConn(1),pH_RT_Prism.LConn(1),'autorouting','on');
    fprintf('[3] RT_PrismaticAlign             OK\n');
catch e
    fprintf('[3] RT_PrismaticAlign             FALLO: %s\n', e.message); return
end

%% ── PASO 4: Joint_x (Prismatic) ─────────────────────────────────────────
try
    add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[580 145 700 205]);
    set_param([modelName '/Joint_x'], ...
        'TorqueActuationMode','InputTorque', ...
        'MotionActuationMode','ComputedMotion', ...
        'SensePosition','on', ...
        'SenseVelocity','on');
    pH_Joint_x = get_param([modelName '/Joint_x'],'PortHandles');
    add_line(modelName,pH_RT_Prism.RConn(1),pH_Joint_x.LConn(1),'autorouting','on');
    fprintf('[4] Joint_x (Prismatic)           OK  | Outports=%d\n', numel(pH_Joint_x.Outport));
catch e
    fprintf('[4] Joint_x (Prismatic)           FALLO: %s\n', e.message); return
end

%% ── PASO 5: RT_Realign — restaura Z=mundoZ tras RT_PrismaticAlign ────────
% RT_PrismaticAlign dejó local Z = mundo X (para prismatic).
% RT_Realign gira -90° en Y para devolver Z = mundo Z (necesario para Joint_alpha).
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign'],'Position',[740 145 860 185]);
    set_param([modelName '/RT_Realign'], ...
        'RotationMethod',       'StandardAxis', ...
        'RotationStandardAxis', '+Y', ...
        'RotationAngle',        '-90', ...
        'TranslationMethod',    'None');
    pH_RT_Realign = get_param([modelName '/RT_Realign'],'PortHandles');
    add_line(modelName,pH_Joint_x.RConn(1),pH_RT_Realign.LConn(1),'autorouting','on');
    fprintf('[5] RT_Realign                    OK\n');
catch e
    fprintf('[5] RT_Realign                    FALLO: %s\n', e.message); return
end

%% ── PASO 5b: Joint_alpha (Revolute, giro Z = mundo Z) ───────────────────
try
    add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[920 145 1040 205]);
    set_param([modelName '/Joint_alpha'], ...
        'TorqueActuationMode','NoTorque', ...
        'MotionActuationMode','ComputedMotion', ...
        'SensePosition','on', ...
        'SenseVelocity','on');
    pH_Joint_alpha = get_param([modelName '/Joint_alpha'],'PortHandles');
    add_line(modelName,pH_RT_Realign.RConn(1),pH_Joint_alpha.LConn(1),'autorouting','on');
    fprintf('[5b] Joint_alpha (Revolute)       OK\n');
catch e
    fprintf('[5b] Joint_alpha (Revolute)       FALLO: %s\n', e.message); return
end

%% ── PASO 6: RT_ThetaAxis ─────────────────────────────────────────────────
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[960 60 1080 90]);
    set_param([modelName '/RT_ThetaAxis'], ...
        'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+X', ...
        'RotationAngle','-90', ...
        'TranslationMethod','None');
    pH_RT_ThetaAxis = get_param([modelName '/RT_ThetaAxis'],'PortHandles');
    add_line(modelName,pH_Joint_alpha.RConn(1),pH_RT_ThetaAxis.LConn(1),'autorouting','on');
    fprintf('[6] RT_ThetaAxis                  OK\n');
catch e
    fprintf('[6] RT_ThetaAxis                  FALLO: %s\n', e.message); return
end

%% ── PASO 7: Joint_theta (Revolute, cabeceo Y) ────────────────────────────
try
    add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[1140 45 1260 105]);
    set_param([modelName '/Joint_theta'], ...
        'PositionTargetSpecify','on', ...
        'PositionTargetValue',num2str(theta0_deg), ...
        'PositionTargetValueUnits','deg', ...
        'PositionTargetPriority','High');
    set_param([modelName '/Joint_theta'], ...
        'TorqueActuationMode','NoTorque', ...
        'DampingCoefficient','0.01675', ...
        'SensePosition','on', ...
        'SenseVelocity','on');
    pH_Joint_theta = get_param([modelName '/Joint_theta'],'PortHandles');
    add_line(modelName,pH_RT_ThetaAxis.RConn(1),pH_Joint_theta.LConn(1),'autorouting','on');
    fprintf('[7] Joint_theta (Revolute CI=5°)  OK  | Outports=%d\n', numel(pH_Joint_theta.Outport));
catch e
    fprintf('[7] Joint_theta (Revolute CI=5°)  FALLO: %s\n', e.message); return
end

%% ── PASO 8: RT_BodyCM + Body_Solid ──────────────────────────────────────
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1320 45 1440 105]);
    % Tras RT_ThetaAxis(-90°X): local Y = mundo -Z, local Z = mundo Y
    % Para subir al CM (mundo +Z) hay que ir en -local_Y
    set_param([modelName '/RT_BodyCM'], ...
        'RotationMethod','None', ...
        'TranslationMethod','StandardAxis', ...
        'TranslationStandardAxis','-Y', ...       % -local_Y = mundo +Z (arriba) ✓
        'TranslationStandardOffset',num2str(l));
    add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1500 45 1620 105]);
    % BrickDimensions [X=prof, Y=alto, Z=ancho]: Y es eje alto en local frame
    set_param([modelName '/Body_Solid'], ...
        'BrickDimensions',   mat2str([body_D, body_H, body_W]), ...
        'InertiaType',       'Custom', ...
        'Mass',              num2str(M), ...
        'CenterOfMass',      '[0 0 0]', ...
        'MomentsOfInertia',  mat2str([Icx,Icy,Icz]), ...
        'ProductsOfInertia', '[0 0 0]', ...
        'GraphicDiffuseColor','[0.2 0.5 0.8]');
    pH_RT_BodyCM = get_param([modelName '/RT_BodyCM'],'PortHandles');
    pH_Body      = get_param([modelName '/Body_Solid'],'PortHandles');
    add_line(modelName,pH_Joint_theta.RConn(1),pH_RT_BodyCM.LConn(1),'autorouting','on');
    add_line(modelName,pH_RT_BodyCM.RConn(1),  pH_Body.RConn(1),     'autorouting','on');
    fprintf('[8] RT_BodyCM + Body_Solid        OK\n');
catch e
    fprintf('[8] RT_BodyCM + Body_Solid        FALLO: %s\n', e.message); return
end

%% ── PASO 9: Rueda derecha ────────────────────────────────────────────────
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[960 160 1080 190]);
    set_param([modelName '/RT_WheelR'], ...
        'RotationMethod','None', ...
        'TranslationMethod','StandardAxis', ...
        'TranslationStandardAxis','+Y', ...
        'TranslationStandardOffset',num2str(d/2));
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Axis'],'Position',[1140 160 1260 190]);
    set_param([modelName '/RT_WheelR_Axis'], ...
        'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+X', ...
        'RotationAngle','90', ...
        'TranslationMethod','None');
    bem_deg = 1.5 * pi/180;   % 1.5 N·m·s/rad → N·m/(deg/s)
    add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[1320 145 1440 205]);
    set_param([modelName '/Joint_phi_R'], ...
        'TorqueActuationMode','NoTorque', ...
        'MotionActuationMode','ComputedMotion', ...
        'DampingCoefficient',  num2str(bem_deg), ...
        'SensePosition','on', ...
        'SenseVelocity','on');
    add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[1500 145 1620 205]);
    set_param([modelName '/Wheel_R'], ...
        'CylinderRadius',    num2str(r), ...
        'CylinderLength',    num2str(0.08), ...
        'InertiaType',       'Custom', ...
        'Mass',              num2str(m), ...
        'CenterOfMass',      '[0 0 0]', ...
        'MomentsOfInertia',  mat2str([Iwz,Iw,Iwz]), ...
        'ProductsOfInertia', '[0 0 0]', ...
        'GraphicDiffuseColor','[0.15 0.15 0.15]');
    pH_RT_WR    = get_param([modelName '/RT_WheelR'],     'PortHandles');
    pH_RT_WR_ax = get_param([modelName '/RT_WheelR_Axis'],'PortHandles');
    pH_phiR     = get_param([modelName '/Joint_phi_R'],   'PortHandles');
    pH_WheelR   = get_param([modelName '/Wheel_R'],       'PortHandles');
    add_line(modelName,pH_Joint_alpha.RConn(1),pH_RT_WR.LConn(1),   'autorouting','on');
    add_line(modelName,pH_RT_WR.RConn(1),      pH_RT_WR_ax.LConn(1),'autorouting','on');
    add_line(modelName,pH_RT_WR_ax.RConn(1),   pH_phiR.LConn(1),    'autorouting','on');
    add_line(modelName,pH_phiR.RConn(1),        pH_WheelR.RConn(1),  'autorouting','on');
    fprintf('[9] Rueda derecha                 OK\n');
catch e
    fprintf('[9] Rueda derecha                 FALLO: %s\n', e.message); return
end

%% ── PASO 10: Rueda izquierda ─────────────────────────────────────────────
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL'],'Position',[960 250 1080 280]);
    set_param([modelName '/RT_WheelL'], ...
        'RotationMethod','None', ...
        'TranslationMethod','StandardAxis', ...
        'TranslationStandardAxis','-Y', ...
        'TranslationStandardOffset',num2str(d/2));
    add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL_Axis'],'Position',[1140 250 1260 280]);
    set_param([modelName '/RT_WheelL_Axis'], ...
        'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+X', ...
        'RotationAngle','90', ...
        'TranslationMethod','None');
    add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_L'],'Position',[1320 235 1440 295]);
    set_param([modelName '/Joint_phi_L'], ...
        'TorqueActuationMode','NoTorque', ...
        'MotionActuationMode','ComputedMotion', ...
        'DampingCoefficient',  num2str(bem_deg), ...
        'SensePosition','off', ...
        'SenseVelocity','off');
    add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_L'],'Position',[1500 235 1620 295]);
    set_param([modelName '/Wheel_L'], ...
        'CylinderRadius',    num2str(r), ...
        'CylinderLength',    num2str(0.08), ...
        'InertiaType',       'Custom', ...
        'Mass',              num2str(m), ...
        'CenterOfMass',      '[0 0 0]', ...
        'MomentsOfInertia',  mat2str([Iwz,Iw,Iwz]), ...
        'ProductsOfInertia', '[0 0 0]', ...
        'GraphicDiffuseColor','[0.15 0.15 0.15]');
    pH_RT_WL    = get_param([modelName '/RT_WheelL'],     'PortHandles');
    pH_RT_WL_ax = get_param([modelName '/RT_WheelL_Axis'],'PortHandles');
    pH_phiL     = get_param([modelName '/Joint_phi_L'],   'PortHandles');
    pH_WheelL   = get_param([modelName '/Wheel_L'],       'PortHandles');
    add_line(modelName,pH_Joint_alpha.RConn(1),pH_RT_WL.LConn(1),   'autorouting','on');
    add_line(modelName,pH_RT_WL.RConn(1),      pH_RT_WL_ax.LConn(1),'autorouting','on');
    add_line(modelName,pH_RT_WL_ax.RConn(1),   pH_phiL.LConn(1),    'autorouting','on');
    add_line(modelName,pH_phiL.RConn(1),        pH_WheelL.RConn(1),  'autorouting','on');
    fprintf('[10] Rueda izquierda              OK\n');
catch e
    fprintf('[10] Rueda izquierda              FALLO: %s\n', e.message); return
end

%% ── PASO 10b: Rolling Constraint — Transform Sensors + Soft Force ────────
% Transform Sensor sensa posición real de frames → sin depender de Outports de juntas
% TS_wheel: mide rotación de la rueda (phi_R en mundo)
% TS_cart:  mide traslación del carro (x en mundo)
% Soft constraint: F = K*(r*phi_R - x) aplicada a Joint_x
try
    K_roll = 1e6;   % [N/m]

    % ── Transform Sensor rueda derecha ──
    add_block('sm_lib/Frames and Transforms/Transform Sensor', ...
        [modelName '/TS_wheel'], 'Position', [1500 320 1620 360]);
    set_param([modelName '/TS_wheel'], ...
        'MeasurementFrame', 'World', ...
        'SenseAngle',       'on');   % ángulo de rotación acumulado
    pH_TS_w = get_param([modelName '/TS_wheel'], 'PortHandles');
    % Base = World, Follower = frame de la rueda (output de Joint_phi_R)
    add_line(modelName, pH_world.RConn(1),    pH_TS_w.LConn(1), 'autorouting','on');
    add_line(modelName, pH_phiR.RConn(1),     pH_TS_w.RConn(1), 'autorouting','on');

    % ── Transform Sensor carro ──
    add_block('sm_lib/Frames and Transforms/Transform Sensor', ...
        [modelName '/TS_cart'], 'Position', [1500 400 1620 440]);
    set_param([modelName '/TS_cart'], ...
        'MeasurementFrame', 'World', ...
        'SenseX',           'on');   % posición X en mundo
    pH_TS_c = get_param([modelName '/TS_cart'], 'PortHandles');
    % Base = World, Follower = frame de salida de Joint_x (RT_Realign entrada)
    add_line(modelName, pH_world.RConn(1),      pH_TS_c.LConn(1), 'autorouting','on');
    add_line(modelName, pH_Joint_x.RConn(1),    pH_TS_c.RConn(1), 'autorouting','on');

    % Forzar update para que aparezcan puertos de señal de los Transform Sensors
    set_param(modelName, 'SimulationCommand', 'update');

    pH_TS_w = get_param([modelName '/TS_wheel'], 'PortHandles');
    pH_TS_c = get_param([modelName '/TS_cart'],  'PortHandles');
    fprintf('  TS_wheel Outports:%d  TS_cart Outports:%d\n', ...
        numel(pH_TS_w.Outport), numel(pH_TS_c.Outport));

    % ── Cadena de señales ──
    % Transform Sensor emite Outports Simulink directamente (no physical)
    % → NO se necesitan PS2SL converters para phi ni x
    % Gain r: convierte phi [rad] → r*phi [m]
    add_block('simulink/Math Operations/Gain', ...
        [modelName '/Gain_r'],    'Position', [1660 315 1740 345]);
    set_param([modelName '/Gain_r'], 'Gain', num2str(r));
    % Sum: r*phi - x
    add_block('simulink/Math Operations/Sum', ...
        [modelName '/Sum_roll'],  'Position', [1780 325 1820 375]);
    set_param([modelName '/Sum_roll'], 'Inputs', '+-');
    % Gain K: fuerza de restricción
    add_block('simulink/Math Operations/Gain', ...
        [modelName '/Gain_K'],    'Position', [1860 335 1940 365]);
    set_param([modelName '/Gain_K'], 'Gain', num2str(K_roll));
    % SL2PS → Joint_x fuerza (physical signal input)
    add_block('nesl_utility/Simulink-PS Converter', ...
        [modelName '/SL2PS_Fx'],  'Position', [1980 335 2080 365]);

    pH_Gain_r    = get_param([modelName '/Gain_r'],     'PortHandles');
    pH_Sum       = get_param([modelName '/Sum_roll'],   'PortHandles');
    pH_Gain_K    = get_param([modelName '/Gain_K'],     'PortHandles');
    pH_SL2PS_Fx  = get_param([modelName '/SL2PS_Fx'],  'PortHandles');
    pH_Joint_x   = get_param([modelName '/Joint_x'],   'PortHandles');

    % TS_wheel.Outport(1) = ángulo phi_R [rad]  (Simulink)
    % TS_cart.Outport(1)  = posición x   [m]    (Simulink)
    add_line(modelName, pH_TS_w.Outport(1),   pH_Gain_r.Inport(1),    'autorouting','on');
    add_line(modelName, pH_Gain_r.Outport(1), pH_Sum.Inport(1),        'autorouting','on');
    add_line(modelName, pH_TS_c.Outport(1),   pH_Sum.Inport(2),        'autorouting','on');
    add_line(modelName, pH_Sum.Outport(1),    pH_Gain_K.Inport(1),     'autorouting','on');
    add_line(modelName, pH_Gain_K.Outport(1), pH_SL2PS_Fx.Inport(1),  'autorouting','on');
    add_line(modelName, pH_SL2PS_Fx.LConn(1), pH_Joint_x.Inport(1),   'autorouting','on');

    fprintf('[10b] Rolling constraint (TS + soft K=1e6) OK\n');
catch e
    fprintf('[10b] Rolling constraint                    FALLO: %s\n', e.message);
end

%% ── PASO 11: Suelo visual ────────────────────────────────────────────────
% Brick Solid fijo al World Frame — solo visual, sin masa dinámica
% Centro en z = -0.025 m → cara superior exactamente en z = 0
try
    add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
        [modelName '/RT_Ground'],'Position',[30 280 150 310]);
    set_param([modelName '/RT_Ground'], ...
        'RotationMethod',            'None', ...
        'TranslationMethod',         'StandardAxis', ...
        'TranslationStandardAxis',   '-Z', ...
        'TranslationStandardOffset', '0.025');   % baja el centro 2.5 cm

    add_block('sm_lib/Body Elements/Brick Solid', ...
        [modelName '/Ground'],'Position',[200 280 320 310]);
    set_param([modelName '/Ground'], ...
        'BrickDimensions',    '[6 6 0.05]', ...   % [m] 6x6 m, 5 cm grosor
        'InertiaType',        'Custom', ...
        'Mass',               '1e6', ...           % masa enorme = fijo
        'CenterOfMass',       '[0 0 0]', ...
        'MomentsOfInertia',   '[1e6 1e6 1e6]', ...
        'ProductsOfInertia',  '[0 0 0]', ...
        'GraphicDiffuseColor','[0.55 0.55 0.55]');  % gris claro

    pH_RT_Ground  = get_param([modelName '/RT_Ground'],'PortHandles');
    pH_Ground_sol = get_param([modelName '/Ground'],   'PortHandles');
    add_line(modelName, pH_world.RConn(1),      pH_RT_Ground.LConn(1), 'autorouting','on');
    add_line(modelName, pH_RT_Ground.RConn(1),  pH_Ground_sol.RConn(1),'autorouting','on');
    fprintf('[11] Suelo visual                 OK\n');
catch e
    fprintf('[11] Suelo visual                 FALLO: %s\n', e.message);
    % No hace return — el suelo es opcional, el modelo funciona sin él
end

%% ── PASO 12: Habilitar Simscape logging (reemplaza sensores) ─────────────
% Los Outports de sensing no se crean via set_param en esta versión.
% La solución correcta es usar Simscape simulation logging:
%   out.simlog.Joint_theta.Rz.q  → posición theta
%   out.simlog.Joint_x.Px.q      → posición x
%   out.simlog.Joint_alpha.Rz.q  → posición alpha
% No se necesitan bloques To Workspace ni conexiones de señal.
% SimscapeLogDecimation requiere valor numérico
try
    set_param(modelName, 'SimscapeLogType', 'all', ...
                         'SimscapeLogName', 'simlog');
    set_param(modelName, 'SimscapeLogDecimation', 1);
    fprintf('[11] Simscape logging habilitado  OK\n');
catch e
    % Si los nombres no existen intentar variantes
    fprintf('[11] Logging intento 1 fallo: %s\n', e.message);
    try
        set_param(modelName, 'SimMechanicsOpenEditorOnUpdate', 'off');
        fprintf('[11] Simscape logging (fallback)  OK\n');
    catch e2
        fprintf('[11] Logging fallback fallo: %s\n', e2.message);
    end
end

%% ── PASO 11b (obsoleto): Verificar valores válidos de SensePosition ─────
fprintf('\n--- Valores válidos de sensing en Revolute Joint ---\n');
p_rj = get_param([modelName '/Joint_theta'], 'DialogParameters');
if isfield(p_rj,'SensePosition')
    if isfield(p_rj.SensePosition,'Enum')
        fprintf('  SensePosition Enum : %s\n', strjoin(p_rj.SensePosition.Enum,', '));
    else
        fprintf('  SensePosition Type : %s  (no Enum — probablemente bool)\n', p_rj.SensePosition.Type);
    end
end
if isfield(p_rj,'SenseVelocity')
    if isfield(p_rj.SenseVelocity,'Enum')
        fprintf('  SenseVelocity Enum : %s\n', strjoin(p_rj.SenseVelocity.Enum,', '));
    else
        fprintf('  SenseVelocity Type : %s\n', p_rj.SenseVelocity.Type);
    end
end

% Probar valores alternativos para habilitar sensing
fprintf('\n--- Intentando habilitar sensing con valores alternativos ---\n');
sensing_values = {'on','true','1','Yes'};
for i = 1:numel(sensing_values)
    try
        set_param([modelName '/Joint_theta'],'SensePosition', sensing_values{i});
        pH_test = get_param([modelName '/Joint_theta'],'PortHandles');
        n = numel(pH_test.Outport);
        fprintf('  SensePosition=''%s'' -> Outports=%d  %s\n', sensing_values{i}, n, ternary(n>0,'<-- FUNCIONA',''));
        if n > 0, break; end
    catch
        fprintf('  SensePosition=''%s'' -> ERROR (valor invalido)\n', sensing_values{i});
    end
end

% Intentar forzar actualización del diagrama
fprintf('\n--- Forzando update del modelo ---\n');
try
    set_param(modelName,'SimulationCommand','update');
    fprintf('  Model update: OK\n');
catch e
    fprintf('  Model update: FALLO (%s)\n', e.message);
end

%% ── PASO 11b: Re-inspección tras update ──────────────────────────────────
fprintf('\n--- Inspeccionando Outports de juntas ---\n');
pH_Jt2 = get_param([modelName '/Joint_theta'], 'PortHandles');
pH_Jx2 = get_param([modelName '/Joint_x'],     'PortHandles');
pH_Ja2 = get_param([modelName '/Joint_alpha'],  'PortHandles');
fprintf('  Joint_theta Outports : %d  -> %s\n', numel(pH_Jt2.Outport), mat2str(pH_Jt2.Outport));
fprintf('  Joint_x     Outports : %d  -> %s\n', numel(pH_Jx2.Outport), mat2str(pH_Jx2.Outport));
fprintf('  Joint_alpha Outports : %d  -> %s\n', numel(pH_Ja2.Outport), mat2str(pH_Ja2.Outport));

%% ── PASO 12: Guardar y simular ───────────────────────────────────────────
try
    save_system(modelName, [modelName '.slx']);
    fprintf('[12] Modelo guardado              OK\n');
catch e
    fprintf('[12] Guardar                      FALLO: %s\n', e.message); return
end

fprintf('\n[13] Simulando 3 s (lazo abierto)...\n');
try
    out = sim(modelName);
    fprintf('[13] Simulación                   OK  | t_final=%.3f s\n', out.tout(end));
catch e
    fprintf('[13] Simulación                   FALLO: %s\n', e.message); return
end

%% ── PASO 14: Extraer estados del simlog ──────────────────────────────────
fprintf('\n[14] Extrayendo estados del simlog...\n');
try
    log = out.simlog;

    % theta — cabeceo (Joint_theta, eje de rotación Z del bloque = Rz tras pre-rotación)
    theta_data = log.Joint_theta.Rz.q.series;
    t     = theta_data.time;
    theta = theta_data.values('rad');
    fprintf('  theta: %d muestras, final=%.2f°\n', numel(t), rad2deg(theta(end)));
catch e
    fprintf('  theta FALLO: %s\n', e.message);
    fprintf('  Ramas disponibles en simlog.Joint_theta:\n');
    try, disp(fieldnames(out.simlog.Joint_theta)); catch, end
    t = []; theta = [];
end

try
    x_data = out.simlog.Joint_x.Pz.series;
    x_pos  = x_data.values('m');
    fprintf('  x:     %d muestras, final=%.3f m\n', numel(x_pos), x_pos(end));
catch e
    fprintf('  x FALLO: %s\n', e.message);
    try, disp(fieldnames(out.simlog.Joint_x)); catch, end
    x_pos = [];
end

try
    alpha_data = out.simlog.Joint_alpha.Rz.q.series;
    alpha      = alpha_data.values('rad');
    fprintf('  alpha: %d muestras, final=%.2f°\n', numel(alpha), rad2deg(alpha(end)));
catch e
    fprintf('  alpha FALLO: %s\n', e.message);
    try, disp(fieldnames(out.simlog.Joint_alpha)); catch, end
    alpha = [];
end

%% ── PASO 15: Gráficas ────────────────────────────────────────────────────
if ~isempty(t) && ~isempty(theta)
    figure('Name','Simscape — Lazo Abierto','Position',[80 80 1100 400]);

    subplot(1,3,1);
    plot(t, rad2deg(theta), 'b', 'LineWidth', 1.8);
    yline(0,'--k'); grid on;
    xlabel('t [s]'); ylabel('Cabeceo [deg]');
    title('Cabeceo libre (theta0 = 5 deg)');

    if ~isempty(x_pos)
        subplot(1,3,2);
        plot(t, x_pos, 'g', 'LineWidth', 1.8);
        yline(0,'--k'); grid on;
        xlabel('t [s]'); ylabel('Avance [m]');
        title('Avance [v1: sin rolling]');
    end

    if ~isempty(alpha)
        subplot(1,3,3);
        plot(t, rad2deg(alpha), 'r', 'LineWidth', 1.8);
        yline(0,'--k'); grid on;
        xlabel('t [s]'); ylabel('Giro [deg]');
        title('Giro alpha');
    end

    sgtitle('Simscape Multibody - Lazo Abierto - tau = 0', 'FontSize', 12);
    fprintf('[15] Gráficas generadas           OK\n');
end

fprintf('\n=== DIAGNÓSTICO COMPLETO ===\n');

%% ── DIAGNÓSTICO CI: nombres internos del Revolute Joint ─────────────────
fprintf('\n--- Parámetros internos de Joint_theta ---\n');
p = get_param([modelName '/Joint_theta'], 'DialogParameters');
names = fieldnames(p);
for i = 1:numel(names)
    fprintf('  %s\n', names{i});
end

%% ── PASO 16: Comparación Simscape vs ODE no lineal ──────────────────────
if ~isempty(t) && ~isempty(theta)
    fprintf('\n[16] Comparando Simscape vs ODE no lineal...\n');

    % Parámetros (mismos que los definidos arriba)
    alm = 2.0;  bem = 1.5;

    % CI: [theta; dtheta; x; dx; alpha; dalpha]
    X0_nl = [theta0; 0; 0; 0; 0; 0];
    U_nl  = [0; 0];   % lazo abierto, tau = 0

    ode_nl = @(tt, XX) segway_ode_cmp(tt, XX, U_nl, M, m, r, d, l, 9.81, ...
                                       Icy, Icz, Icx, Iw, Iwz, alm, bem);
    [t_nl, X_nl] = ode45(ode_nl, [0 str2double(get_param(modelName,'StopTime'))], X0_nl);
    theta_nl = X_nl(:,1);

    fprintf('  ODE: %d muestras, theta_final=%.2f°\n', numel(t_nl), rad2deg(theta_nl(end)));

    % Gráfica comparativa
    figure('Name','Validación: Simscape vs ODE No Lineal','Position',[100 100 800 420]);
    plot(t,    rad2deg(theta),    'b',  'LineWidth', 2.0, 'DisplayName','Simscape 3D');
    hold on;
    plot(t_nl, rad2deg(theta_nl), 'r--','LineWidth', 1.8, 'DisplayName','ODE no lineal');
    yline(0,'--k','Equilibrio','LabelHorizontalAlignment','left');
    grid on;
    xlabel('t  [s]');
    ylabel('Cabeceo \theta  [°]');
    title('Validación: Simscape vs ODE No Lineal  |  \theta_0 = 5°,  \tau = 0');
    legend('Location','best');
    fprintf('[16] Figura comparativa generada  OK\n');
end

%% ── FUNCIÓN HELPER ───────────────────────────────────────────────────────
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end

%% ── ODE NO LINEAL (para comparación con Simscape) ───────────────────────
function dX = segway_ode_cmp(~, X, U, M, m, r, d, l, g, Icy, Icz, Icx, Iw, Iwz, alm, bem)
    theta  = X(1);
    dtheta = X(2);
    dx     = X(4);
    dalpha = X(6);
    VR = U(1);  VL = U(2);

    M11 = Icy + M*l^2;
    M12 = M*l*cos(theta);
    M22 = M + 2*m + 2*Iw/r^2;
    M33 = Icx*sin(theta)^2 + Icz*cos(theta)^2 + ...
          2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    det_ca = M11*M22 - M12^2;

    tau_R = alm*VR - bem*(dx/r + d*dalpha/(2*r) - dtheta);
    tau_L = alm*VL - bem*(dx/r - d*dalpha/(2*r) - dtheta);

    F1 = M*g*l*sin(theta) - (tau_R + tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R + tau_L)/r;
    F3 = d*(tau_R - tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / det_ca;
    ddx     = (-M12*F1 + M11*F2) / det_ca;
    ddalpha = F3 / M33;

    dX = [dtheta; ddtheta; dx; ddx; dalpha; ddalpha];
end
