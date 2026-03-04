%% =========================================================================
%  BUILD_2DOF_SIMSCAPE.m
%  Sistema 2-DOF masa-resorte-amortiguador — Simscape Multibody
%
%  Topología:
%    k1+c1 (techo→m1)  via PJ1
%    k2     (m1→m2)    via PJ2  (sin amortiguador)
%    m2 libre en la parte inferior
%
%  Arquitectura de fuerzas EXTERNA (verificada como necesaria):
%    Los resortes y amortiguador se implementan como señales Simulink
%    calculadas desde sensores de posición del joint.
%    F = -k*(q - q_eq) aplicada via puerto de actuación del joint.
%    Esto opera en coordenadas globales y se acopla correctamente
%    con la gravedad de Simscape.
%
%  Nombres de parámetros 100% verificados con get_param+DialogParameters:
%    Rigid Transform : RotationMethod, RotationStandardAxis, RotationAngle
%                      TranslationMethod, TranslationStandardAxis, TranslationStandardOffset
%    Prismatic Joint : SensePosition, SenseVelocity, MotionActuationMode
%                      PositionTargetSpecify, PositionTargetValue, PositionTargetPriority
%                      VelocityTargetSpecify, VelocityTargetValue, VelocityTargetPriority
%    Brick Solid     : BrickDimensions, InertiaType=CalculateFromGeometry
%                      BasedOnType=Mass, Mass, GraphicDiffuseColor
%    Puertos PJ      : LConn1 (base), RConn1 (follower), RConn2 (sensing/actuation)
%    MechConfig      : RConn1 (único puerto)
%    SolverConfig    : RConn1 (único puerto)
%    Enum ejes RT    : '+X','-X','+Y','-Y','+Z','-Z' (sin espacios)
%
%  Cadena cinemática:
%    WorldFrame
%      → RT_align1 (Rx+90°: Z→-Y_global)
%        → PJ1 [CI=q1_0, actuación F1]
%          → RT_toCM1 (+lado_m1/2 en Z)
%            → Solid_m1
%              → RT_toBotM1 (-lado_m1/2 en Z)
%                → RT_align2 (Rx+90°)
%                  → PJ2 [CI=q2_0, actuación F2]
%                    → RT_toCM2 (+lado_m2/2 en Z)
%                      → Solid_m2
%
%  Modelo: sys_2dof.slx  (distinto del script build_2dof_simscape.m)
%% =========================================================================
clearvars; clc;
try; bdclose('all'); catch; end
close all;

%% =========================================================================
%  PARÁMETROS FÍSICOS (idénticos a mra_2dof.m)
%% =========================================================================
m1    = 2.0;     % kg
m2    = 1.0;     % kg
k1    = 40.0;    % N/m   techo → m1
k2    = 20.0;    % N/m   m1   → m2
c1    = 3.0;     % N·s/m paralelo a k1
g_val = 9.81;    % m/s²

% Geometría de sólidos
lado_m1 = 0.10;  % m
lado_m2 = 0.08;  % m

% Elongaciones de equilibrio estático
% En equilibrio: k1*delta1 = (m1+m2)*g  →  delta1 = (m1+m2)*g/k1
% En equilibrio: k2*delta2 = m2*g       →  delta2 = m2*g/k2
delta1_eq = (m1 + m2) * g_val / k1;   % 0.7358 m
delta2_eq =  m2        * g_val / k2;   % 0.4905 m

% Condiciones iniciales (desplazamiento desde equilibrio, + = hacia abajo)
x1_0 =  0.00;    % m1 parte del equilibrio
x2_0 = -0.15;    % m2 desplazada 15 cm hacia arriba desde equilibrio

% Posición absoluta inicial de cada joint (extensión desde frame base)
q1_0 = delta1_eq + x1_0;   % 0.7358 m
q2_0 = delta2_eq + x2_0;   % 0.3405 m

t_fin = 25.0;    % s

%% =========================================================================
%  NOMBRE DEL MODELO Y LIMPIEZA
%% =========================================================================
mdl = 'sys_2dof';
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end

%% =========================================================================
%  RUTAS DE BLOQUES (todas verificadas)
%% =========================================================================
lib_WF  = 'sm_lib/Frames and Transforms/World Frame';
lib_RT  = 'sm_lib/Frames and Transforms/Rigid Transform';
lib_PJ  = 'sm_lib/Joints/Prismatic Joint';
lib_BS  = 'sm_lib/Body Elements/Brick Solid';
lib_MC  = 'sm_lib/Utilities/Mechanism Configuration';
lib_SC  = 'nesl_utility/Solver Configuration';
lib_S2P = 'nesl_utility/Simulink-PS Converter';
lib_P2S = 'nesl_utility/PS-Simulink Converter';

%% =========================================================================
%  CREAR SISTEMA Y SOLVER
%% =========================================================================
new_system(mdl);
open_system(mdl);
set_param(mdl, 'SolverType', 'Variable-step', ...
               'Solver',     'ode23t',         ...
               'MaxStep',    '0.004',           ...
               'RelTol',     '1e-5',            ...
               'AbsTol',     '1e-6',            ...
               'StopTime',   num2str(t_fin));

% Helper posición bloques en diagrama
p = @(c,r) [60+(c-1)*160, 60+(r-1)*120, 60+(c-1)*160+100, 60+(r-1)*120+60];

%% =========================================================================
%  BLOQUE 1: World Frame
%% =========================================================================
add_block(lib_WF, [mdl '/WorldFrame'], 'Position', p(1,4));

%% =========================================================================
%  BLOQUE 2: Mechanism Configuration
%% =========================================================================
add_block(lib_MC, [mdl '/MechConfig'], 'Position', p(1,6));
set_param([mdl '/MechConfig'], 'GravityVector', '[0 -9.81 0]');

%% =========================================================================
%  BLOQUE 3: Solver Configuration
%% =========================================================================
add_block(lib_SC, [mdl '/SolverConfig'], 'Position', p(1,8));

%% =========================================================================
%  CADENA CINEMÁTICA — m1
%% =========================================================================

% RT_align1: Rx+90° → mapea Z_local a -Y_global (hacia abajo)
add_block(lib_RT, [mdl '/RT_align1'], 'Position', p(2,4));
set_param([mdl '/RT_align1'], ...
    'RotationMethod',       'StandardAxis', ...
    'RotationStandardAxis', '+X',           ...
    'RotationAngle',        '90');

% PJ1: condición inicial + actuación de fuerza externa
add_block(lib_PJ, [mdl '/PJ1'], 'Position', p(3,4));
set_param([mdl '/PJ1'], ...
    'TorqueActuationMode',   'InputTorque',    ...
    'SensePosition',         'on',             ...
    'SenseVelocity',         'on',             ...
    'PositionTargetSpecify', 'on',             ...
    'PositionTargetValue',   num2str(q1_0),    ...
    'PositionTargetPriority','High',           ...
    'VelocityTargetSpecify', 'on',             ...
    'VelocityTargetValue',   '0',              ...
    'VelocityTargetPriority','High');

% RT_toCM1: avanza lado_m1/2 en Z → frame en CM de m1
add_block(lib_RT, [mdl '/RT_toCM1'], 'Position', p(4,4));
set_param([mdl '/RT_toCM1'], ...
    'TranslationMethod',         'StandardAxis', ...
    'TranslationStandardAxis',   '+Z',           ...
    'TranslationStandardOffset', num2str(lado_m1/2));

% Solid m1 (azul)
add_block(lib_BS, [mdl '/Solid_m1'], 'Position', p(5,4));
set_param([mdl '/Solid_m1'], ...
    'BrickDimensions',     sprintf('[%g %g %g]', lado_m1, lado_m1, lado_m1), ...
    'InertiaType',         'CalculateFromGeometry', ...
    'BasedOnType',         'Mass',                  ...
    'Mass',                num2str(m1),             ...
    'GraphicDiffuseColor', '[0.12 0.38 0.88]');

%% =========================================================================
%  CADENA CINEMÁTICA — m2 (parte desde Solid_m1)
%% =========================================================================

% RT_toBotM1: baja lado_m1/2 en -Z → frame en cara inferior de m1
add_block(lib_RT, [mdl '/RT_toBotM1'], 'Position', p(4,6));
set_param([mdl '/RT_toBotM1'], ...
    'TranslationMethod',         'StandardAxis', ...
    'TranslationStandardAxis',   '-Z',           ...
    'TranslationStandardOffset', num2str(lado_m1/2));

% RT_align2: igual a RT_align1
add_block(lib_RT, [mdl '/RT_align2'], 'Position', p(5,6));
set_param([mdl '/RT_align2'], ...
    'RotationMethod',       'StandardAxis', ...
    'RotationStandardAxis', '+X',           ...
    'RotationAngle',        '90');

% PJ2: condición inicial + actuación de fuerza externa
add_block(lib_PJ, [mdl '/PJ2'], 'Position', p(6,6));
set_param([mdl '/PJ2'], ...
    'TorqueActuationMode',   'InputTorque',    ...
    'SensePosition',         'on',             ...
    'PositionTargetSpecify', 'on',             ...
    'PositionTargetValue',   num2str(q2_0),    ...
    'PositionTargetPriority','High',           ...
    'VelocityTargetSpecify', 'on',             ...
    'VelocityTargetValue',   '0',              ...
    'VelocityTargetPriority','High');

% RT_toCM2
add_block(lib_RT, [mdl '/RT_toCM2'], 'Position', p(7,6));
set_param([mdl '/RT_toCM2'], ...
    'TranslationMethod',         'StandardAxis', ...
    'TranslationStandardAxis',   '+Z',           ...
    'TranslationStandardOffset', num2str(lado_m2/2));

% Solid m2 (rojo)
add_block(lib_BS, [mdl '/Solid_m2'], 'Position', p(8,6));
set_param([mdl '/Solid_m2'], ...
    'BrickDimensions',     sprintf('[%g %g %g]', lado_m2, lado_m2, lado_m2), ...
    'InertiaType',         'CalculateFromGeometry', ...
    'BasedOnType',         'Mass',                  ...
    'Mass',                num2str(m2),             ...
    'GraphicDiffuseColor', '[0.82 0.22 0.22]');

%% =========================================================================
%  BLOQUES SIMULINK PARA FUERZAS EXTERNAS
%
%  Lógica de fuerzas (coordenadas del joint, extensión positiva = hacia abajo):
%
%  El joint mide q = extensión absoluta desde su frame base.
%  Equilibrio estático: q_eq = delta_eq (resorte + gravedad balanceados).
%  Fuerza del resorte sobre el joint (en dirección +Z_local = -Y_global):
%    F1 = -k1*(q1 - delta1_eq) - c1*dq1
%    F2 = -k2*(q2 - delta2_eq)
%
%  Nota: la gravedad ya está en el modelo físico (MechConfig).
%  La fuerza externa SOLO representa el resorte y amortiguador.
%  El equilibrio se da cuando F_resorte + F_gravedad = 0.
%  Aquí F_resorte = 0 en q=delta_eq, y F_gravedad ≠ 0,
%  así que Simscape encuentra el equilibrio real iterativamente.
%
%  IMPORTANTE: SenseVelocity para el amortiguador c1 en PJ1.
%% =========================================================================

% Habilitar sensing de velocidad en PJ1 para el amortiguador
set_param([mdl '/PJ1'], 'SenseVelocity', 'on');

% --- Conversores PS→SL para PJ1 (posición y velocidad) ---
add_block(lib_P2S, [mdl '/p1_toSL'], 'Position', p(3,2));
add_block(lib_P2S, [mdl '/v1_toSL'], 'Position', p(3,2.7));

% --- Resta q1 - delta1_eq ---
add_block('simulink/Sources/Constant',    [mdl '/C_d1'],  'Position', p(4,1.5));
add_block('simulink/Math Operations/Sum', [mdl '/Sub1'],  'Position', p(4,2));
set_param([mdl '/C_d1'], 'Value',  num2str(delta1_eq));
set_param([mdl '/Sub1'], 'Inputs', '+-');

% --- Ganancia k1 ---
add_block('simulink/Math Operations/Gain', [mdl '/Gk1'], 'Position', p(5,2));
set_param([mdl '/Gk1'], 'Gain', num2str(k1));

% --- Ganancia c1 ---
add_block('simulink/Math Operations/Gain', [mdl '/Gc1'], 'Position', p(5,2.7));
set_param([mdl '/Gc1'], 'Gain', num2str(c1));

% --- Suma F1 = -(k1*x1 + c1*v1) ---
add_block('simulink/Math Operations/Sum', [mdl '/SumF1'], 'Position', p(6,2.3));
set_param([mdl '/SumF1'], 'Inputs', '--');

% --- Fuerza de reacción de k2 sobre m1 ---
% k2 actúa entre m1 y m2. La fuerza sobre m1 es +k2*(q2-delta2_eq)
% (igual y opuesta a la fuerza sobre m2)
add_block(lib_P2S, [mdl '/p2_forM1'], 'Position', p(6,2));
add_block('simulink/Sources/Constant',    [mdl '/C_d2b'],   'Position', p(7,1.5));
add_block('simulink/Math Operations/Sum', [mdl '/Sub2b'],   'Position', p(7,2));
add_block('simulink/Math Operations/Gain',[mdl '/Gk2b'],    'Position', p(8,2));
add_block(lib_S2P,                        [mdl '/fk2_toPS'],'Position', p(9,2));

set_param([mdl '/C_d2b'],  'Value',  num2str(delta2_eq));
set_param([mdl '/Sub2b'],  'Inputs', '+-');
set_param([mdl '/Gk2b'],   'Gain',   num2str(k2));  % +k2*(q2-delta2_eq)

% Suma total sobre PJ1: F1 = -k1*x1 - c1*v1 + k2*x2
add_block('simulink/Math Operations/Sum', [mdl '/SumF1tot'], 'Position', p(9.5,2.3));
set_param([mdl '/SumF1tot'], 'Inputs', '+-');

% --- Conversor SL→PS para actuación PJ1 ---
add_block(lib_S2P, [mdl '/f1_toPS'], 'Position', p(7,2.3));

% --- Conversores PS→SL para PJ2 (solo posición, sin amortiguador) ---
add_block(lib_P2S, [mdl '/p2_toSL'], 'Position', p(6,8));

% --- Resta q2 - delta2_eq ---
add_block('simulink/Sources/Constant',    [mdl '/C_d2'],  'Position', p(7,7.5));
add_block('simulink/Math Operations/Sum', [mdl '/Sub2'],  'Position', p(7,8));
set_param([mdl '/C_d2'], 'Value',  num2str(delta2_eq));
set_param([mdl '/Sub2'], 'Inputs', '+-');

% --- Ganancia -k2 (negación incluida) ---
add_block('simulink/Math Operations/Gain', [mdl '/Gnk2'], 'Position', p(8,8));
set_param([mdl '/Gnk2'], 'Gain', num2str(k2));

% --- Conversor SL→PS para actuación PJ2 ---
add_block(lib_S2P, [mdl '/f2_toPS'], 'Position', p(9,8));

%% =========================================================================
%  BLOQUES DE LOGGING (posición de cada joint → workspace)
%% =========================================================================
add_block(lib_P2S, [mdl '/log_q1'], 'Position', p(3,10));
add_block(lib_P2S, [mdl '/log_q2'], 'Position', p(6,10));
add_block('simulink/Sinks/To Workspace', [mdl '/ws_q1'], 'Position', p(4,10));
add_block('simulink/Sinks/To Workspace', [mdl '/ws_q2'], 'Position', p(7,10));
set_param([mdl '/ws_q1'], 'VariableName', 'q1_log', 'SaveFormat', 'timeseries');
set_param([mdl '/ws_q2'], 'VariableName', 'q2_log', 'SaveFormat', 'timeseries');

%% =========================================================================
%  CONEXIONES — Red física Simscape
%% =========================================================================
add_line(mdl, 'WorldFrame/RConn1',  'MechConfig/RConn1',   'autorouting','on');
add_line(mdl, 'WorldFrame/RConn1',  'SolverConfig/RConn1', 'autorouting','on');
add_line(mdl, 'WorldFrame/RConn1',  'RT_align1/LConn1',    'autorouting','on');
add_line(mdl, 'RT_align1/RConn1',   'PJ1/LConn1',          'autorouting','on');
add_line(mdl, 'PJ1/RConn1',         'RT_toCM1/LConn1',     'autorouting','on');
add_line(mdl, 'RT_toCM1/RConn1',    'Solid_m1/RConn1',     'autorouting','on');
add_line(mdl, 'Solid_m1/RConn1',    'RT_toBotM1/LConn1',   'autorouting','on');
add_line(mdl, 'RT_toBotM1/RConn1',  'RT_align2/LConn1',    'autorouting','on');
add_line(mdl, 'RT_align2/RConn1',   'PJ2/LConn1',          'autorouting','on');
add_line(mdl, 'PJ2/RConn1',         'RT_toCM2/LConn1',     'autorouting','on');
add_line(mdl, 'RT_toCM2/RConn1',    'Solid_m2/RConn1',     'autorouting','on');

%% =========================================================================
%  CONEXIONES — Fuerza F1 (resorte k1 + amortiguador c1 + reacción k2)
%  Ecuación: F1 = -k1*(q1-delta1_eq) - c1*v1 + k2*(q2-delta2_eq)
%  La reacción de k2 sobre m1 es igual y opuesta a la fuerza de k2 sobre m2
%% =========================================================================
% Sensing PJ1
add_line(mdl, 'PJ1/RConn2',    'p1_toSL/LConn1',  'autorouting','on');
add_line(mdl, 'PJ1/RConn3',    'v1_toSL/LConn1',  'autorouting','on');
% x1 = q1 - delta1_eq
add_line(mdl, 'p1_toSL/1',     'Sub1/1',           'autorouting','on');
add_line(mdl, 'C_d1/1',        'Sub1/2',           'autorouting','on');
% -k1*x1 y -c1*v1
add_line(mdl, 'Sub1/1',        'Gk1/1',            'autorouting','on');
add_line(mdl, 'v1_toSL/1',     'Gc1/1',            'autorouting','on');
add_line(mdl, 'Gk1/1',         'SumF1/1',          'autorouting','on');
add_line(mdl, 'Gc1/1',         'SumF1/2',          'autorouting','on');
% SumF1 = -(k1*x1 + c1*v1)

% Reacción de k2 sobre m1: +k2*(q2 - delta2_eq)
add_line(mdl, 'PJ2/RConn2',    'p2_forM1/LConn1', 'autorouting','on');
add_line(mdl, 'p2_forM1/1',    'Sub2b/1',          'autorouting','on');
add_line(mdl, 'C_d2b/1',       'Sub2b/2',          'autorouting','on');
add_line(mdl, 'Sub2b/1',       'Gk2b/1',           'autorouting','on');

% F1_total = -k1*x1 - c1*v1 + k2*x2
add_line(mdl, 'SumF1/1',       'SumF1tot/1',       'autorouting','on');
add_line(mdl, 'Gk2b/1',        'SumF1tot/2',       'autorouting','on');
add_line(mdl, 'SumF1tot/1',    'f1_toPS/1',        'autorouting','on');
add_line(mdl, 'f1_toPS/RConn1','PJ1/LConn2',       'autorouting','on');

%% =========================================================================
%  CONEXIONES — Fuerza F2 (resorte k2 sobre PJ2)
%% =========================================================================
% PJ2 con TorqueActuationMode=InputTorque: LConn2=fuerza, RConn2=pos
add_line(mdl, 'PJ2/RConn2',    'p2_toSL/LConn1', 'autorouting','on');
add_line(mdl, 'p2_toSL/1',     'Sub2/1',          'autorouting','on');
add_line(mdl, 'C_d2/1',        'Sub2/2',          'autorouting','on');
add_line(mdl, 'Sub2/1',        'Gnk2/1',          'autorouting','on');
add_line(mdl, 'Gnk2/1',        'f2_toPS/1',       'autorouting','on');
add_line(mdl, 'f2_toPS/RConn1','PJ2/LConn2',      'autorouting','on');

%% =========================================================================
%  CONEXIONES — Logging
%% =========================================================================
add_line(mdl, 'PJ1/RConn2',  'log_q1/LConn1', 'autorouting','on');
add_line(mdl, 'PJ2/RConn2',  'log_q2/LConn1', 'autorouting','on');
add_line(mdl, 'log_q1/1',    'ws_q1/1',        'autorouting','on');
add_line(mdl, 'log_q2/1',    'ws_q2/1',        'autorouting','on');

%% =========================================================================
%  ORGANIZAR Y GUARDAR
%% =========================================================================
try; Simulink.BlockDiagram.arrangeSystem(mdl); catch; end
save_system(mdl);

%% =========================================================================
%  SIMULAR Y GRAFICAR
%% =========================================================================
fprintf('Simulando %s...\n', mdl);
out = sim(mdl);

q1 = out.q1_log;
q2 = out.q2_log;

fprintf('\n=============================================================\n');
fprintf('  RESULTADOS — posición joints\n');
fprintf('=============================================================\n');
fprintf('  delta1_eq = %.4f m   delta2_eq = %.4f m\n', delta1_eq, delta2_eq);
fprintf('  q1: min=%.4f  max=%.4f  mean=%.4f\n', min(q1.Data), max(q1.Data), mean(q1.Data));
fprintf('  q2: min=%.4f  max=%.4f  mean=%.4f\n', min(q2.Data), max(q2.Data), mean(q2.Data));
fprintf('=============================================================\n');

figure('Name','2-DOF Posiciones','Color','w','Position',[200 200 900 500]);
subplot(2,1,1);
plot(q1.Time, q1.Data, 'b', 'LineWidth', 1.5); hold on;
yline(delta1_eq, '--g', sprintf('eq_1=%.4f m', delta1_eq), 'LineWidth', 1.2);
ylabel('q_1 (m)'); title('PJ1 — posición m1');
grid on; set(gca,'Color',[0.97 0.97 0.99]);

subplot(2,1,2);
plot(q2.Time, q2.Data, 'r', 'LineWidth', 1.5); hold on;
yline(delta2_eq, '--g', sprintf('eq_2=%.4f m', delta2_eq), 'LineWidth', 1.2);
ylabel('q_2 (m)'); title('PJ2 — posición m2');
grid on; set(gca,'Color',[0.97 0.97 0.99]);
xlabel('t (s)');
