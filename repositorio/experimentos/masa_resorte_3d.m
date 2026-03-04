%% =========================================================================
%  MASA-RESORTE-AMORTIGUADOR 3D — Simscape Multibody R2025b
%  Masa suspendida, lazo abierto, Mechanics Explorer.
%
%  Basado en documentación oficial de MathWorks:
%  - Brick Solid: https://mathworks.com/help/sm/ref/bricksolid.html
%  - Prismatic Joint: https://mathworks.com/help/sm/ref/prismaticjoint.html
%
%  m = 1 kg, k = 100 N/m, c = 2 N·s/m, x0 = 0.05 m
%  wn = 10 rad/s, zeta = 0.1, ~3-5 oscilaciones en 10 s
%% =========================================================================
clearvars; clc; close all;
try bdclose('all'); catch; end

%% --- Parámetros físicos ---
m   = 1;          % kg
k   = 100;        % N/m
c   = 2;          % N·s/m
x0  = 0.05;       % m
g   = 9.81;       % m/s²
delta_eq = m*g/k;  % 0.0981 m

%% --- Crear modelo (nombre distinto al script para evitar shadowing) ---
mdl = 'mra_modelo_3d';
try close_system(mdl, 0); catch; end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end
new_system(mdl, 'Model');
open_system(mdl);

%% =========================================================================
%  AGREGAR BLOQUES
%% =========================================================================

% World Frame
add_block('sm_lib/Frames and Transforms/World Frame', ...
    [mdl '/World'], 'Position', [100 200 170 250]);

% Mechanism Configuration
add_block('sm_lib/Utilities/Mechanism Configuration', ...
    [mdl '/MechConfig'], 'Position', [100 300 250 350]);

% Solver Configuration (Simscape — obligatorio)
add_block('nesl_utility/Solver Configuration', ...
    [mdl '/SolverConfig'], 'Position', [100 400 250 450]);

% Rigid Transform: rotar frame 90° en X para que eje Z apunte hacia abajo
% (El Prismatic Joint actúa en Z del frame Base. Gravedad es -Y global.
%  Rotando -90° en X: Z_local → -Y_global → alineado con gravedad)
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [mdl '/RT_Rotacion'], 'Position', [300 200 400 250]);

% Prismatic Joint
add_block('sm_lib/Joints/Prismatic Joint', ...
    [mdl '/Prismatic'], 'Position', [500 195 630 255]);

% Masa (Brick Solid — cubo azul)
add_block('sm_lib/Body Elements/Brick Solid', ...
    [mdl '/Masa'], 'Position', [750 195 850 255]);

% Techo visual (de donde cuelga el resorte)
add_block('sm_lib/Body Elements/Brick Solid', ...
    [mdl '/Techo'], 'Position', [300 350 400 420]);

% Suelo visual
add_block('sm_lib/Body Elements/Brick Solid', ...
    [mdl '/Suelo'], 'Position', [300 470 400 540]);
add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [mdl '/RT_Suelo'], 'Position', [100 480 200 530]);

fprintf('Bloques agregados.\n');

%% =========================================================================
%  CONFIGURAR GRAVEDAD  [0 -g 0]
%% =========================================================================
set_param([mdl '/MechConfig'], 'GravityVector', sprintf('[0 -%g 0]', g));

%% =========================================================================
%  RIGID TRANSFORM: Rotar para alinear Prismatic (eje Z) con gravedad (-Y)
%% =========================================================================
set_param([mdl '/RT_Rotacion'], ...
    'RotationMethod',       'StandardAxis', ...
    'RotationStandardAxis', '+X', ...
    'RotationAngle',        '-90', ...
    'RotationAngleUnits',   'deg');

%% =========================================================================
%  RIGID TRANSFORM: Suelo visual abajo
%% =========================================================================
set_param([mdl '/RT_Suelo'], ...
    'TranslationMethod',          'Cartesian', ...
    'TranslationCartesianOffset', sprintf('[0 -%g 0]', delta_eq + x0 + 0.5));

%% =========================================================================
%  CONFIGURAR BRICK SOLIDS
%  Documentación: parámetro "Dimensions" = [x y z] en metros
%  Color: parámetro "Color" = [R G B] o [R G B A]
%  Inertia Type: "Calculate from Geometry" (default), Based on: "Density"
%% =========================================================================

% --- MASA: cubo azul 0.08 m ---
set_param([mdl '/Masa'], ...
    'Density', num2str(m / 0.08^3));
% Intentamos Dimensions con try/catch por si el nombre cambió en R2025b
try
    set_param([mdl '/Masa'], 'Dimensions', '[0.08 0.08 0.08]');
catch e1
    % Si falla, listar parámetros para diagnóstico
    fprintf('ERROR en Dimensions de Masa: %s\n', e1.message);
    fprintf('Listando parámetros del Brick Solid:\n');
    dp = get_param([mdl '/Masa'], 'DialogParameters');
    names = fieldnames(dp);
    for i = 1:numel(names)
        val = get_param([mdl '/Masa'], names{i});
        if ischar(val)
            fprintf('  %-30s = %s\n', names{i}, val);
        end
    end
    error('No se pudo configurar Dimensions. Revisa la lista de parámetros arriba.');
end
try set_param([mdl '/Masa'], 'Color', '[0.15 0.35 0.85]'); catch; end

% --- TECHO: placa oscura ---
set_param([mdl '/Techo'], 'Dimensions', '[0.3 0.02 0.3]');
set_param([mdl '/Techo'], 'Density', '500');
try set_param([mdl '/Techo'], 'Color', '[0.25 0.25 0.25]'); catch; end

% --- SUELO: placa clara ---
set_param([mdl '/Suelo'], 'Dimensions', '[0.3 0.02 0.3]');
set_param([mdl '/Suelo'], 'Density', '500');
try set_param([mdl '/Suelo'], 'Color', '[0.75 0.75 0.75]'); catch; end

fprintf('Sólidos configurados.\n');

%% =========================================================================
%  CONFIGURAR PRISMATIC JOINT
%  Documentación:
%  - Actúa en eje Z del frame Base
%  - Puertos: B (Base), F (Follower)
%  - Actuation > Motion = "Automatically Computed" → movimiento libre (default)
%  - Internal Mechanics: SpringStiffness, DampingCoefficient, EquilibriumPosition
%  - State Targets: PositionTargetSpecify, PositionTargetValue, etc.
%% =========================================================================

% Spring + Damper (Internal Mechanics)
set_param([mdl '/Prismatic'], ...
    'SpringStiffness',     num2str(k), ...
    'DampingCoefficient',  num2str(c), ...
    'EquilibriumPosition', num2str(-delta_eq));

fprintf('Spring = %g N/m, Damper = %g N·s/m, Eq = %g m\n', k, c, -delta_eq);

% State Targets: condición inicial
% La masa empieza en equilibrio + perturbación x0 (hacia abajo = -Z local)
set_param([mdl '/Prismatic'], ...
    'PositionTargetSpecify',  'on', ...
    'PositionTargetValue',    num2str(-(delta_eq + x0)), ...
    'PositionTargetPriority', 'High');

set_param([mdl '/Prismatic'], ...
    'VelocityTargetSpecify',  'on', ...
    'VelocityTargetValue',    '0', ...
    'VelocityTargetPriority', 'High');

fprintf('IC: pos = %g m, vel = 0 m/s\n', -(delta_eq + x0));

% Verificar que Motion es Automatically Computed (free motion)
motionMode = get_param([mdl '/Prismatic'], 'MotionActuationMode');
fprintf('MotionActuationMode = %s (debe ser AutomaticallyComputed)\n', motionMode);

%% =========================================================================
%  CONEXIONES
%  Documentación:
%  - Prismatic Joint: puertos B (LConn1) y F (RConn1)
%  - Brick Solid: puerto R (RConn1)
%  - World Frame: puerto (RConn1)
%  - Rigid Transform: puertos B (LConn1) y F (RConn1)
%% =========================================================================

% Utilidades conectadas al World
add_line(mdl, 'World/RConn1', 'MechConfig/RConn1',  'autorouting', 'on');
add_line(mdl, 'World/RConn1', 'SolverConfig/RConn1', 'autorouting', 'on');

% Techo visual: directo al World
add_line(mdl, 'World/RConn1', 'Techo/RConn1', 'autorouting', 'on');

% Cadena principal: World → Rotación → Prismatic(B→F) → Masa
add_line(mdl, 'World/RConn1',       'RT_Rotacion/LConn1', 'autorouting', 'on');
add_line(mdl, 'RT_Rotacion/RConn1', 'Prismatic/LConn1',   'autorouting', 'on');
add_line(mdl, 'Prismatic/RConn1',   'Masa/RConn1',        'autorouting', 'on');

% Suelo visual
add_line(mdl, 'World/RConn1',    'RT_Suelo/LConn1',  'autorouting', 'on');
add_line(mdl, 'RT_Suelo/RConn1', 'Suelo/RConn1',     'autorouting', 'on');

fprintf('Conexiones OK.\n');

%% =========================================================================
%  SOLVER
%% =========================================================================
set_param(mdl, ...
    'StopTime',  '10', ...
    'Solver',    'ode23t', ...
    'MaxStep',   '0.005', ...
    'RelTol',    '1e-5');

% Organizar diagrama
try Simulink.BlockDiagram.arrangeSystem(mdl); catch; end
set_param(mdl, 'ZoomFactor', 'FitSystem');

% Guardar
save_system(mdl);

%% =========================================================================
%  RESUMEN
%% =========================================================================
fprintf('\n========================================\n');
fprintf('  MODELO LISTO: %s.slx\n', mdl);
fprintf('  m = %.1f kg\n', m);
fprintf('  k = %.0f N/m\n', k);
fprintf('  c = %.1f N·s/m\n', c);
fprintf('  wn = %.1f rad/s (%.2f Hz)\n', sqrt(k/m), sqrt(k/m)/(2*pi));
fprintf('  zeta = %.3f (subamortiguado)\n', c/(2*sqrt(k*m)));
fprintf('  delta_eq = %.4f m\n', delta_eq);
fprintf('  x0 = %.4f m\n', x0);
fprintf('========================================\n');
fprintf('\n  >> sim(''%s'')\n\n', mdl);
fprintf('El Mechanics Explorer se abre automáticamente.\n');
fprintf('Deberías ver un cubo azul oscilando verticalmente.\n\n');