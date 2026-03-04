%% =========================================================================
%  GENERADOR SIMULINK — CONTROL LQR + MANIOBRA DE AVANCE
%  Compatible MATLAB R2019b
%  Ejecutar DESPUES de vehiculo_autobalanceado_ss.m
%% =========================================================================
close all;

%% --- Parametros y LQR ---
run('vehiculo_autobalanceado_ss.m');
close all;

Q = diag([1, 1, 1000, 500]);
R = 0.1;
K_lqr = lqr(A_pitch, B_pitch, Q, R);

%% --- Senal de perturbacion ---
t_end     = 30;
theta_max = 5*pi/180;
t_subida  = 5;
t_plano   = 8;
t_bajada  = 12;

t_vec     = (0:0.005:t_end)';
theta_ref = zeros(size(t_vec));

idx_sub  = t_vec <= t_subida;
idx_plan = t_vec > t_subida & t_vec <= t_plano;
idx_baj  = t_vec > t_plano  & t_vec <= t_bajada;

theta_ref(idx_sub)  = theta_max * (t_vec(idx_sub)/t_subida).^2;
theta_ref(idx_plan) = theta_max;
theta_ref(idx_baj)  = theta_max * (1 - ((t_vec(idx_baj)-t_plano)/(t_bajada-t_plano)).^2);

theta_perturbacion = [t_vec, theta_ref];

%% --- Crear modelo ---
mdl = 'segway_lqr';
if bdIsLoaded(mdl); close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);
ph = @(blk) get_param([mdl '/' blk], 'PortHandles');

%% =========================================================================
%  BLOQUES
%% =========================================================================

% 1. Perturbacion
add_block('simulink/Sources/From Workspace', [mdl '/Perturbacion'], ...
    'VariableName', 'theta_perturbacion', 'Interpolate', 'on', ...
    'Position', [30 263 110 287]);

% 2. e3: escalar -> vector [0;0;-1;0]
add_block('simulink/Math Operations/Gain', [mdl '/e3'], ...
    'Gain', '[0;0;-1;0]', 'Multiplication', 'Matrix(K*u)', ...
    'Position', [150 258 210 292]);

% 3. Planta
add_block('simulink/Continuous/State-Space', [mdl '/Planta'], ...
    'A', 'A_pitch', 'B', 'B_pitch', ...
    'C', 'eye(4)', 'D', 'zeros(4,1)', ...
    'X0', '[0;0;0;0]', ...
    'Position', [480 243 620 307]);

% 4. SumaTheta
add_block('simulink/Math Operations/Add', [mdl '/SumaTheta'], ...
    'Inputs', '++', 'Position', [660 248 690 302]);

% 5. Demux: separa 4 estados
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_x'], ...
    'Outputs', '4', 'Position', [730 246 736 304]);

% 6. K
add_block('simulink/Math Operations/Gain', [mdl '/K'], ...
    'Gain', 'K_lqr', 'Multiplication', 'Matrix(K*u)', ...
    'Position', [660 340 750 370]);

% 7. Neg
add_block('simulink/Math Operations/Gain', [mdl '/Neg'], ...
    'Gain', '-1', 'Position', [790 343 830 367]);

% 8. Sat_Vsum
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_Vsum'], ...
    'UpperLimit', '48', 'LowerLimit', '-48', ...
    'Position', [870 338 920 372]);

% 9. Half
add_block('simulink/Math Operations/Gain', [mdl '/Half'], ...
    'Gain', '0.5', 'Position', [960 343 1000 367]);

% 10. Sat_VL
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_VL'], ...
    'UpperLimit', '24', 'LowerLimit', '-24', ...
    'Position', [1040 323 1080 347]);

% 11. Sat_VR
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_VR'], ...
    'UpperLimit', '24', 'LowerLimit', '-24', ...
    'Position', [1040 363 1080 387]);

% 12. VsumFinal
add_block('simulink/Math Operations/Add', [mdl '/VsumFinal'], ...
    'Inputs', '++', 'Position', [1120 333 1160 377]);

% 13. Mux estados + Scope_Estados (R2019b: sin ScopeConfiguration)
add_block('simulink/Signal Routing/Mux', [mdl '/MuxEstados'], ...
    'Inputs', '4', 'Position', [780 150 786 260]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_Estados'], ...
    'Position', [830 185 870 225]);

% 14. Mux voltajes + Scope_Voltajes
add_block('simulink/Signal Routing/Mux', [mdl '/MuxScope'], ...
    'Inputs', '2', 'Position', [1120 418 1126 452]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_Voltajes'], ...
    'Position', [1170 423 1210 447]);

%% =========================================================================
%  CONEXIONES
%% =========================================================================

% Perturbacion -> e3
add_line(mdl, ph('Perturbacion').Outport(1), ph('e3').Inport(1),         'autorouting','on');

% Planta -> SumaTheta(1)
add_line(mdl, ph('Planta').Outport(1),       ph('SumaTheta').Inport(1),  'autorouting','on');

% e3 -> SumaTheta(2)
add_line(mdl, ph('e3').Outport(1),           ph('SumaTheta').Inport(2),  'autorouting','on');

% SumaTheta -> Demux
add_line(mdl, ph('SumaTheta').Outport(1),    ph('Demux_x').Inport(1),    'autorouting','on');

% SumaTheta -> K
add_line(mdl, ph('SumaTheta').Outport(1),    ph('K').Inport(1),          'autorouting','on');

% Demux -> MuxEstados
h1 = add_line(mdl, ph('Demux_x').Outport(1), ph('MuxEstados').Inport(1), 'autorouting','on');
h2 = add_line(mdl, ph('Demux_x').Outport(2), ph('MuxEstados').Inport(2), 'autorouting','on');
h3 = add_line(mdl, ph('Demux_x').Outport(3), ph('MuxEstados').Inport(3), 'autorouting','on');
h4 = add_line(mdl, ph('Demux_x').Outport(4), ph('MuxEstados').Inport(4), 'autorouting','on');

set_param(h1, 'Name', 'Posición');
set_param(h2, 'Name', 'Velocidad');
set_param(h3, 'Name', 'Inclinación');
set_param(h4, 'Name', 'VelInclinacion');

% MuxEstados -> Scope_Estados
add_line(mdl, ph('MuxEstados').Outport(1), ph('Scope_Estados').Inport(1),'autorouting','on');

% K -> Neg -> Sat_Vsum -> Half
add_line(mdl, ph('K').Outport(1),         ph('Neg').Inport(1),           'autorouting','on');
add_line(mdl, ph('Neg').Outport(1),       ph('Sat_Vsum').Inport(1),      'autorouting','on');
add_line(mdl, ph('Sat_Vsum').Outport(1),  ph('Half').Inport(1),          'autorouting','on');

% Half -> Sat_VL y Sat_VR
add_line(mdl, ph('Half').Outport(1),      ph('Sat_VL').Inport(1),        'autorouting','on');
add_line(mdl, ph('Half').Outport(1),      ph('Sat_VR').Inport(1),        'autorouting','on');

% Sat_VL, Sat_VR -> VsumFinal -> Planta
add_line(mdl, ph('Sat_VL').Outport(1),    ph('VsumFinal').Inport(1),     'autorouting','on');
add_line(mdl, ph('Sat_VR').Outport(1),    ph('VsumFinal').Inport(2),     'autorouting','on');
add_line(mdl, ph('VsumFinal').Outport(1), ph('Planta').Inport(1),        'autorouting','on');

% Sat_VL, Sat_VR -> MuxScope -> Scope_Voltajes
add_line(mdl, ph('Sat_VL').Outport(1),    ph('MuxScope').Inport(1),      'autorouting','on');
add_line(mdl, ph('Sat_VR').Outport(1),    ph('MuxScope').Inport(2),      'autorouting','on');
add_line(mdl, ph('MuxScope').Outport(1),  ph('Scope_Voltajes').Inport(1),'autorouting','on');

%% =========================================================================
%  SOLVER, AUTOARRANGE Y GUARDAR
%% =========================================================================
set_param(mdl, 'StopTime', '30', 'Solver', 'ode45', ...
    'MaxStep', '0.005', 'RelTol', '1e-4');

Simulink.BlockDiagram.arrangeSystem(mdl);
set_param(mdl, 'ZoomFactor', 'FitSystem');

save_system(mdl);
fprintf('\n=== Modelo listo para R2019b. Corre sim(''%s'') o presiona Run ===\n', mdl);
fprintf('Perturbacion: 0->5deg en 5s | plano 5-8s | baja 8->12s\n');
fprintf('NOTA: abrir Scope_Estados -> config -> layout 2x2 manualmente\n');