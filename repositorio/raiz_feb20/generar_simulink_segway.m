%% =========================================================================
%  GENERADOR DE MODELO SIMULINK — VEHICULO AUTOBALANCEADO (v2)
%  Usa port handles para conexiones robustas
%  Ejecutar DESPUES de vehiculo_autobalanceado_ss.m
%  =========================================================================
close all;

%% Cargar parametros
run('vehiculo_autobalanceado_ss.m');

%% Crear modelo
mdl = 'segway_control';
if bdIsLoaded(mdl); close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

%% Funcion auxiliar: conectar por port handles
ph = @(blk) get_param([mdl '/' blk], 'PortHandles');

%% =====================================================================
%  BLOQUES — CABECEO + AVANCE
%  =====================================================================

add_block('simulink/Sources/Constant', [mdl '/theta_ref'], ...
    'Value', '0', 'Position', [30 100 60 130]);

add_block('simulink/Math Operations/Sum', [mdl '/Sum_theta'], ...
    'Inputs', '+-', 'Position', [130 100 150 130]);

add_block('simulink/Continuous/PID Controller', [mdl '/PID_pitch'], ...
    'P', '50', 'I', '5', 'D', '8', 'N', '100', ...
    'Position', [210 92 300 138]);

add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_pitch'], ...
    'UpperLimit', '24', 'LowerLimit', '-24', ...
    'Position', [360 97 400 133]);

add_block('simulink/Continuous/State-Space', [mdl '/Planta_pitch'], ...
    'A', 'A_pitch', 'B', 'B_pitch', 'C', 'C_pitch', 'D', 'D_pitch', ...
    'X0', '[0; 0; 5*pi/180; 0]', ...
    'Position', [470 85 580 145]);

add_block('simulink/Signal Routing/Demux', [mdl '/Demux_pitch'], ...
    'Outputs', '2', 'Position', [640 85 645 145]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_phi'], ...
    'Position', [730 72 760 98]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_theta'], ...
    'Position', [730 127 760 153]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_Vsum'], ...
    'Position', [360 170 390 200]);

%% Conexiones cabeceo
add_line(mdl, ph('theta_ref').Outport(1),    ph('Sum_theta').Inport(1));
add_line(mdl, ph('Sum_theta').Outport(1),    ph('PID_pitch').Inport(1));
add_line(mdl, ph('PID_pitch').Outport(1),    ph('Sat_pitch').Inport(1));
add_line(mdl, ph('Sat_pitch').Outport(1),    ph('Planta_pitch').Inport(1));
add_line(mdl, ph('Planta_pitch').Outport(1), ph('Demux_pitch').Inport(1));
add_line(mdl, ph('Demux_pitch').Outport(1),  ph('Scope_phi').Inport(1));
add_line(mdl, ph('Demux_pitch').Outport(2),  ph('Scope_theta').Inport(1));
add_line(mdl, ph('Demux_pitch').Outport(2),  ph('Sum_theta').Inport(2));
add_line(mdl, ph('Sat_pitch').Outport(1),    ph('Scope_Vsum').Inport(1));

%% =====================================================================
%  BLOQUES — GIRO
%  =====================================================================

add_block('simulink/Sources/Constant', [mdl '/psi_ref'], ...
    'Value', '0', 'Position', [30 300 60 330]);

add_block('simulink/Math Operations/Sum', [mdl '/Sum_psi'], ...
    'Inputs', '+-', 'Position', [130 300 150 330]);

add_block('simulink/Continuous/PID Controller', [mdl '/PD_yaw'], ...
    'Controller', 'PD', 'P', '2', 'D', '0.3', 'N', '100', ...
    'Position', [210 292 300 338]);

add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_yaw'], ...
    'UpperLimit', '24', 'LowerLimit', '-24', ...
    'Position', [360 297 400 333]);

add_block('simulink/Continuous/State-Space', [mdl '/Planta_yaw'], ...
    'A', 'A_yaw', 'B', 'B_yaw', 'C', 'C_yaw', 'D', 'D_yaw', ...
    'X0', '[0; 0]', ...
    'Position', [470 290 580 340]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_psi'], ...
    'Position', [650 300 680 330]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_Vdif'], ...
    'Position', [360 370 390 400]);

%% Conexiones giro
add_line(mdl, ph('psi_ref').Outport(1),    ph('Sum_psi').Inport(1));
add_line(mdl, ph('Sum_psi').Outport(1),    ph('PD_yaw').Inport(1));
add_line(mdl, ph('PD_yaw').Outport(1),     ph('Sat_yaw').Inport(1));
add_line(mdl, ph('Sat_yaw').Outport(1),    ph('Planta_yaw').Inport(1));
add_line(mdl, ph('Planta_yaw').Outport(1), ph('Scope_psi').Inport(1));
add_line(mdl, ph('Planta_yaw').Outport(1), ph('Sum_psi').Inport(2));
add_line(mdl, ph('Sat_yaw').Outport(1),    ph('Scope_Vdif').Inport(1));

%% =====================================================================
%  BLOQUES — CONVERSION VL, VR
%  =====================================================================

% VR = (V_sum + V_dif) / 2
add_block('simulink/Math Operations/Sum', [mdl '/Sum_VR'], ...
    'Inputs', '++', 'Position', [500 430 520 450]);
add_block('simulink/Math Operations/Gain', [mdl '/Gain_VR'], ...
    'Gain', '0.5', 'Position', [570 432 600 448]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_VR'], ...
    'Position', [650 432 680 448]);

% VL = (V_sum - V_dif) / 2
add_block('simulink/Math Operations/Sum', [mdl '/Sum_VL'], ...
    'Inputs', '+-', 'Position', [500 490 520 510]);
add_block('simulink/Math Operations/Gain', [mdl '/Gain_VL'], ...
    'Gain', '0.5', 'Position', [570 492 600 508]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_VL'], ...
    'Position', [650 492 680 508]);

%% Conexiones voltajes fisicos
add_line(mdl, ph('Sat_pitch').Outport(1), ph('Sum_VR').Inport(1));
add_line(mdl, ph('Sat_yaw').Outport(1),  ph('Sum_VR').Inport(2));
add_line(mdl, ph('Sat_pitch').Outport(1), ph('Sum_VL').Inport(1));
add_line(mdl, ph('Sat_yaw').Outport(1),  ph('Sum_VL').Inport(2));
add_line(mdl, ph('Sum_VR').Outport(1),   ph('Gain_VR').Inport(1));
add_line(mdl, ph('Sum_VL').Outport(1),   ph('Gain_VL').Inport(1));
add_line(mdl, ph('Gain_VR').Outport(1),  ph('Scope_VR').Inport(1));
add_line(mdl, ph('Gain_VL').Outport(1),  ph('Scope_VL').Inport(1));

%% Configuracion
set_param(mdl, 'StopTime', '5', 'Solver', 'ode45', 'MaxStep', '0.01');

%% Guardar
save_system(mdl);