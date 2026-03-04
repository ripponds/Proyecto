%% ========================================================================
%  segway_controlador.m  —  SCRIPT SECUNDARIO
%
%  NO ejecutar directamente. Llamado desde segway_sim_lazo_abierto.m
%  Requiere en workspace: A_num, B_num, p, params_vec
%
%  1. Verifica controlabilidad
%  2. Diseña controlador por pole placement
%  3. Construye modelo Simulink completo con planta no lineal
% =========================================================================

fprintf('P5: Verificando workspace...\n')
if ~exist('A_num','var') || ~exist('B_num','var') || ~exist('p','var')
    error('Ejecutar segway_sim_lazo_abierto.m primero.')
end
fprintf('    A_num, B_num, p — OK\n\n')

A = A_num;
B = B_num;
n = size(A,1);
m = size(B,2);

%% ── P6: CONTROLABILIDAD ──────────────────────────────────────────────────
fprintf('P6: Controlabilidad...\n')
Co    = ctrb(A, B);
rango = rank(Co);
sv    = svd(Co);
fprintf('    Rango: %d/%d', rango, n)
if rango == n
    fprintf('  OK\n')
else
    fprintf('  ERROR — sistema no controlable\n')
    error('Pole placement no aplicable.')
end
fprintf('    Numero de condicion: %.2e\n\n', sv(1)/sv(end))

%% ── P7: POLE PLACEMENT ───────────────────────────────────────────────────
% Ajustar polos aquí para sintonizar el controlador.
% Regla: partes reales al menos 3x más negativas que el polo inestable.
% Polo inestable lazo abierto ≈ +4.9  →  mínimo -15

fprintf('P7: Pole placement...\n')

polos_deseados = [ -15 + 5j;   % cabeceo — rápido, amortiguado
                   -15 - 5j;
                    -8;        % avance  — más lento
                    -9;
                    -6 + 2j;   % yaw     — moderado
                    -6 - 2j ];

K   = place(A, B, polos_deseados);
Alc = A - B*K;
plc = eig(Alc);

fprintf('    Polos lazo cerrado:\n')
todos_ok = true;
for k = 1:n
    ok = real(plc(k)) < 0;
    if ~ok, todos_ok = false; end
    fprintf('      plc%d = %+.4f', k, real(plc(k)))
    if abs(imag(plc(k))) > 1e-6
        fprintf(' %+.4fj', imag(plc(k)))
    end
    if ok, fprintf('  OK'); else, fprintf('  INESTABLE'); end
    fprintf('\n')
end

if todos_ok
    fprintf('    Todos los polos estables. OK\n')
else
    warning('Hay polos inestables — revisar polos_deseados.')
end

fprintf('\n    K [2x6]:\n')
fprintf('              theta    dtheta    x        dx       alpha    dalpha\n')
for i = 1:2
    if i==1, lbl='V_R'; else, lbl='V_L'; end
    fprintf('      %s:  ', lbl)
    fprintf(' %+8.4f', K(i,:))
    fprintf('\n')
end
fprintf('\n')

%% ── P8: GENERAR segway_sfcn.m ────────────────────────────────────────────
fprintf('P8: Generando segway_sfcn.m...\n')

fid = fopen('segway_sfcn.m','w');
fprintf(fid,'function Xdot = segway_sfcn(u)\n');
fprintf(fid,'%% Planta no lineal Segway — Interpreted MATLAB Function\n');
fprintf(fid,'%% u = [V_R(1); V_L(2); params(3..15); X(16..21)]\n\n');
fprintf(fid,'Xdot  = zeros(6,1);\n');
fprintf(fid,'V_R   = u(1);  V_L   = u(2);\n');
fprintf(fid,'M_b   = u(3);  m_w   = u(4);  r     = u(5);\n');
fprintf(fid,'d     = u(6);  l     = u(7);  g     = u(8);\n');
fprintf(fid,'Icy   = u(9);  Icz   = u(10); Icx   = u(11);\n');
fprintf(fid,'Iw    = u(12); Iwz   = u(13);\n');
fprintf(fid,'alp_m = u(14); bet_m = u(15);\n');
fprintf(fid,'theta  = u(16); dtheta = u(17);\n');
fprintf(fid,'dx     = u(19); dalpha = u(21);\n\n');
fprintf(fid,'omR   = dx/r + d/(2*r)*dalpha;\n');
fprintf(fid,'omL   = dx/r - d/(2*r)*dalpha;\n');
fprintf(fid,'tau_R = alp_m*V_R - bet_m*(omR - dtheta);\n');
fprintf(fid,'tau_L = alp_m*V_L - bet_m*(omL - dtheta);\n\n');
fprintf(fid,'M11 = M_b*l^2 + Icy;\n');
fprintf(fid,'M12 = M_b*l*cos(theta);\n');
fprintf(fid,'M22 = (M_b + 2*m_w) + 2*Iw/r^2;\n');
fprintf(fid,'M33 = (M_b + 2*m_w)*d^2/4 + 2*Iwz + Icz;\n\n');
fprintf(fid,'h1 = M_b*g*l*sin(theta) - (tau_R + tau_L);\n');
fprintf(fid,'h2 = M_b*l*sin(theta)*dtheta^2 + (tau_R + tau_L)/r;\n');
fprintf(fid,'h3 = d*(tau_R - tau_L)/(2*r);\n\n');
fprintf(fid,'det_M   = M11*M22 - M12*M12;\n');
fprintf(fid,'Xdot(1) = dtheta;\n');
fprintf(fid,'Xdot(2) = ( M22*h1 - M12*h2) / det_M;\n');
fprintf(fid,'Xdot(3) = dx;\n');
fprintf(fid,'Xdot(4) = (-M12*h1 + M11*h2) / det_M;\n');
fprintf(fid,'Xdot(5) = dalpha;\n');
fprintf(fid,'Xdot(6) = h3 / M33;\n');
fclose(fid);
fprintf('    segway_sfcn.m generado. OK\n\n')

%% ── P9: CONSTRUIR MODELO SIMULINK ────────────────────────────────────────
fprintf('P9: Construyendo modelo Simulink...\n')

mdl = 'segway_control';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
open_system(mdl);

set_param(mdl, ...
    'StopTime',       '5', ...
    'SolverType',     'Variable-step', ...
    'Solver',         'ode45', ...
    'RelTol',         '1e-6', ...
    'AbsTol',         '1e-8', ...
    'SaveTime',       'on',  'TimeSaveName',   'tout', ...
    'SaveOutput',     'on',  'OutputSaveName', 'yout', ...
    'SaveFormat',     'Array')

% ── Bloques ───────────────────────────────────────────────────────────────

% Referencia (6x1 ceros — theta_ref=0, x_ref=0, alpha_ref=0)
add_block('simulink/Sources/Constant', [mdl '/Referencia'], ...
    'Position', [30 185 110 215], ...
    'Value', 'zeros(6,1)')

% Suma error: e = X_ref - X
add_block('simulink/Math Operations/Sum', [mdl '/Suma_e'], ...
    'Position', [150 188 175 212], ...
    'Inputs', '+-', 'IconShape', 'round')

% Ganancia K
add_block('simulink/Math Operations/Gain', [mdl '/Ctrl_K'], ...
    'Position', [210 180 310 220], ...
    'Gain', 'K', ...
    'Multiplication', 'Matrix(K*u)')

% Saturación voltaje motor ±12V
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_V'], ...
    'Position', [340 183 400 217], ...
    'UpperLimit', '12', 'LowerLimit', '-12')

% Mux que combina [U(2); params(13); X(6)] = 21 entradas a la planta
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Planta'], ...
    'Position', [430 160 450 260], ...
    'Inputs', '3')

% Constante parámetros
add_block('simulink/Sources/Constant', [mdl '/Params'], ...
    'Position', [340 255 420 285], ...
    'Value', 'params_vec')

% Planta no lineal
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [mdl '/Planta'], ...
    'Position', [470 170 590 250], ...
    'MATLABFcn', 'segway_sfcn', ...
    'OutputDimensions', '6')

% Integrador de estados (condición inicial = pequeño theta)
add_block('simulink/Continuous/Integrator', [mdl '/Integrador'], ...
    'Position', [620 178 680 242], ...
    'InitialCondition', '[0.05; 0; 0; 0; 0; 0]')

% Demux salida 6 estados
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
    'Position', [710 180 725 240], ...
    'Outputs', '6')

% Scopes
add_block('simulink/Sinks/Scope', [mdl '/Scope_Cabeceo'], ...
    'Position', [760 175 810 205], 'NumInputPorts', '2')
add_block('simulink/Sinks/Scope', [mdl '/Scope_Avance'], ...
    'Position', [760 210 810 235], 'NumInputPorts', '2')
add_block('simulink/Sinks/Scope', [mdl '/Scope_Yaw'], ...
    'Position', [760 243 810 268], 'NumInputPorts', '2')

% To Workspace — estados
add_block('simulink/Sinks/To Workspace', [mdl '/WS_X'], ...
    'Position', [760 278 840 303], ...
    'VariableName', 'X_ctrl', ...
    'SaveFormat', 'Array', 'MaxDataPoints', 'inf')

% To Workspace — control
add_block('simulink/Sinks/To Workspace', [mdl '/WS_U'], ...
    'Position', [340 308 420 333], ...
    'VariableName', 'U_ctrl', ...
    'SaveFormat', 'Array', 'MaxDataPoints', 'inf')

fprintf('    Bloques creados. OK\n')

% ── Conexiones ────────────────────────────────────────────────────────────

add_line(mdl, 'Referencia/1',  'Suma_e/1',      'autorouting','on')
add_line(mdl, 'Suma_e/1',      'Ctrl_K/1',      'autorouting','on')
add_line(mdl, 'Ctrl_K/1',      'Sat_V/1',       'autorouting','on')
add_line(mdl, 'Sat_V/1',       'Mux_Planta/1',  'autorouting','on')
add_line(mdl, 'Params/1',      'Mux_Planta/2',  'autorouting','on')
add_line(mdl, 'Integrador/1',  'Mux_Planta/3',  'autorouting','on')
add_line(mdl, 'Mux_Planta/1',  'Planta/1',      'autorouting','on')
add_line(mdl, 'Planta/1',      'Integrador/1',  'autorouting','on')
add_line(mdl, 'Integrador/1',  'Suma_e/2',      'autorouting','on')
add_line(mdl, 'Integrador/1',  'Demux/1',       'autorouting','on')
add_line(mdl, 'Integrador/1',  'WS_X/1',        'autorouting','on')
add_line(mdl, 'Sat_V/1',       'WS_U/1',        'autorouting','on')
add_line(mdl, 'Demux/1',       'Scope_Cabeceo/1','autorouting','on')
add_line(mdl, 'Demux/2',       'Scope_Cabeceo/2','autorouting','on')
add_line(mdl, 'Demux/3',       'Scope_Avance/1', 'autorouting','on')
add_line(mdl, 'Demux/4',       'Scope_Avance/2', 'autorouting','on')
add_line(mdl, 'Demux/5',       'Scope_Yaw/1',    'autorouting','on')
add_line(mdl, 'Demux/6',       'Scope_Yaw/2',    'autorouting','on')

fprintf('    Conexiones realizadas. OK\n')

save_system(mdl)
fprintf('    Modelo guardado: %s.slx\n\n', mdl)

%% ── P10: REPORTE FINAL ───────────────────────────────────────────────────
fprintf('══════════════════════════════════════════════════\n')
fprintf('  LISTO\n')
fprintf('══════════════════════════════════════════════════\n')
fprintf('  Modelo Simulink:  %s.slx\n', mdl)
fprintf('  Ganancia K:       en workspace\n')
fprintf('\n  Para simular:\n')
fprintf('    >> sim(''%s'')\n', mdl)
fprintf('\n  Para cambiar condicion inicial:\n')
fprintf('    Doble click en bloque Integrador\n')
fprintf('\n  Para cambiar referencia:\n')
fprintf('    Doble click en bloque Referencia\n')
fprintf('\n  Para resintonizar polos:\n')
fprintf('    Editar polos_deseados en P7 de segway_controlador.m\n')
fprintf('    y correr segway_sim_lazo_abierto.m de nuevo\n')
fprintf('══════════════════════════════════════════════════\n')
