%% ========================================================================
%  validacion_robusta_final.m — Planta Lineal vs No Lineal con LQR Ideal
%  MATLAB 2025b | Generación Automática de Simulink y Dark Mode
% =========================================================================

clear; clc; close all;
% Apagamos el warning de Simulink para mantener la consola limpia
warning('off', 'Simulink:Engine:MdlFileShadowing');

%% S0 a S11 — DERIVACIÓN MATEMÁTICA Y ESPACIO DE ESTADOS
fprintf('1. Derivando modelo matemático (incluyendo perturbación T_d)...\n');
syms theta dtheta ddtheta real
syms x dx ddx real
syms alpha dalpha ddalpha real
syms M m r d l g real
syms Icy Icz Icx Iw Iwz real
syms alpha_m beta_m real
syms V_R V_L T_d real % T_d = Torque de perturbación externa

p = struct('M',80, 'm',2, 'r',0.20, 'd',0.60, 'l',0.90, 'g',9.81, ...
           'Icy',10, 'Icz',12, 'Icx',12, 'Iw',0.08, 'Iwz',0.04, ...
           'alpha_m',2.0, 'beta_m',1.5);

dq = [dtheta; dx; dalpha];
v_G = [dx + l*dtheta*cos(theta); l*dalpha*sin(theta); -l*dtheta*sin(theta)];
omega_C = [-dalpha*sin(theta); dtheta; dalpha*cos(theta)];

T_cuerpo = (1/2)*M*(v_G.'*v_G) + (1/2)*(Icx*omega_C(1)^2 + Icy*omega_C(2)^2 + Icz*omega_C(3)^2);
T_ruedas = (1/2)*m*((dx + d/2*dalpha)^2 + (dx - d/2*dalpha)^2) + (1/2)*Iw*((dx/r + d/(2*r)*dalpha)^2 + (dx/r - d/(2*r)*dalpha)^2) + Iwz*dalpha^2;
T = simplify(expand(T_cuerpo + T_ruedas));
V_pot = M*g*l*cos(theta);

M_mat = simplify(jacobian(jacobian(T, dq).', dq));
q_vec = [theta; x; alpha];
n = 3;
dM_dq = sym(zeros(n, n, n));
for k = 1:n, dM_dq(:,:,k) = diff(M_mat, q_vec(k)); end
C_mat = sym(zeros(n));
for i = 1:n
    for j = 1:n
        Gamma_ij = (1/2) * (squeeze(dM_dq(i,j,:)) + squeeze(dM_dq(i,:,j)).' - squeeze(dM_dq(j,:,i)).');
        C_mat(i,j) = Gamma_ij.' * dq;
    end
end
C_mat = simplify(C_mat);

omega_Rw = dx/r + d/(2*r)*dalpha;
omega_Lw = dx/r - d/(2*r)*dalpha;
tau_R = alpha_m*V_R - beta_m*(omega_Rw - dtheta);
tau_L = alpha_m*V_L - beta_m*(omega_Lw - dtheta);

% INYECCIÓN DE LA PERTURBACIÓN (T_d) EN EL CABECEO
Q_nc = simplify([ -(tau_R + tau_L) + T_d; 
                   (tau_R + tau_L)/r;
                   d*(tau_R - tau_L)/(2*r) ]);

G_vec = jacobian(V_pot, q_vec).';
f_rhs = simplify(Q_nc - C_mat*dq - G_vec);
M_ca   = M_mat(1:2, 1:2);
det_ca = simplify(M_ca(1,1)*M_ca(2,2) - M_ca(1,2)^2);

ddtheta_eq = simplify(( M_ca(2,2)*f_rhs(1) - M_ca(1,2)*f_rhs(2)) / det_ca);
ddx_eq     = simplify((-M_ca(1,2)*f_rhs(1) + M_ca(1,1)*f_rhs(2)) / det_ca);
ddalpha_eq = simplify(f_rhs(3) / M_mat(3,3));

X_sym = [theta; dtheta; x; dx; alpha; dalpha];
U_sym = [V_R; V_L];
f_nl = [dtheta; ddtheta_eq; dx; ddx_eq; dalpha; ddalpha_eq];

A_sym = jacobian(f_nl, X_sym);
B_sym = jacobian(f_nl, U_sym);
Bd_sym = jacobian(f_nl, T_d); % Matriz B para la perturbación

eq_subs = [X_sym; U_sym; T_d];
eq_zeros = zeros(9,1);
A_lin = simplify(subs(A_sym, eq_subs, eq_zeros));
B_lin = simplify(subs(B_sym, eq_subs, eq_zeros));
Bd_lin = simplify(subs(Bd_sym, eq_subs, eq_zeros));

sym_list = [M, m, r, d, l, g, Icy, Icz, Icx, Iw, Iwz, alpha_m, beta_m];
num_list = [p.M, p.m, p.r, p.d, p.l, p.g, p.Icy, p.Icz, p.Icx, p.Iw, p.Iwz, p.alpha_m, p.beta_m];

A_num = double(subs(A_lin, sym_list, num_list));
B_num = double(subs(B_lin, sym_list, num_list));
Bd_num = double(subs(Bd_lin, sym_list, num_list));

A_num(abs(A_num) < 1e-10) = 0; 
B_num(abs(B_num) < 1e-10) = 0; 
Bd_num(abs(Bd_num) < 1e-10) = 0;

%% S12 — DISEÑO DEL CONTROLADOR LQR ESTRICTO
fprintf('2. Diseñando LQR (Control principal en Cabeceo y Velocidad)...\n');
% Pesos: [Theta, dTheta, x, dx, alpha, dalpha]
% Evitamos singularidad algorítmica con 1e-4 en posición y giro
Q = diag([2000, 100, 1e-4, 50, 1e-4, 1e-4]); 
R = diag([1, 1]); 

K = lqr(A_num, B_num, Q, R);
B_aug = [B_num, Bd_num]; % Matriz B aumentada [U, Td]
C_num = eye(6); 
D_aug = zeros(6, 3);

%% S13 — GENERACIÓN AUTOMÁTICA DE SIMULINK
modelName = 'LQR_Lineal_vs_NoLineal';
fprintf('3. Construyendo Simulink de Validación: %s.slx...\n', modelName);
if bdIsLoaded(modelName), close_system(modelName, 0); end
new_system(modelName); open_system(modelName);

% --- 1. Bloque de Perturbación (Torque T_d) ---
add_block('simulink/Sources/Clock', [modelName '/Reloj'], 'Position', [30 180 60 210]);
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Generador_Td'], 'Position', [100 170 200 220]);
codigo_td = sprintf('function Td = fcn(t)\n    %% Perturbación de 15 Nm entre 2s y 6s\n    if t >= 2 && t <= 6\n        Td = 15;\n    else\n        Td = 0;\n    end\nend');
rt = sfroot; blk_td = rt.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/Generador_Td']);
blk_td.Script = codigo_td;
add_line(modelName, 'Reloj/1', 'Generador_Td/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/Goto', [modelName '/Goto_Td'], 'Position', [230 180 280 210]);
set_param([modelName '/Goto_Td'], 'GotoTag', 'TD');
add_line(modelName, 'Generador_Td/1', 'Goto_Td/1', 'autorouting', 'on');

% --- 2. Lazo Planta Lineal ---
add_block('simulink/Signal Routing/From', [modelName '/From_X_Lin'], 'Position', [30 30 80 60]);
set_param([modelName '/From_X_Lin'], 'GotoTag', 'X_LIN');
add_block('simulink/Math Operations/Gain', [modelName '/K_Lin'], 'Position', [110 30 180 60]);
set_param([modelName '/K_Lin'], 'Gain', '-K', 'Multiplication', 'Matrix(K*u)');
add_block('simulink/Discontinuities/Saturation', [modelName '/Sat_Lin'], 'Position', [210 30 240 60]);
set_param([modelName '/Sat_Lin'], 'UpperLimit', '24', 'LowerLimit', '-24');

add_block('simulink/Signal Routing/From', [modelName '/From_Td_Lin'], 'Position', [210 80 240 100]);
set_param([modelName '/From_Td_Lin'], 'GotoTag', 'TD');
add_block('simulink/Commonly Used Blocks/Mux', [modelName '/Mux_Lin'], 'Position', [280 40 285 90]);
set_param([modelName '/Mux_Lin'], 'Inputs', '2');
add_block('simulink/Continuous/State-Space', [modelName '/Planta_Lineal'], 'Position', [330 47 430 83]);
set_param([modelName '/Planta_Lineal'], 'A', 'A_num', 'B', 'B_aug', 'C', 'C_num', 'D', 'D_aug', 'X0', 'zeros(6,1)');

add_block('simulink/Signal Routing/Goto', [modelName '/Goto_X_Lin'], 'Position', [480 50 530 80]);
set_param([modelName '/Goto_X_Lin'], 'GotoTag', 'X_LIN');
add_block('simulink/Sinks/To Workspace', [modelName '/Out_Lin'], 'Position', [480 100 550 130]);
set_param([modelName '/Out_Lin'], 'VariableName', 'sim_Lin', 'SaveFormat', 'Array');

add_line(modelName, 'From_X_Lin/1', 'K_Lin/1', 'autorouting', 'on');
add_line(modelName, 'K_Lin/1', 'Sat_Lin/1', 'autorouting', 'on');
add_line(modelName, 'Sat_Lin/1', 'Mux_Lin/1', 'autorouting', 'on');
add_line(modelName, 'From_Td_Lin/1', 'Mux_Lin/2', 'autorouting', 'on');
add_line(modelName, 'Mux_Lin/1', 'Planta_Lineal/1', 'autorouting', 'on');
add_line(modelName, 'Planta_Lineal/1', 'Goto_X_Lin/1', 'autorouting', 'on');
add_line(modelName, 'Planta_Lineal/1', 'Out_Lin/1', 'autorouting', 'on');

% --- 3. Lazo Planta No Lineal ---
add_block('simulink/Signal Routing/From', [modelName '/From_X_NL'], 'Position', [30 280 80 310]);
set_param([modelName '/From_X_NL'], 'GotoTag', 'X_NL');
add_block('simulink/Math Operations/Gain', [modelName '/K_NL'], 'Position', [110 280 180 310]);
set_param([modelName '/K_NL'], 'Gain', '-K', 'Multiplication', 'Matrix(K*u)');
add_block('simulink/Discontinuities/Saturation', [modelName '/Sat_NL'], 'Position', [210 280 240 310]);
set_param([modelName '/Sat_NL'], 'UpperLimit', '24', 'LowerLimit', '-24');
add_block('simulink/Signal Routing/From', [modelName '/From_Td_NL'], 'Position', [210 330 240 350]);
set_param([modelName '/From_Td_NL'], 'GotoTag', 'TD');

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Planta_NoLineal'], 'Position', [330 280 430 350]);
codigo_nl = sprintf([...
    'function dX = fcn(X, U, Td)\n',...
    '    %% Parámetros Físicos\n',...
    '    M = 80; m = 2; r = 0.20; d = 0.60; l = 0.90; g = 9.81;\n',...
    '    Icy = 10; Icz = 12; Icx = 12; Iw = 0.08; Iwz = 0.04;\n',...
    '    alpha_m = 2.0; beta_m = 1.5;\n',...
    '    %% Desglose de Entradas\n',...
    '    theta = X(1); dtheta = X(2); dx = X(4); dalpha = X(6);\n',...
    '    VR = U(1); VL = U(2);\n',...
    '    %% Dinámica Física No Lineal\n',...
    '    omega_Rw = dx/r + d/(2*r)*dalpha; omega_Lw = dx/r - d/(2*r)*dalpha;\n',...
    '    tau_R = alpha_m*VR - beta_m*(omega_Rw - dtheta);\n',...
    '    tau_L = alpha_m*VL - beta_m*(omega_Lw - dtheta);\n',...
    '    M11 = Icy + M*l^2; M12 = M*l*cos(theta); M22 = M + 2*m + 2*Iw/r^2;\n',...
    '    detM = M11*M22 - M12^2;\n',...
    '    F1 = M*g*l*sin(theta) - (tau_R + tau_L) + Td;\n',...
    '    F2 = M*l*dtheta^2*sin(theta) + (tau_R + tau_L)/r;\n',...
    '    ddtheta = ( M22*F1 - M12*F2) / detM;\n',...
    '    ddx     = (-M12*F1 + M11*F2) / detM;\n',...
    '    M33 = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r)^2) + Iwz;\n',...
    '    ddalpha = (d*(tau_R - tau_L)/(2*r)) / M33;\n',...
    '    dX = [dtheta; ddtheta; dx; ddx; dalpha; ddalpha];\n',...
    'end']);
rt = sfroot; blk_nl = rt.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/Planta_NoLineal']);
blk_nl.Script = codigo_nl;

add_block('simulink/Continuous/Integrator', [modelName '/Int_NL'], 'Position', [480 300 510 330]);
set_param([modelName '/Int_NL'], 'InitialCondition', 'zeros(6,1)');

add_block('simulink/Signal Routing/Goto', [modelName '/Goto_X_NL'], 'Position', [550 270 600 300]);
set_param([modelName '/Goto_X_NL'], 'GotoTag', 'X_NL');
add_block('simulink/Sinks/To Workspace', [modelName '/Out_NL'], 'Position', [550 330 620 360]);
set_param([modelName '/Out_NL'], 'VariableName', 'sim_NL', 'SaveFormat', 'Array');

% Ruteo explícito por puertos para la planta no lineal
pH_NL_Fun = get_param([modelName '/Planta_NoLineal'], 'PortHandles');
pH_NL_Int = get_param([modelName '/Int_NL'], 'PortHandles');
pH_Sat_NL = get_param([modelName '/Sat_NL'], 'PortHandles');
pH_Td_NL = get_param([modelName '/From_Td_NL'], 'PortHandles');
pH_X_NL = get_param([modelName '/From_X_NL'], 'PortHandles');
pH_K_NL = get_param([modelName '/K_NL'], 'PortHandles');

add_line(modelName, pH_X_NL.Outport(1), pH_K_NL.Inport(1), 'autorouting', 'on');
add_line(modelName, pH_K_NL.Outport(1), pH_Sat_NL.Inport(1), 'autorouting', 'on');
add_line(modelName, pH_NL_Int.Outport(1), pH_NL_Fun.Inport(1), 'autorouting', 'on'); % Feedback X a Pto 1
add_line(modelName, pH_Sat_NL.Outport(1), pH_NL_Fun.Inport(2), 'autorouting', 'on'); % Voltaje U a Pto 2
add_line(modelName, pH_Td_NL.Outport(1), pH_NL_Fun.Inport(3), 'autorouting', 'on');  % Perturb. Td a Pto 3
add_line(modelName, pH_NL_Fun.Outport(1), pH_NL_Int.Inport(1), 'autorouting', 'on'); % dX a Integrador
add_line(modelName, pH_NL_Int.Outport(1), get_param([modelName '/Goto_X_NL'], 'PortHandles').Inport(1), 'autorouting', 'on');
add_line(modelName, pH_NL_Int.Outport(1), get_param([modelName '/Out_NL'], 'PortHandles').Inport(1), 'autorouting', 'on');

%% S14 — SIMULACIÓN Y GRÁFICAS
fprintf('4. Ejecutando simulación de 10 segundos...\n');
set_param(modelName, 'Solver', 'ode45', 'StopTime', '10');
save_system(modelName); 
out = sim(modelName);

% Extracción segura de datos
if isstruct(out.sim_Lin)
    X_L = out.sim_Lin.signals.values; t = out.sim_Lin.time;
else
    X_L = out.sim_Lin; t = out.tout; 
end

if isstruct(out.sim_NL)
    X_NL = out.sim_NL.signals.values;
else
    X_NL = out.sim_NL; 
end

if ndims(X_L) >= 3
    X_L = squeeze(X_L); X_NL = squeeze(X_NL); 
end
if size(X_L,1) ~= length(t)
    X_L = X_L.'; X_NL = X_NL.'; 
end

fprintf('5. Generando comparativa visual...\n');
set(groot, 'defaultFigureColor', [0.15 0.15 0.15], 'defaultAxesColor', [0.1 0.1 0.1], ...
    'defaultAxesXColor', [0.9 0.9 0.9], 'defaultAxesYColor', [0.9 0.9 0.9], ...
    'defaultAxesGridColor', [0.3 0.3 0.3], 'defaultLineLineWidth', 1.5, ...
    'defaultTextInterpreter', 'latex', 'defaultLegendInterpreter', 'latex', ...
    'defaultLegendColor', [0.15 0.15 0.15], 'defaultLegendTextColor', [0.9 0.9 0.9]);

figure('Name', 'LQR: Lineal vs No Lineal', 'Position', [100, 100, 1000, 450]);

% Cabeceo
subplot(1,2,1);
plot(t, rad2deg(X_L(:,1)), '--', 'Color', '#FF3333'); hold on;
plot(t, rad2deg(X_NL(:,1)), '-', 'Color', '#FF8800');
xline(2, 'w:', 'Inicio Perturb.', 'HandleVisibility', 'off');
xline(6, 'w:', 'Fin Perturb.', 'HandleVisibility', 'off');
title('Rechazo de Perturbación ($\theta$)');
xlabel('Tiempo (s)'); ylabel('Grados ($^\circ$)');
legend('Modelo Lineal', 'Modelo No Lineal', 'Location', 'best'); grid on;

% Avance
subplot(1,2,2);
plot(t, X_L(:,3), '--', 'Color', '#33CCFF'); hold on;
plot(t, X_NL(:,3), '-', 'Color', '#00FF00');
xline(2, 'w:', 'HandleVisibility', 'off');
xline(6, 'w:', 'HandleVisibility', 'off');
title('Deriva de Posición Libre ($x$)');
xlabel('Tiempo (s)'); ylabel('Metros (m)');
legend('Modelo Lineal', 'Modelo No Lineal', 'Location', 'best'); grid on;

fprintf('\n=== VALIDACIÓN LQR COMPLETADA ===\n');
warning('on', 'Simulink:Engine:MdlFileShadowing');