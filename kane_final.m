%% ========================================================================
%  segway_modelo_optimizado.m — Modelo dinámico del vehículo autobalanceado
%  Optimizado para MATLAB 2025b (Vectorización Simbólica)
%
%  Estados: X = [theta; dtheta; x; dx; alpha; dalpha]
%  Entradas: U = [V_R; V_L]
%
%  Convención:
%    theta = 0 → erguido,  theta > 0 → inclinación adelante
%    alpha > 0 → giro izquierda (regla mano derecha sobre vertical)
%    a1 = avance, a2 = lateral izquierda, a3 = vertical arriba
% =========================================================================

%% S0 — CONFIGURACIÓN VISUAL Y PARÁMETROS
clear; clc; close all;

% Configuración Dark Mode 
set(groot, 'defaultFigureColor', [0.15 0.15 0.15]);
set(groot, 'defaultAxesColor', [0.1 0.1 0.1]);
set(groot, 'defaultAxesXColor', [0.9 0.9 0.9]);
set(groot, 'defaultAxesYColor', [0.9 0.9 0.9]);
set(groot, 'defaultAxesGridColor', [0.3 0.3 0.3]);
set(groot, 'defaultLineLineWidth', 1.5);

fprintf('S0: Parámetros y variables simbólicas\n')
syms theta dtheta ddtheta real
syms x dx ddx real
syms alpha dalpha ddalpha real
syms M m r d l g real
syms Icy Icz Icx Iw Iwz real
syms alpha_m beta_m real
syms V_R V_L real

p.M       = 80;        % kg     cuerpo + persona
p.m       = 2;         % kg     masa una rueda
p.r       = 0.20;      % m      radio rueda
p.d       = 0.60;      % m      separación entre ruedas
p.l       = 0.90;      % m      eje ruedas → CM (positivo = arriba)
p.g       = 9.81;      % m/s²
p.Icy     = 10;        % kg·m²  inercia cabeceo
p.Icz     = 12;        % kg·m²  inercia giro
p.Icx     = 12;        % kg·m²  inercia alabeo
p.Iw      = 0.08;      % kg·m²  inercia axial rueda
p.Iwz     = 0.04;      % kg·m²  inercia transversal rueda
p.alpha_m = 2.0;       % N·m/V
p.beta_m  = 1.5;       % N·m·s/rad

%% S1 — ENERGÍA CINÉTICA Y POTENCIAL
fprintf('S1: Energías\n')
dq = [dtheta; dx; dalpha];

% Velocidad del CM del cuerpo en marco A [a1, a2, a3]
v_G = [ dx + l*dtheta*cos(theta);
        l*dalpha*sin(theta);
       -l*dtheta*sin(theta)];

% Velocidad angular del cuerpo en marco C [c1, c2, c3]
omega_C = [-dalpha*sin(theta);
            dtheta;
            dalpha*cos(theta)];

% Energía cinética
T_cuerpo = (1/2)*M*(v_G.'*v_G) + (1/2)*(Icx*omega_C(1)^2 + Icy*omega_C(2)^2 + Icz*omega_C(3)^2);
T_ruedas = (1/2)*m*((dx + d/2*dalpha)^2 + (dx - d/2*dalpha)^2) ...
         + (1/2)*Iw*((dx/r + d/(2*r)*dalpha)^2 + (dx/r - d/(2*r)*dalpha)^2) ...
         + Iwz*dalpha^2;
T = simplify(expand(T_cuerpo + T_ruedas));

% Energía potencial
V_pot = M*g*l*cos(theta);

%% S2 — MATRIZ DE MASA
fprintf('S2: Matriz de masa (Jacobiano optimizado)\n')
M_mat = simplify(jacobian(jacobian(T, dq).', dq));

% Verificaciones lógicas (corregidas para MATLAB 2025b con 'all')
assert(all(isAlways(M_mat == M_mat.'), 'all'), 'ERROR: M no simétrica');
assert(isAlways(M_mat(1,3) == 0) && isAlways(M_mat(2,3) == 0), 'ERROR: Giro acoplado');

%% S3 — MATRIZ DE CORIOLIS (Christoffel Vectorizado)
fprintf('S3: Coriolis (Cálculo tensorial vectorizado)\n')
q_vec = [theta; x; alpha];
n = 3;

% Pre-cálculo del gradiente de M_mat (Tensor 3D)
dM_dq = sym(zeros(n, n, n));
for k = 1:n
    dM_dq(:,:,k) = diff(M_mat, q_vec(k));
end

C_mat = sym(zeros(n));
for i = 1:n
    for j = 1:n
        % Combinación lineal rápida
        Gamma_ij = (1/2) * (squeeze(dM_dq(i,j,:)) + squeeze(dM_dq(i,:,j)).' - squeeze(dM_dq(j,:,i)).');
        C_mat(i,j) = Gamma_ij.' * dq;
    end
end
C_mat = simplify(C_mat);

% Verificación de antisimetría
dMdt_vec = jacobian(M_mat(:), q_vec) * dq;
dMdt = reshape(dMdt_vec, n, n);
N_test = simplify(dMdt - 2*C_mat);
assert(all(isAlways(N_test + N_test.' == 0), 'all'), 'ERROR: dM/dt−2C no antisimétrica');

%% S4 — FUERZAS GENERALIZADAS
fprintf('S4: Fuerzas generalizadas\n')
omega_Rw = dx/r + d/(2*r)*dalpha;
omega_Lw = dx/r - d/(2*r)*dalpha;
tau_R = alpha_m*V_R - beta_m*(omega_Rw - dtheta);
tau_L = alpha_m*V_L - beta_m*(omega_Lw - dtheta);

Q_nc = simplify([ -(tau_R + tau_L);
                   (tau_R + tau_L)/r;
                   d*(tau_R - tau_L)/(2*r) ]);

%% S5 — ECUACIONES DE MOVIMIENTO
fprintf('S5: Ecuaciones de movimiento\n')
G_vec = jacobian(V_pot, q_vec).';
f_rhs = simplify(Q_nc - C_mat*dq - G_vec);

M_ca   = M_mat(1:2, 1:2);
det_ca = simplify(M_ca(1,1)*M_ca(2,2) - M_ca(1,2)^2);

ddtheta_eq = simplify(( M_ca(2,2)*f_rhs(1) - M_ca(1,2)*f_rhs(2)) / det_ca);
ddx_eq     = simplify((-M_ca(1,2)*f_rhs(1) + M_ca(1,1)*f_rhs(2)) / det_ca);
ddalpha_eq = simplify(f_rhs(3) / M_mat(3,3));

%% S6 & S7 — ESPACIO DE ESTADOS Y LINEALIZACIÓN
fprintf('S6-S7: Espacio de estados y Linealización analítica\n')
X_sym = [theta; dtheta; x; dx; alpha; dalpha];
U_sym = [V_R; V_L];
f_nl = [dtheta; ddtheta_eq; dx; ddx_eq; dalpha; ddalpha_eq];

% Jacobiano evaluado directamente en el equilibrio
A_sym = jacobian(f_nl, X_sym);
B_sym = jacobian(f_nl, U_sym);
A_lin = simplify(subs(A_sym, [X_sym; U_sym], zeros(8,1)));
B_lin = simplify(subs(B_sym, [X_sym; U_sym], zeros(8,1)));

%% S8 — EVALUACIÓN NUMÉRICA
fprintf('S8: Evaluación numérica rápida\n')
sym_list = [M, m, r, d, l, g, Icy, Icz, Icx, Iw, Iwz, alpha_m, beta_m];
num_list = [p.M, p.m, p.r, p.d, p.l, p.g, p.Icy, p.Icz, p.Icx, p.Iw, p.Iwz, p.alpha_m, p.beta_m];

A_num = double(subs(A_lin, sym_list, num_list));
B_num = double(subs(B_lin, sym_list, num_list));

% Limpieza de ruido numérico
A_num(abs(A_num) < 1e-10) = 0;
B_num(abs(B_num) < 1e-10) = 0;

C_num = eye(6);
D_num = zeros(6, 2);

disp('  A ='); disp(A_num)
disp('  B ='); disp(B_num)

polos = eig(A_num);
if any(real(polos) > 1e-6)
    fprintf('  Sistema INESTABLE (pendulo invertido) OK\n')
end

%% S9 & S10 — SUBSISTEMAS Y CONTROLABILIDAD
fprintf('\nS9-S10: Subsistemas y Controlabilidad\n')
A_ca   = A_num(1:4, 1:4);   B_ca   = B_num(1:4, :);
A_giro = A_num(5:6, 5:6);   B_giro = B_num(5:6, :);

fprintf('  Completo:         rank = %d/6\n', rank(ctrb(A_num, B_num)))
fprintf('  Cabeceo-avance:   rank = %d/4\n', rank(ctrb(A_ca, B_ca)))
fprintf('  Giro:             rank = %d/2\n', rank(ctrb(A_giro, B_giro)))

%% S11 — GUARDAR
fprintf('\nS11: Guardando variables de entorno\n')
save('segway_modelo_resultados.mat', ...
    'A_lin', 'B_lin', 'A_num', 'B_num', 'C_num', 'D_num', ...
    'A_ca', 'B_ca', 'A_giro', 'B_giro', ...
    'M_mat', 'C_mat', 'G_vec', 'Q_nc', 'f_nl', ...
    'X_sym', 'U_sym', 'polos', 'p')
fprintf('  Guardado exitoso.\n')