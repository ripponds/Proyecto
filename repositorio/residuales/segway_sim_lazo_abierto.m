%% ========================================================================
%  segway_sim_lazo_abierto.m  —  SCRIPT PRINCIPAL
%
%  1. Define parámetros del sistema
%  2. Simula lazo abierto no lineal (3 escenarios)
%  3. Carga workspace con A_num, B_num, p, params_vec
%  4. Llama automáticamente a segway_controlador.m
%
%  Estados: X = [theta; dtheta; x; dx; alpha; dalpha]
%  Entradas: U = [V_R; V_L]
% =========================================================================

%% ── LIMPIEZA ─────────────────────────────────────────────────────────────
clear; clc; close all force;
try; bdclose all; catch; end

fprintf('╔══════════════════════════════════════════════════╗\n')
fprintf('║  SEGWAY — SCRIPT PRINCIPAL                      ║\n')
fprintf('║  1/2: Lazo Abierto No Lineal                    ║\n')
fprintf('╚══════════════════════════════════════════════════╝\n\n')

%% ── P0: PARÁMETROS ───────────────────────────────────────────────────────
% ÚNICA sección donde se definen parámetros.
% Cambiar aquí — se propagan automáticamente al controlador.

p.M       = 80;      % kg    masa cuerpo + persona
p.m       = 2;       % kg    masa una rueda
p.r       = 0.20;    % m     radio rueda
p.d       = 0.60;    % m     separación entre ruedas
p.l       = 0.90;    % m     distancia eje → CM (positivo = péndulo invertido)
p.g       = 9.81;    % m/s²
p.Icy     = 10;      % kg·m²  inercia cabeceo  (eje a2)
p.Icz     = 12;      % kg·m²  inercia giro     (eje a3)
p.Icx     = 12;      % kg·m²  inercia alabeo   (eje a1)
p.Iw      = 0.08;    % kg·m²  inercia axial rueda
p.Iwz     = 0.04;    % kg·m²  inercia transversal rueda
p.alpha_m = 2.0;     % N·m/V  ganancia motor
p.beta_m  = 1.5;     % N·m·s/rad  amortiguamiento (back-EMF + fricción)

params_vec = [p.M; p.m; p.r; p.d; p.l; p.g; ...
              p.Icy; p.Icz; p.Icx; p.Iw; p.Iwz; p.alpha_m; p.beta_m];

fprintf('P0: Parámetros cargados.\n')
fprintf('    M=%.0f kg, l=%.2f m, r=%.2f m, d=%.2f m\n\n', ...
    p.M, p.l, p.r, p.d)

%% ── P1: MODELO LINEALIZADO (Jacobiano numérico) ──────────────────────────
fprintf('P1: Calculando modelo linealizado...\n')

eps_j = 1e-6;
X0_eq = zeros(6,1);
U0_eq = zeros(2,1);
f0    = segway_dyn_main(X0_eq, U0_eq, p);

A_num = zeros(6,6);
for k = 1:6
    Xp = X0_eq; Xp(k) = Xp(k) + eps_j;
    A_num(:,k) = (segway_dyn_main(Xp, U0_eq, p) - f0) / eps_j;
end

B_num = zeros(6,2);
for k = 1:2
    Up = U0_eq; Up(k) = Up(k) + eps_j;
    B_num(:,k) = (segway_dyn_main(X0_eq, Up, p) - f0) / eps_j;
end

polos_la = eig(A_num);
fprintf('    Polos lazo abierto:\n')
for k = 1:6
    fprintf('      p%d = %+.4f', k, real(polos_la(k)))
    if abs(imag(polos_la(k))) > 1e-6
        fprintf(' %+.4fj', imag(polos_la(k)))
    end
    if real(polos_la(k)) > 0
        fprintf('  <- inestable')
    end
    fprintf('\n')
end
fprintf('\n')

%% ── P2: SIMULACIÓN LAZO ABIERTO ─────────────────────────────────────────
fprintf('P2: Simulando lazo abierto (3 escenarios)...\n')

T_sim = 3.0;
opts  = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
               'Events', @evento_caida_main);

% X0 = [theta; dtheta; x; dx; alpha; dalpha]
esc(1).X0   = [0.05; 0; 0; 0; 0; 0];
esc(1).U    = [0; 0];
esc(1).desc = 'theta0=0.05 rad, U=0 (caida libre)';

esc(2).X0   = [0.02; 0; 0; 0; 0; 0];
esc(2).U    = [0.5; 0.5];
esc(2).desc = 'Escalon V=0.5V, theta0=0.02 rad';

esc(3).X0   = [0.001; 0; 0; 0; 0.10; 0.20];
esc(3).U    = [0; 0];
esc(3).desc = 'Perturbacion yaw alpha0=0.10 rad';

for k = 1:3
    U_k = esc(k).U;
    [t, X, te] = ode45(@(t,X) segway_dyn_main(X, U_k, p), ...
                        [0 T_sim], esc(k).X0, opts);
    esc(k).t  = t;
    esc(k).X  = X;
    esc(k).te = te;
    if ~isempty(te)
        fprintf('  Esc %d (%s): caida en t=%.3f s\n', k, esc(k).desc, te(1))
    else
        fprintf('  Esc %d (%s): completo sin caida\n', k, esc(k).desc)
    end
end

%% ── P3: REPORTE CONSOLA ──────────────────────────────────────────────────
fprintf('\n')
fprintf('══════════════════════════════════════════════════\n')
fprintf('  RESUMEN LAZO ABIERTO\n')
fprintf('══════════════════════════════════════════════════\n')
for k = 1:3
    fprintf('  Esc %d: theta_max=%+.4f rad (%.1f deg),  x_final=%+.4f m\n', ...
        k, ...
        max(abs(esc(k).X(:,1))), ...
        rad2deg(max(abs(esc(k).X(:,1)))), ...
        esc(k).X(end,3))
end
fprintf('══════════════════════════════════════════════════\n\n')

%% ── P4: CONFIRMAR WORKSPACE ──────────────────────────────────────────────
fprintf('P4: Workspace listo para controlador:\n')
fprintf('    p            — struct de parametros\n')
fprintf('    params_vec   — vector [13x1] para Simulink\n')
fprintf('    A_num        — matriz A linealizada [6x6]\n')
fprintf('    B_num        — matriz B linealizada [6x2]\n')
fprintf('    esc          — resultados lazo abierto\n')
fprintf('\n')

%% ── P5: LLAMAR AL CONTROLADOR ────────────────────────────────────────────
fprintf('╔══════════════════════════════════════════════════╗\n')
fprintf('║  Pasando a 2/2: Diseno de Controlador           ║\n')
fprintf('╚══════════════════════════════════════════════════╝\n\n')

run('segway_controlador.m')

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES LOCALES
%% ════════════════════════════════════════════════════════════════════════

function Xdot = segway_dyn_main(X, U, p)
Xdot   = zeros(6,1);
theta  = X(1);  dtheta = X(2);
dx     = X(4);  dalpha = X(6);
V_R    = U(1);  V_L    = U(2);

omR   = dx/p.r + p.d/(2*p.r)*dalpha;
omL   = dx/p.r - p.d/(2*p.r)*dalpha;
tau_R = p.alpha_m*V_R - p.beta_m*(omR - dtheta);
tau_L = p.alpha_m*V_L - p.beta_m*(omL - dtheta);

M11 = p.M*p.l^2 + p.Icy;
M12 = p.M*p.l*cos(theta);
M22 = (p.M + 2*p.m) + 2*p.Iw/(p.r^2);
M33 = (p.M + 2*p.m)*(p.d^2)/4 + 2*p.Iwz + p.Icz;
h1  = p.M*p.g*p.l*sin(theta) - (tau_R + tau_L);
h2  = p.M*p.l*sin(theta)*dtheta^2 + (tau_R + tau_L)/p.r;
h3  = p.d*(tau_R - tau_L)/(2*p.r);

det_M   = M11*M22 - M12*M12;
Xdot(1) = dtheta;
Xdot(2) = ( M22*h1 - M12*h2) / det_M;
Xdot(3) = dx;
Xdot(4) = (-M12*h1 + M11*h2) / det_M;
Xdot(5) = dalpha;
Xdot(6) = h3 / M33;
end

function [val, isterm, dir] = evento_caida_main(~, X)
val    = abs(X(1)) - deg2rad(60);
isterm = 1;
dir    = 1;
end
