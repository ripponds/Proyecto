%% params.m — Parámetros físicos del vehículo autobalanceado
%  CORRER PRIMERO. Todos los demás scripts dependen de este workspace.
%
%  Estado:  X = [theta, dtheta, x, dx, alpha, dalpha]
%  Entrada: U = [V_R, V_L]  [V]
%
%  Convención de signos:
%    theta  > 0  →  inclinado hacia adelante
%    alpha  > 0  →  giro izquierda (RHR sobre eje vertical)
%    V_R, V_L    →  voltaje rueda derecha e izquierda
% =========================================================================

%% ── 1. PARÁMETROS FÍSICOS DEL VEHÍCULO ──────────────────────────────────
M   = 80;      % [kg]           Masa cuerpo + persona
m   = 2;       % [kg]           Masa por rueda
r   = 0.20;    % [m]            Radio de rueda
d   = 0.60;    % [m]            Separación entre ruedas (vía)
l   = 0.90;    % [m]            Distancia eje de ruedas → CM del cuerpo
g   = 9.81;    % [m/s²]         Gravedad estándar

Icy = 10;      % [kg·m²]        Inercia cuerpo — eje cabeceo (pitch, Y)
Icz = 12;      % [kg·m²]        Inercia cuerpo — eje giro    (yaw,   Z)
Icx = 12;      % [kg·m²]        Inercia cuerpo — eje alabeo  (roll,  X)
Iw  = 0.08;    % [kg·m²]        Inercia rueda  — eje de spin (axial)
Iwz = 0.04;    % [kg·m²]        Inercia rueda  — eje transversal

alm = 2.0;     % [N·m/V]        Ganancia de par del motor  (tau = alm*V)
bem = 1.5;     % [N·m·s/rad]    Amortiguamiento (back-EMF + fricción viscosa)

%% ── 2. GEOMETRÍA VISUAL (Simscape Multibody) ─────────────────────────────
body_W = 0.40;   % [m]  Ancho del cuerpo
body_D = 0.20;   % [m]  Profundidad del cuerpo
body_H = 1.60;   % [m]  Altura del cuerpo

%% ── 3. MATRICES DE MASA LINEALIZADAS (en equilibrio, theta=0) ────────────
%  Derivadas del Lagrangiano — no cambian si no cambian los parámetros físicos
%
%  Sistema desacoplado:
%    Subsistema avance : det0 = M11*M22 - M12^2
%    Subsistema giro   : M33 (independiente)
M11  = Icy + M*l^2;
M12  = M*l;                                          % cos(0) = 1
M22  = M + 2*m + 2*Iw/r^2;
M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
det0 = M11*M22 - M12^2;

%% ── 4. CONFIRMACIÓN ──────────────────────────────────────────────────────
fprintf('=========================================================\n');
fprintf(' params.m cargado\n');
fprintf(' Vehículo: M=%.0fkg  r=%.2fm  l=%.2fm  d=%.2fm\n', M, r, l, d);
fprintf(' Motor:    alm=%.1f N·m/V   bem=%.1f N·m·s/rad\n', alm, bem);
fprintf(' M11=%.2f  M22=%.2f  M33=%.2f  det0=%.2f\n', M11, M22, M33, det0);
fprintf('=========================================================\n');
