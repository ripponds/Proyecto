%% ========================================================================
%  segway_pid.m — Modelo No Lineal → Linealización → PID → Simulación
%  Extraído del modelado Lagrangiano (kane_final.m)
%  MATLAB 2025b
% =========================================================================
%
%  Estado:  X = [theta; dtheta; x; dx; alpha; dalpha]
%
%    theta   [rad]   — ángulo de cabeceo:  0=erguido, >0=inclinado adelante
%    dtheta  [rad/s] — velocidad angular de cabeceo
%    x       [m]     — posición de avance (eje a1, a lo largo del movimiento)
%    dx      [m/s]   — velocidad de avance
%    alpha   [rad]   — giro: >0=giro izquierda, producido por diferencia de motores (RHR sobre a3)
%    dalpha  [rad/s] — velocidad angular de giro
%
%  Entrada: U = [V_R; V_L]  [V] — voltaje rueda derecha e izquierda
%
%  Nota de visualización: los ángulos se muestran en GRADOS [°] en las
%  gráficas, pero el sistema opera internamente en RADIANES [rad].
% =========================================================================

clear; clc; close all;

%% ── 1. PARÁMETROS FÍSICOS (placeholders — modificar aquí) ───────────────
M       = 80;    % [kg]           Masa cuerpo + persona
m       = 2;     % [kg]           Masa por rueda
r       = 0.20;  % [m]            Radio de rueda
d       = 0.60;  % [m]            Separación entre ruedas (vía)
l       = 0.90;  % [m]            Distancia eje de ruedas → CM del cuerpo
g       = 9.81;  % [m/s²]         Gravedad estándar
Icy     = 10;    % [kg·m²]        Inercia de cabeceo del cuerpo (eje lateral c2)
Icz     = 12;    % [kg·m²]        Inercia de giro del cuerpo (eje vertical c3)
Icx     = 12;    % [kg·m²]        Inercia de alabeo del cuerpo  (eje avance c1)
Iw      = 0.08;  % [kg·m²]        Inercia axial por rueda (spin, eje de cabeceo)
Iwz     = 0.04;  % [kg·m²]        Inercia transversal por rueda (eje de giro)
alm     = 2.0;   % [N·m/V]        Ganancia de par del motor
bem     = 1.5;   % [N·m·s/rad]    Coeficiente de amortiguación del motor

%% ── 2. MODELO NO LINEAL ─────────────────────────────────────────────────
% Ecuaciones de movimiento extraídas del Lagrangiano (kane_final.m, S5).
%
% Sub-sistema cabeceo–avance (acoplado, 2×2):
%
%   ┌ M11    M12(θ) ┐ ┌ ddθ ┐   ┌ F1 ┐
%   └ M12(θ) M22   ┘ └ ddx ┘ = └ F2 ┘
%
%   M11      = Icy + M·l²                        [kg·m²]
%   M12(θ)   = M·l·cos(θ)                        [kg·m]
%   M22      = M + 2m + 2·Iw/r²                  [kg]
%   det(θ)   = M11·M22 − M12²
%
% Sub-sistema giro (desacoplado por simetría lateral):
%   Producido por la diferencia de par entre rueda derecha e izquierda.
%
%   M33(θ)·ddα = F3
%   M33(θ) = Icx·sin²(θ) + Icz·cos²(θ) + 2m·(d/2)² + 2·Iw·(d/(2r))² + 2·Iwz
%
% Pares motor:
%   τ_R = alm·V_R − bem·( dx/r + d·dα/(2r) − dθ )   [N·m]
%   τ_L = alm·V_L − bem·( dx/r − d·dα/(2r) − dθ )   [N·m]
%
% Fuerzas generalizadas:
%   F1 = M·g·l·sin(θ) − (τ_R + τ_L)                 [N·m]  ← gravedad + reacción motor
%   F2 = M·l·dθ²·sin(θ) + (τ_R + τ_L)/r             [N]    ← coriolis + tracción
%   F3 = d·(τ_R − τ_L)/(2r)                           [N·m]  ← torque diferencial (giro)
%
% → Ver función local segway_ode al final del script.

%% ── 3. LINEALIZACIÓN EN θ=0, X=0, U=0 ───────────────────────────────────
% Jacobiana analítica de f_nl evaluada en el equilibrio vertical:
%   X_eq = [0; 0; 0; 0; 0; 0],  U_eq = [0; 0]

% Elementos de la matriz de masa evaluados en θ=0 (cos=1, sin=0)
M11 = Icy + M*l^2;                              % [kg·m²]
M12 = M*l;                                       % [kg·m]   M12 = M·l·cos(0) = M·l
M22 = M + 2*m + 2*Iw/r^2;                       % [kg]
M33 = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz; % [kg·m²]
det0 = M11*M22 - M12^2;                         % [kg²·m²]

% Suma de torques: τ_R+τ_L = alm·(VR+VL) − 2·bem·(dx/r − dθ)
% Diferencia:      τ_R−τ_L = alm·(VR−VL) − bem·d·dα/r
%
% Derivadas parciales de F1 en el equilibrio:
dF1_dth  =  M*g*l;    % ∂F1/∂θ     [N·m/rad]   ← término gravitacional linealizado
dF1_doth = -2*bem;    % ∂F1/∂dθ    [N·m/(rad/s)]
dF1_dox  =  2*bem/r;  % ∂F1/∂dx    [N·m/(m/s)]
dF1_VR   = -alm;      % ∂F1/∂V_R   [N·m/V]
dF1_VL   = -alm;      % ∂F1/∂V_L   [N·m/V]

% Derivadas parciales de F2 en el equilibrio:
% (dθ²·sin(θ)|₀ = 0, el término de Coriolis desaparece)
dF2_doth =  2*bem/r;   % ∂F2/∂dθ   [N/(rad/s)]
dF2_dox  = -2*bem/r^2; % ∂F2/∂dx   [N/(m/s)]
dF2_VR   =  alm/r;     % ∂F2/∂V_R  [N/V]
dF2_VL   =  alm/r;     % ∂F2/∂V_L  [N/V]

% Derivadas parciales de F3 en el equilibrio:
dF3_doal = -bem*d^2/(2*r^2); % ∂F3/∂dα    [N·m/(rad/s)]
dF3_VR   =  alm*d/(2*r);     % ∂F3/∂V_R   [N·m/V]
dF3_VL   = -alm*d/(2*r);     % ∂F3/∂V_L   [N·m/V]

% ── Matriz A (6×6) ────────────────────────────────────────────────────────
A = zeros(6);

% Cinemática (identidades triviales de la realización espacio de estados)
A(1,2) = 1;   % dθ/dt  = dθ  [rad/s → rad/s]
A(3,4) = 1;   % dx/dt  = dx  [m/s   → m/s  ]
A(5,6) = 1;   % dα/dt  = dα  [rad/s → rad/s]

% Dinámica de cabeceo  ddθ = (M22·F1 − M12·F2) / det
A(2,1) = (M22*dF1_dth)                          / det0;  % [rad/s²/rad]
A(2,2) = (M22*dF1_doth - M12*dF2_doth)          / det0;  % [rad/s²/(rad/s)]
A(2,4) = (M22*dF1_dox  - M12*dF2_dox)           / det0;  % [rad/s²/(m/s)]

% Dinámica de avance   ddx = (−M12·F1 + M11·F2) / det
A(4,1) = (-M12*dF1_dth)                          / det0;  % [m/s²/rad]
A(4,2) = (-M12*dF1_doth + M11*dF2_doth)          / det0;  % [m/s²/(rad/s)]
A(4,4) = (-M12*dF1_dox  + M11*dF2_dox)           / det0;  % [m/s²/(m/s)]

% Dinámica de giro  ddα = F3 / M33  (activada por diferencia de voltajes)
A(6,6) = dF3_doal / M33;                                   % [rad/s²/(rad/s)]

% ── Matriz B (6×2) ────────────────────────────────────────────────────────
B = zeros(6,2);

% Efecto de V_R y V_L sobre cabeceo (simétrico: mismo efecto en ambas ruedas)
B(2,1) = (M22*dF1_VR - M12*dF2_VR) / det0;  % ∂ddθ/∂V_R  [rad/s²/V]
B(2,2) = (M22*dF1_VL - M12*dF2_VL) / det0;  % ∂ddθ/∂V_L
B(4,1) = (-M12*dF1_VR + M11*dF2_VR) / det0; % ∂ddx/∂V_R  [m/s²/V]
B(4,2) = (-M12*dF1_VL + M11*dF2_VL) / det0; % ∂ddx/∂V_L

% Efecto de V_R y V_L sobre giro (antisimétrico: canal diferencial de voltaje)
B(6,1) = dF3_VR / M33;   % ∂ddα/∂V_R   [rad/s²/V]
B(6,2) = dF3_VL / M33;   % ∂ddα/∂V_L

C_out = eye(6);       % Salida = estado completo (todos los estados medidos)
D_out = zeros(6,2);

fprintf('=== SISTEMA LINEALIZADO EN θ=0 ===\n');
disp('A ='); disp(A)
disp('B ='); disp(B)

% Polos en lazo abierto
polos_OL = eig(A);
fprintf('Polos en lazo abierto:\n'); disp(polos_OL)
if any(real(polos_OL) > 1e-6)
    fprintf('→ Sistema INESTABLE (péndulo invertido) — esperado.\n\n');
end

%% ── 4. CONTROLABILIDAD Y OBSERVABILIDAD (subsistemas por separado) ──────

fprintf('=== CONTROLABILIDAD Y OBSERVABILIDAD ===\n');

% -- Sub-sistema cabeceo–avance: [θ, dθ, x, dx] (índices 1:4) -------------
A_ca = A(1:4, 1:4);
B_ca = B(1:4, :);
C_ca = C_out(1:4, 1:4);

rk_C_ca = rank(ctrb(A_ca, B_ca));
rk_O_ca = rank(obsv(A_ca, C_ca));

fprintf('\nCabeceo–Avance  [θ(rad), dθ(rad/s), x(m), dx(m/s)]:\n');
fprintf('  Controlabilidad: rango = %d/4', rk_C_ca);
if rk_C_ca == 4, fprintf('  ← CONTROLABLE\n'); else, fprintf('  ← NO CONTROLABLE\n'); end
fprintf('  Observabilidad:  rango = %d/4', rk_O_ca);
if rk_O_ca == 4, fprintf('  ← OBSERVABLE\n');   else, fprintf('  ← NO OBSERVABLE\n');   end

% -- Sub-sistema giro: [α, dα] (índices 5:6) — canal diferencial de motores -
A_g  = A(5:6, 5:6);
B_g  = B(5:6, :);
C_g  = C_out(5:6, 5:6);

rk_C_g = rank(ctrb(A_g, B_g));
rk_O_g = rank(obsv(A_g, C_g));

fprintf('\nGiro  [α(rad), dα(rad/s)]  — canal diferencial de motores:\n');
fprintf('  Controlabilidad: rango = %d/2', rk_C_g);
if rk_C_g == 2, fprintf('  ← CONTROLABLE\n'); else, fprintf('  ← NO CONTROLABLE\n'); end
fprintf('  Observabilidad:  rango = %d/2', rk_O_g);
if rk_O_g == 2, fprintf('  ← OBSERVABLE\n');   else, fprintf('  ← NO OBSERVABLE\n');   end

%% ── 5. DISEÑO PID — SOLO CABECEO ─────────────────────────────────────────
% El PID actúa únicamente sobre el ángulo de cabeceo θ [rad].
% Señal de error:  e(t) = 0 − θ(t)  (referencia = posición erguida)
%
% Ley de control PID:
%   u_pid(t) = Kp·e + Ki·∫e dt + Kd·de/dt
%            = −Kp·θ − Ki·∫θ dt − Kd·dθ
%
% El término derivativo usa dθ directamente del estado X(2) [rad/s],
% evitando diferenciación numérica ruidosa.
%
% El voltaje resultante se aplica igual a ambas ruedas (canal simétrico):
%   V_R = V_L = u_pid   [V]
% El giro no se regula en este controlador (canal diferencial = 0).

% ── Punto de partida para la optimización (ajustar si es necesario) ───────
Kp0 = 50;   % [V/rad]       Estimación inicial proporcional
Ki0 = 1;    % [V/(rad·s)]   Estimación inicial integral
Kd0 = 8;    % [V·s/rad]     Estimación inicial derivativa

% ── Condiciones iniciales (usadas tanto en optimización como en simulación final) ──
theta0_deg = 15;   % [°]
alpha0_deg = 10;   % [°]
theta0 = theta0_deg * (pi/180);   % [rad]
alpha0 = alpha0_deg * (pi/180);   % [rad]

% Estado aumentado inicial [θ, dθ, x, dx, α, dα, ∫θ]
X0 = [theta0;  ...   % theta    [rad]   — cabeceo
      0;        ...   % dtheta   [rad/s]
      0;        ...   % x        [m]
      0;        ...   % dx       [m/s]
      alpha0;   ...   % alpha    [rad]   — giro (diferencial de motores)
      0;        ...   % dalpha   [rad/s]
      0];             % ∫θ dt    [rad·s] — estado integral del PID

t_sim = 30;   % [s] — horizonte de simulación (máximo para optimización y respuesta final)

% ── Función de coste: ISE de cabeceo + penalización por inestabilidad ─────
% Minimiza ∫θ²dt para encontrar las ganancias que mejor regulan el cabeceo.
% Si el sistema diverge (|θ| > 90°) se penaliza fuertemente.
costo_pid = @(gains) pid_costo(gains, A, B, X0, t_sim);

fprintf('\n=== OPTIMIZACIÓN PID (fminsearch) ===\n');
fprintf('Buscando Kp, Ki, Kd que minimicen ISE de cabeceo en %d s...\n', t_sim);

opts_opt = optimset('Display','iter', 'TolX',1e-3, 'TolFun',1e-3, 'MaxIter',200);
gains_opt = fminsearch(costo_pid, [Kp0; Ki0; Kd0], opts_opt);

Kp = gains_opt(1);   % [V/rad]       Ganancia proporcional al cabeceo — optimizada
Ki = gains_opt(2);   % [V/(rad·s)]   Ganancia integral                — optimizada
Kd = gains_opt(3);   % [V·s/rad]     Ganancia derivativa              — optimizada

fprintf('\n=== CONTROLADOR PID (solo cabeceo) — GANANCIAS ÓPTIMAS ===\n');
fprintf('  Kp = %.4f  [V/rad]      → %.4f [V/°]\n', Kp, Kp*pi/180);
fprintf('  Ki = %.4f  [V/(rad·s)]\n', Ki);
fprintf('  Kd = %.4f  [V·s/rad]\n',  Kd);

%% ── 6. SIMULACIÓN CON ode45 ──────────────────────────────────────────────
% El PID se implementa como estado aumentado: se añade la integral de θ
% como séptima variable de estado para integrar dentro de ode45.
%
% Estado aumentado: Xa = [θ; dθ; x; dx; α; dα; ∫θ dt]
%
% Ley de control en cada paso de integración:
%   e     = −θ                          (error de cabeceo respecto a 0)
%   u_pid = −Kp·θ − Kd·dθ − Ki·∫θ dt  [V]
%   V_R   = V_L = u_pid                 [V]  (sin saturación)

% ODE con PID sin saturación — ganancias optimizadas
% El sistema lineal recibe U = [V_R; V_L], ambos iguales al voltaje PID
ode_pid = @(t, Xa) pid_ode(Xa, A, B, Kp, Ki, Kd);
[t_cl, Xa_cl] = ode45(ode_pid, [0 t_sim], X0);
% Xa_cl: [N_puntos × 7]  — los 6 estados + la integral de θ

x_cl = Xa_cl(:, 1:6);   % [N_puntos × 6] — estados del sistema

% Voltaje PID reconstruido en post-proceso para graficar
int_theta = Xa_cl(:, 7);                                      % [rad·s] — integral de θ
u_pid_hist = -Kp*x_cl(:,1) - Kd*x_cl(:,2) - Ki*int_theta;   % [V] — sin saturación
% Ambas ruedas reciben el mismo voltaje (canal simétrico, sin giro)
U_hist = [u_pid_hist, u_pid_hist];   % [N_puntos × 2]: columna 1 = V_R, columna 2 = V_L

%% ── 7. GRÁFICAS ──────────────────────────────────────────────────────────
figure('Name','PID Segway — Respuesta a Condiciones Iniciales', ...
       'Position',[80 80 1200 700]);

% Cabeceo θ — ángulo de inclinación [mostrado en grados]
subplot(2,3,1);
plot(t_cl, rad2deg(x_cl(:,1)), 'b', 'LineWidth', 1.5);
yline(0,'--k','Equilibrio'); grid on;
xlabel('t  [s]'); ylabel('Cabeceo  [°]');
title(sprintf('Cabeceo  (\\theta_0 = %d°)', theta0_deg));

% Velocidad angular de cabeceo dθ [°/s]
subplot(2,3,2);
plot(t_cl, rad2deg(x_cl(:,2)), 'b', 'LineWidth', 1.5);
yline(0,'--k'); grid on;
xlabel('t  [s]'); ylabel('Vel. cabeceo  [°/s]');
title('Velocidad de cabeceo');

% Posición de avance x [m]
subplot(2,3,3);
plot(t_cl, x_cl(:,3), 'g', 'LineWidth', 1.5);
yline(0,'--k','Origen'); grid on;
xlabel('t  [s]'); ylabel('Avance  [m]');
title('Avance');

% Giro α — producido por diferencia de motores [mostrado en grados]
% El PID no regula este canal: el giro evoluciona libremente
subplot(2,3,4);
plot(t_cl, rad2deg(x_cl(:,5)), 'r', 'LineWidth', 1.5);
yline(0,'--k','Equilibrio'); grid on;
xlabel('t  [s]'); ylabel('Giro  [°]');
title(sprintf('Giro libre  (\\alpha_0 = %d°)', alpha0_deg));

% Velocidad de avance dx [m/s]
subplot(2,3,5);
plot(t_cl, x_cl(:,4), 'g', 'LineWidth', 1.5);
yline(0,'--k'); grid on;
xlabel('t  [s]'); ylabel('Velocidad  [m/s]');
title('Velocidad de avance');

% Voltajes de control [V] — V_R = V_L (canal simétrico, PID de cabeceo)
subplot(2,3,6);
plot(t_cl, U_hist(:,1), 'Color','#0077BB', 'LineWidth', 1.5); hold on;
plot(t_cl, U_hist(:,2), '--', 'Color','#EE7733', 'LineWidth', 1.5);
yline(0,'--k'); grid on;
xlabel('t  [s]'); ylabel('V  [V]');
legend('V_R (dcha)','V_L (izq)','Location','best');
title('Voltajes motor  (V_R = V_L, sin giro, sin saturación)');

sgtitle(sprintf('PID Segway — Solo Cabeceo  |  \\theta_0=%d°, \\alpha_0=%d°  |  Kp=%.0f  Ki=%.0f  Kd=%.0f', ...
                theta0_deg, alpha0_deg, Kp, Ki, Kd), 'FontSize', 12);

%% ── FUNCIÓN LOCAL: Coste para optimización de ganancias PID ──────────────
% Simula el sistema con las ganancias dadas y devuelve ISE de cabeceo.
% Penaliza fuertemente si el sistema diverge (|θ| > 90° = 1.57 rad).

function J = pid_costo(gains, A, B, X0, t_sim)
    Kp = gains(1);   % [V/rad]
    Ki = gains(2);   % [V/(rad·s)]
    Kd = gains(3);   % [V·s/rad]

    % Integración con tolerancias relajadas para rapidez en la optimización
    opts = odeset('RelTol',1e-3, 'AbsTol',1e-4, 'Events',@diverge_event);
    [t, Xa] = ode45(@(t,Xa) pid_ode(Xa,A,B,Kp,Ki,Kd), [0 t_sim], X0, opts);

    theta_hist = Xa(:, 1);   % [rad] — ángulo de cabeceo a lo largo del tiempo

    % Penalización si el sistema divergió antes de terminar
    if t(end) < t_sim * 0.95
        J = 1e6;   % coste muy alto si el sistema cayó
        return
    end

    % ISE (Integral of Squared Error) del cabeceo: ∫θ² dt  [rad²·s]
    J = trapz(t, theta_hist.^2);
end

% Evento: detener la integración si |θ| supera 90° (sistema caído)
function [val, isterm, dir] = diverge_event(~, Xa)
    val    = abs(Xa(1)) - (pi/2);   % [rad] — se activa cuando |θ| = 90°
    isterm = 1;                      % detener la integración
    dir    = 1;                      % solo cruce positivo
end

%% ── FUNCIÓN LOCAL: ODE del sistema lineal con controlador PID ────────────
% Estado aumentado Xa = [X(6×1); integral_theta(1×1)]
% El PID actúa solo sobre cabeceo: usa Xa(1)=θ, Xa(2)=dθ, Xa(7)=∫θ

function dXa = pid_ode(Xa, A, B, Kp, Ki, Kd)
    X      = Xa(1:6);   % estados del sistema  [rad, rad/s, m, m/s, rad, rad/s]
    iTheta = Xa(7);     % integral de θ        [rad·s]

    % Ley PID sobre cabeceo: e = -θ (referencia = 0)
    % u = -Kp·θ - Kd·dθ - Ki·∫θ dt   [V]  (sin saturación)
    u_pid = -Kp*X(1) - Kd*X(2) - Ki*iTheta;

    % Ambas ruedas reciben el mismo voltaje (canal simétrico = solo cabeceo)
    U = [u_pid; u_pid];   % [V_R; V_L]

    % Dinámica del sistema lineal: dX/dt = A·X + B·U
    dX = A*X + B*U;

    % Integral del error de cabeceo: d(∫θ)/dt = θ  [rad]
    dXa = [dX; X(1)];
end

%% ── FUNCIÓN LOCAL: ODE no lineal ─────────────────────────────────────────
% Definición exacta del sistema, sin aproximación en α.
% Útil para validación no lineal con ode45 (no usada arriba).
%
% Llamada: segway_ode(t, X, U, parámetros...)

function dX = segway_ode(~, X, U, M, m, r, d, l, g, Icy, Icz, Icx, Iw, Iwz, alm, bem) %#ok<DEFNU>
    theta  = X(1);  % [rad]
    dtheta = X(2);  % [rad/s]
    dx     = X(4);  % [m/s]
    dalpha = X(6);  % [rad/s]
    VR = U(1);  VL = U(2);  % [V]

    % Matriz de masa (depende de θ — no linealidad principal)
    M11 = Icy + M*l^2;                                          % [kg·m²]
    M12 = M*l*cos(theta);                                        % [kg·m]
    M22 = M + 2*m + 2*Iw/r^2;                                   % [kg]
    M33 = Icx*sin(theta)^2 + Icz*cos(theta)^2 + ...
          2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;               % [kg·m²]
    det_ca = M11*M22 - M12^2;                                    % [kg²·m²]

    % Pares motor [N·m]
    tau_R = alm*VR - bem*(dx/r + d*dalpha/(2*r) - dtheta);
    tau_L = alm*VL - bem*(dx/r - d*dalpha/(2*r) - dtheta);

    % Fuerzas generalizadas
    F1 = M*g*l*sin(theta) - (tau_R + tau_L);            % [N·m]
    F2 = M*l*dtheta^2*sin(theta) + (tau_R + tau_L)/r;   % [N]
    F3 = d*(tau_R - tau_L)/(2*r);                        % [N·m]

    % Aceleraciones generalizadas
    ddtheta = ( M22*F1 - M12*F2) / det_ca;   % [rad/s²]
    ddx     = (-M12*F1 + M11*F2) / det_ca;   % [m/s²]
    ddalpha = F3 / M33;                       % [rad/s²]

    dX = [dtheta; ddtheta; dx; ddx; dalpha; ddalpha];
end
