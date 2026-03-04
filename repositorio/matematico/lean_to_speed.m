%% lean_to_speed_v2_opt.m
%  Lean-to-speed optimizado con:
%  ─ Anti-windup vectorizado (sin if/else)
%  ─ Filtro de primer orden para theta_rider (biomecanica suave)
%  ─ Limite explicito en v_ref
% =========================================================================
clear; clc; close all;

%% 0. CONFIGURACION VISUAL (Modo Oscuro)
set(0,'DefaultFigureColor',    [0.12 0.12 0.12]);
set(0,'DefaultAxesColor',      [0.18 0.18 0.18]);
set(0,'DefaultAxesXColor',      'w');
set(0,'DefaultAxesYColor',      'w');
set(0,'DefaultAxesGridColor',  [0.3 0.3 0.3]);
set(0,'DefaultTextColor',       'w');

%% 1. PARAMETROS
M=80; r=0.20; d=0.60; l=0.90; g=9.81;
m=2; Icy=10; Icz=12; Iw=0.08; Iwz=0.04; alm=2.0;
V_sat_a=24; V_sat_d=24; V_sat_f=24;
t_sim = 15;

% ── Escenario del rider ──────────────────────────────────────────────────
Kv        = 5.0;            % ganancia lean-to-speed [m/s / rad]
th_lean   = deg2rad(5);     % inclinacion que aplica el rider [rad]
t_lean    = 3.0;            % instante en que empieza a inclinarse [s]
t_back    = 10.0;           % instante en que se incorpora [s]

% ── Dinámica del rider (Filtro 1er orden) ────────────────────────────────
% Reemplazamos el rate_limiter duro por un filtro con constante de tiempo tau.
% Una persona tarda ~0.4s en llegar al 63% de su inclinacion.
tau_rider = 0.3; % [s]

% ── Limites de seguridad ─────────────────────────────────────────────────
v_max     = 2.0;            % velocidad maxima permitida [m/s]
int_max   = 4.0;            % limite anti-windup del integrador

%% 2. MATRICES DE MASA
M11 = Icy + M*l^2;
M12 = M*l;
M22 = M + 2*m + 2*Iw/r^2;
M33 = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
det0 = M11*M22 - M12^2;

%% 3. LQR K1 AUMENTADO — [theta_err, dtheta, dx, integral_ev]
b21 = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41 = ( M12*(2*alm) + M11*(2*alm/r)) / det0;

A1r = [0,               1, 0;
       M22*M*g*l/det0,  0, 0;
      -M12*M*g*l/det0,  0, 0];
B1r = [0; b21; b41];

A1a = [A1r, zeros(3,1); 0 0 1 0];
B1a = [B1r; 0];

Q1a = diag([2000, 100, 50, 50/(1.5*5)]);
R1  = 1;
[K1a, ~, ~] = lqr(A1a, B1a, Q1a, R1);

%% 4. LQR K2 GIRO
A2=[0 1; 0 0]; B2=[0; d*alm/(r*M33)];
Q2=diag([800,50]); R2=1;
[K2,~,~] = lqr(A2,B2,Q2,R2);

%% 5. ODE — 7 estados fisicos + 1 estado filtro rider
X0 = zeros(8,1);

% Uso de odeset optimizado para sistemas suaves
options = odeset('RelTol',1e-5, 'AbsTol', 1e-6);

ode_fn = @(t,X) ode_lts_v3(t, X, K1a, K2, ...
    V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    Kv, th_lean, t_lean, t_back, tau_rider, v_max, int_max);

[t, X] = ode45(ode_fn, [0 t_sim], X0, options);

%% 6. RECONSTRUCCION VECTORIZADA
theta_rider_t = X(:,8);                                    % rider filtrado
v_ref_t       = min(Kv * theta_rider_t, v_max);            % v_ref limitada

Xe1_mat = [X(:,1)-theta_rider_t, X(:,2), X(:,4), max(min(X(:,7), int_max), -int_max)]';
Va_raw  = -K1a * Xe1_mat;
Va      = max(min(Va_raw', V_sat_a), -V_sat_a);

%% 7. GRAFICAS
figure('Name','Lean-to-Speed v2.1','Position',[50 50 950 950]);

% -- theta robot vs rider --
subplot(4,1,1); hold on; grid on;
plot(t, rad2deg(X(:,1)),       'r', 'LineWidth',2, 'DisplayName','\theta robot');
plot(t, rad2deg(theta_rider_t),'--w','LineWidth',1.5,'DisplayName','\theta rider (filtro 1er orden)');
xline(t_lean, '--g','Inclina', 'LabelHorizontalAlignment','left','Color',[0.3 0.9 0.3]);
xline(t_back, '--m','Incorpora','LabelHorizontalAlignment','left','Color',[0.9 0.3 0.9]);
xlabel('t [s]'); ylabel('\theta [deg]');
legend('Location','best','TextColor','w','Color','none');
xlim([0 t_sim]);

% -- velocidad --
subplot(4,1,2); hold on; grid on;
plot(t, X(:,4),   'b',  'LineWidth',2,  'DisplayName','dx simulado');
plot(t, v_ref_t,  '--w','LineWidth',1.5,'DisplayName',sprintf('v_{ref} (max %.1f)',v_max));
yline(v_max, '--y', 'Limite v_{max}');
xlabel('t [s]'); ylabel('dx [m/s]');
legend('Location','best','TextColor','w','Color','none');
xlim([0 t_sim]);

% -- Va --
subplot(4,1,3); hold on; grid on;
plot(t, Va, 'g', 'LineWidth',2);
yline( V_sat_a,'--w'); yline(-V_sat_a,'--w');
xlabel('t [s]'); ylabel('Va [V]');
xlim([0 t_sim]);

% -- integrador --
subplot(4,1,4); hold on; grid on;
plot(t, X(:,7), 'm', 'LineWidth',2,'DisplayName','\int e_v dt');
yline( int_max,'--w'); yline(-int_max,'--w');
xlabel('t [s]'); ylabel('Integral');
legend('Location','best','TextColor','w','Color','none');
xlim([0 t_sim]);

%% FUNCIONES LOCALES OPTIMIZADAS
function dX = ode_lts_v3(t, X, K1a, K2, ...
    V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    Kv, th_lean, t_lean, t_back, tau_rider, v_max, int_max)

    % ── Dinamica del rider (Filtro pasa-bajos) ─────────────────────────
    th_target = th_lean * (t >= t_lean && t < t_back);
    dX8 = (th_target - X(8)) / tau_rider; % Suave y continuo para el solver

    th_r = X(8);                          
    vr   = min(Kv * th_r, v_max);         

    % ── Anti-windup ────────────────────────────────────────────────────
    int_ev = max(min(X(7), int_max), -int_max);

    % ── Controladores ──────────────────────────────────────────────────
    Xe1 = [X(1)-th_r; X(2); X(4); int_ev];
    Va  = max(min(-K1a*Xe1, V_sat_a), -V_sat_a);
    
    Xe2 = [X(5); X(6)];
    Vd  = max(min(-K2*Xe2, V_sat_d), -V_sat_d);
    
    VR = max(min(Va+Vd, V_sat_f), -V_sat_f);
    VL = max(min(Va-Vd, V_sat_f), -V_sat_f);
    tau_R = alm*VR; tau_L = alm*VL;

    % ── Dinamica Planta ────────────────────────────────────────────────
    theta=X(1); dtheta=X(2);
    M11 = Icy + M*l^2;
    M12 = M*l*cos(theta);
    M22 = M + 2*m + 2*Iw/r^2;
    M33 = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    detM = M11*M22 - M12^2;

    F1 = M*g*l*sin(theta) - (tau_R+tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R+tau_L)/r;
    F3 = d*(tau_R-tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / detM;
    ddx     = (-M12*F1 + M11*F2) / detM;
    ddalpha = F3 / M33;

    % ── Anti-windup Vectorizado (Sin if/else) ──────────────────────────
    ev = X(4) - vr;
    % Anula la derivada si estamos en el límite y empujando en la misma dirección
    windup_flag = (X(7) >= int_max && ev > 0) || (X(7) <= -int_max && ev < 0);
    d_int = ev * (~windup_flag); 

    dX = [dtheta; ddtheta; X(4); ddx; X(6); ddalpha; d_int; dX8];
end