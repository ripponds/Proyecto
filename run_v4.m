%% run_v4.m
%  SIMULACION — Segway Testbench v4
%  ─ Carga Segway_Testbench_v4.slx existente (NO lo reconstruye)
%  ─ Actualiza parametros y ganancias K1/K2 via set_param
%  ─ Corre ODE no lineal + Simscape y grafica comparacion
%  ─ PREREQUISITO: correr build_v4.m al menos una vez primero
% =========================================================================
clc; close all;

%% =========================================================================
%  1. PARAMETROS — MODIFICA AQUI TUS VALORES REALES
% =========================================================================
M      = 80;      % masa cuerpo [kg]
r      = 0.20;    % radio rueda [m]
d      = 0.60;    % separacion entre ruedas [m]
l      = 0.90;    % dist. eje ruedas -> CM cuerpo [m]
g      = 9.81;
m      = 2;       % masa cada rueda [kg]
Icy    = 10;      % inercia cuerpo pitch [kg·m^2]
Icz    = 12;      % inercia cuerpo yaw   [kg·m^2]
Icx    = 12;      % inercia cuerpo roll  [kg·m^2]
Iw     = 0.08;    % inercia rueda spin   [kg·m^2]
Iwz    = 0.04;    % inercia rueda transversal [kg·m^2]
alm    = 2.0;     % ganancia par motor [N·m/V]

% -- Condiciones iniciales --
theta0_deg = 5;      % perturbacion balanceo [deg]
alpha0_deg = 10;     % perturbacion giro     [deg]  <- valida K2
theta0     = theta0_deg * pi/180;
alpha0     = alpha0_deg * pi/180;

% -- Simulacion --
t_sim   = 15;    % duracion [s]
V_sat_a = 24;    % saturacion Va (avance)  [V]
V_sat_d = 24;    % saturacion Vd (giro)    [V]
V_sat_f = 24;    % saturacion final VR,VL  [V]

% -- RUTINA AVANCE: theta_ref --
theta_ref_deg = 0;
t_avance      = 999;
t_stop        = 1000;

% -- RUTINA GIRO: alpha_ref --
alpha_ref_deg = 0;
t_avance_a    = 999;
t_stop_a      = 1000;

fprintf('=========================================================\n');
fprintf(' RUN Segway v4 | 6 Estados | K1+K2 desacoplados\n');
fprintf(' theta0=%.1f  alpha0=%.1f  t_sim=%gs\n', theta0_deg, alpha0_deg, t_sim);
fprintf('=========================================================\n');

% Advertencia: alpha0 pequeno -> Vd imperceptible en animacion 3D
if abs(alpha0_deg) < 5
    fprintf('[ADVERTENCIA] alpha0=%.1f deg es pequeno — Vd inicial sera %.2f V.\n', ...
        alpha0_deg, abs(alpha0_deg)*pi/180 * 28.28);
    fprintf('              El giro puede ser imperceptible visualmente.\n');
    fprintf('              Usa alpha0_deg >= 5 para validar K2 visualmente.\n');
end

%% =========================================================================
%  2. LINEALIZACION Y LQR
% =========================================================================
M11  = Icy + M*l^2;
M12  = M*l;
M22  = M + 2*m + 2*Iw/r^2;
M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
det0 = M11*M22 - M12^2;

% ── K1: subsistema avance ─────────────────────────────────────────────
A1 = zeros(4);
A1(1,2) = 1;
A1(2,1) = M22*M*g*l / det0;
A1(3,4) = 1;
A1(4,1) = -M12*M*g*l / det0;

b21_a = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41_a = ( M12*(2*alm) + M11*(2*alm/r)) / det0;
B1    = [0; b21_a; 0; b41_a];

rango1 = rank(ctrb(A1, B1));
fprintf('[K1] Controlabilidad: %d/4', rango1);
if rango1==4, fprintf(' OK\n'); else, fprintf(' REVISAR\n'); end

Q1 = diag([2000, 100, 50, 50]);
R1 = 1;
[K1, ~, eigs1] = lqr(A1, B1, Q1, R1);
fprintf('[K1] K1 = [%.3f  %.3f  %.3f  %.3f]\n', K1);
fprintf('[K1] Polos: '); fprintf('%.3f  ', real(eigs1)); fprintf('\n');

% ── K2: subsistema giro ───────────────────────────────────────────────
A2 = [0 1; 0 0];
B2 = [0; d*alm/(r*M33)];

rango2 = rank(ctrb(A2, B2));
fprintf('[K2] Controlabilidad: %d/2', rango2);
if rango2==2, fprintf(' OK\n'); else, fprintf(' REVISAR\n'); end

Q2 = diag([800, 50]);
R2 = 1;
[K2, ~, eigs2] = lqr(A2, B2, Q2, R2);
fprintf('[K2] K2 = [%.3f  %.3f]\n', K2);
fprintf('[K2] Polos: '); fprintf('%.3f  ', real(eigs2)); fprintf('\n');

%% =========================================================================
%  3. ODE NO LINEAL 6 ESTADOS + K1/K2
% =========================================================================
theta_ref_rad = theta_ref_deg * pi/180;
alpha_ref_rad = alpha_ref_deg * pi/180;
X0_ode = [theta0; 0; 0; 0; alpha0; 0];

ode_fn = @(t,X) ode_nl_6(t, X, K1, K2, ...
    V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    theta_ref_rad, alpha_ref_rad, ...
    t_avance, t_stop, t_avance_a, t_stop_a);

[t_ode, X_ode] = ode45(ode_fn, [0 t_sim], X0_ode, odeset('RelTol',1e-6));

% Reconstruir Va, Vd, VR, VL para graficas
Va_ode = zeros(length(t_ode),1);
Vd_ode = zeros(length(t_ode),1);
VR_ode = zeros(length(t_ode),1);
VL_ode = zeros(length(t_ode),1);
for i = 1:length(t_ode)
    tref_th = get_ref(t_ode(i), theta_ref_rad, t_avance,   t_stop);
    tref_al = get_ref(t_ode(i), alpha_ref_rad, t_avance_a, t_stop_a);
    Xe1 = [X_ode(i,1)-tref_th; X_ode(i,2); X_ode(i,3); X_ode(i,4)];
    Xe2 = [X_ode(i,5)-tref_al; X_ode(i,6)];
    Va_ode(i) = max(min(-K1*Xe1, V_sat_a), -V_sat_a);
    Vd_ode(i) = max(min(-K2*Xe2, V_sat_d), -V_sat_d);
    VR_ode(i) = max(min(Va_ode(i)+Vd_ode(i), V_sat_f), -V_sat_f);
    VL_ode(i) = max(min(Va_ode(i)-Vd_ode(i), V_sat_f), -V_sat_f);
end

fprintf('[ODE] theta_final=%.3f  x_final=%.3f m  alpha_final=%.3f\n', ...
    rad2deg(X_ode(end,1)), X_ode(end,3), rad2deg(X_ode(end,5)));
fprintf('[ODE] theta %s  |  alpha %s\n', ...
    iif(abs(rad2deg(X_ode(end,1)))<1,'OK','REVISAR'), ...
    iif(abs(rad2deg(X_ode(end,5)))<1,'OK','REVISAR'));

%% =========================================================================
%  CARGAR MODELO EXISTENTE Y ACTUALIZAR PARAMETROS
% =========================================================================
modelName = 'Segway_Testbench_v4';

if ~exist([modelName '.slx'],'file')
    error('[ERROR] %s.slx no encontrado. Ejecuta build_v4.m primero.', modelName);
end

if ~bdIsLoaded(modelName)
    load_system(modelName);
    fprintf('[Run] Modelo cargado desde disco\n');
else
    fprintf('[Run] Modelo ya estaba cargado\n');
end

% Actualizar StopTime
set_param(modelName, 'StopTime', num2str(t_sim));

% Actualizar ganancias K1/K2 hardcodeadas dentro del MATLAB Function
func_str = sprintf([...
'function [VR, VL, Va, Vd] = LQR_Controller(theta, dtheta, x, dx, alpha, dalpha, theta_ref, alpha_ref)\n'...
'%% LQR K1+K2 desacoplados — ganancias hardcodeadas\n'...
'%%  K1: avance [theta,dtheta,x,dx]  K2: giro [alpha,dalpha]\n'...
'%%  Salidas: VR, VL (a motores)  Va, Vd (para F_x y tau_alpha)\n'...
'K1 = %s;\n'...
'K2 = %s;\n'...
'V_sat_a = %.4f;\n'...
'V_sat_d = %.4f;\n'...
'V_sat_f = %.4f;\n'...
'\n'...
'Xe1 = [theta - theta_ref; dtheta; x; dx];\n'...
'Va  = max(min(-K1 * Xe1, V_sat_a), -V_sat_a);\n'...
'\n'...
'Xe2 = [alpha - alpha_ref; dalpha];\n'...
'Vd  = max(min(-K2 * Xe2, V_sat_d), -V_sat_d);\n'...
'\n'...
'VR = max(min(Va + Vd, V_sat_f), -V_sat_f);\n'...
'VL = max(min(Va - Vd, V_sat_f), -V_sat_f);\n'...
'end\n'], mat2str(K1,6), mat2str(K2,6), V_sat_a, V_sat_d, V_sat_f);

rt = sfroot();
ctrl_chart = rt.find('-isa','Stateflow.EMChart','Path',[modelName '/LQR_Controller']);
ctrl_chart.Script = func_str;
fprintf('[Run] K1/K2 actualizados en LQR_Controller\n');

% Actualizar condiciones iniciales en joints
set_param([modelName '/Joint_theta'], 'PositionTargetValue', num2str(theta0_deg));
set_param([modelName '/Joint_alpha'], 'PositionTargetValue', num2str(alpha0_deg));

% Actualizar referencias
tref_str = num2str(theta_ref_deg*pi/180,'%.6f');
aref_str = num2str(alpha_ref_deg*pi/180,'%.6f');
set_param([modelName '/Step_th_on'],  'Time', num2str(t_avance), 'After', tref_str);
set_param([modelName '/Step_th_off'], 'Time', num2str(t_stop),   'After', ['-' tref_str]);
set_param([modelName '/Step_al_on'],  'Time', num2str(t_avance_a), 'After', aref_str);
set_param([modelName '/Step_al_off'], 'Time', num2str(t_stop_a),   'After', ['-' aref_str]);

fprintf('[Run] Parametros actualizados en modelo\n');

%% =========================================================================
%  12. SIMULAR
% =========================================================================
fprintf('[Sim] Iniciando simulacion de %g s...\n', t_sim);
try
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimscapeLogType','all','SimscapeLogName','simlog');
    out   = sim(simIn);
    fprintf('[Sim] OK  t_final=%.3f s\n', out.tout(end));
catch e
    fprintf('[Sim] FALLO: %s\n', e.message);
    return
end

%% =========================================================================
%  13. DIAGNOSTICO + GRAFICAS
% =========================================================================
try
    t_sm     = out.simlog.Joint_theta.Rz.q.series.time;
    theta_sm = out.simlog.Joint_theta.Rz.q.series.values('rad');
    alpha_sm = out.simlog.Joint_alpha.Rz.q.series.values('rad');

    fprintf('\n[Diagnostico Simscape]\n');
    fprintf('  theta(0) = %.4f rad (esperado %.4f)  %s\n', theta_sm(1), theta0, iif(abs(theta_sm(1)-theta0)<0.01,'OK','REVISAR'));
    fprintf('  alpha(0) = %.4f rad (esperado %.4f)  %s\n', alpha_sm(1), alpha0, iif(abs(alpha_sm(1)-alpha0)<0.01,'OK','REVISAR'));
    if abs(alpha_sm(1)) < 0.01 && abs(alpha0) > 0.01
        fprintf('  [ADVERTENCIA] alpha(0) Simscape ~ 0 pero alpha0=%.2f rad.\n', alpha0);
        fprintf('                Verificar PositionTargetSpecify en Joint_alpha.\n');
    end
    fprintf('  theta_final = %.3f  %s\n', rad2deg(theta_sm(end)), iif(abs(rad2deg(theta_sm(end)))<1,'OK','REVISAR'));
    fprintf('  alpha_final = %.3f  %s\n', rad2deg(alpha_sm(end)), iif(abs(rad2deg(alpha_sm(end)))<1,'OK','REVISAR'));

    t_U  = out.tout;
    VRVL = out.get('VRVL_log');
    VaVd = out.get('VaVd_log');

    % Figura 1: theta ODE vs Simscape
    figure('Name','v4 — Angulo de inclinacion','Position',[50 430 900 340]);
    hold on; grid on;
    plot(t_ode, rad2deg(X_ode(:,1)),'r--','LineWidth',2,'DisplayName','ODE + K1/K2');
    plot(t_sm,  rad2deg(theta_sm),  'b',  'LineWidth',2,'DisplayName','Simscape + K1/K2');
    yline(0,'--k','Equilibrio');
    xlabel('t [s]'); ylabel('\theta [deg]');
    title(sprintf('\\theta(t)  |  \\theta_0=%.0f  \\alpha_0=%.0f  |  Suelo plano', theta0_deg, alpha0_deg));
    legend('Location','best'); xlim([0 t_sim]);

    % Figura 2: VR y VL ODE vs Simscape
    figure('Name','v4 — VR y VL (motores)','Position',[50 60 900 310]);
    hold on; grid on;
    if ~isempty(VRVL) && size(VRVL,2)>=2
        plot(t_U, VRVL(:,1),'b',  'LineWidth',1.5,'DisplayName','V_R Simscape');
        plot(t_U, VRVL(:,2),'b--','LineWidth',1.5,'DisplayName','V_L Simscape');
    end
    plot(t_ode, VR_ode,'r',  'LineWidth',1.2,'DisplayName','V_R ODE');
    plot(t_ode, VL_ode,'r--','LineWidth',1.2,'DisplayName','V_L ODE');
    yline( V_sat_f,'--k'); yline(-V_sat_f,'--k');
    xlabel('t [s]'); ylabel('V [V]');
    title('VR y VL  |  VR~=VL mientras K2 corrige alpha');
    legend('Location','best'); xlim([0 t_sim]);

    % Figura 3: Va y Vd — evidencia desacoplamiento
    figure('Name','v4 — Va y Vd (desacoplamiento)','Position',[960 60 900 310]);
    hold on; grid on;
    plot(t_ode, Va_ode,'g',  'LineWidth',2,'DisplayName','Va (avance) ODE');
    plot(t_ode, Vd_ode,'m--','LineWidth',2,'DisplayName','Vd (giro)   ODE');
    if ~isempty(VaVd) && size(VaVd,2)>=2
        plot(t_U, VaVd(:,1),'g:','LineWidth',1.2,'DisplayName','Va Simscape');
        plot(t_U, VaVd(:,2),'m:','LineWidth',1.2,'DisplayName','Vd Simscape');
    end
    yline(0,'--k');
    xlabel('t [s]'); ylabel('V [V]');
    title('Desacoplamiento K1/K2  |  Va=avance  Vd=giro');
    legend('Location','best'); xlim([0 t_sim]);

catch e
    fprintf('[Graficas] FALLO: %s\n', e.message);
    disp(fieldnames(out));
end

fprintf('\n=========================================================\n');
fprintf(' LISTO v4\n');
fprintf(' Modificar perturbacion: theta0_deg | alpha0_deg\n');
fprintf(' Activar avance:  theta_ref_deg | t_avance | t_stop\n');
fprintf(' Activar giro:    alpha_ref_deg | t_avance_a | t_stop_a\n');
fprintf('=========================================================\n');

%% =========================================================================
%  FUNCIONES LOCALES
% =========================================================================
function s = iif(cond, a, b)
    if cond, s = a; else, s = b; end
end

function ref = get_ref(t, val, t_on, t_off)
    if t >= t_on && t < t_off
        ref = val;
    else
        ref = 0;
    end
end

function dX = ode_nl_6(t, X, K1, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    theta_ref_rad, alpha_ref_rad, t_avance, t_stop, t_avance_a, t_stop_a)

    % Referencias activas
    tref_th = get_ref(t, theta_ref_rad, t_avance,   t_stop);
    tref_al = get_ref(t, alpha_ref_rad, t_avance_a, t_stop_a);

    % K1 avance
    Xe1 = [X(1)-tref_th; X(2); X(3); X(4)];
    Va  = max(min(-K1*Xe1, V_sat_a), -V_sat_a);

    % K2 giro
    Xe2 = [X(5)-tref_al; X(6)];
    Vd  = max(min(-K2*Xe2, V_sat_d), -V_sat_d);

    % Mezcla + saturacion final
    VR = max(min(Va+Vd, V_sat_f), -V_sat_f);
    VL = max(min(Va-Vd, V_sat_f), -V_sat_f);

    % Torques sin back-EMF
    tau_R = alm*VR;
    tau_L = alm*VL;

    % Matrices de masa (no lineales)
    theta = X(1); dtheta = X(2);
    M11  = Icy + M*l^2;
    M12  = M*l*cos(theta);
    M22  = M + 2*m + 2*Iw/r^2;
    M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    detM = M11*M22 - M12^2;

    % Fuerzas generalizadas (ecuaciones de Kane, sin back-EMF)
    F1 = M*g*l*sin(theta) - (tau_R + tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R + tau_L)/r;
    F3 = d*(tau_R - tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / detM;
    ddx     = (-M12*F1 + M11*F2) / detM;
    ddalpha = F3 / M33;

    dX = [dtheta; ddtheta; X(4); ddx; X(6); ddalpha];
end
