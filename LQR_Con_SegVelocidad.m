%% LQR_Con_SegVelocidad.m
%  TESTBENCH — Segway Gemelo Digital | Lean-to-Speed en Simscape
%  ─ K1_aug (1x4): LQR avance aumentado [theta_err, dtheta, dx, int_ev]
%  ─ K2     (1x2): LQR giro [alpha, dalpha]
%  ─ v_ref = Kv * theta_rider  dentro del LQR_Controller
%  ─ Integrador Simulink con saturacion +-int_max (anti-windup nativo)
%  ─ MATLAB/Simulink R2025b
% =========================================================================
clear; clc; close all; bdclose all;
run('params.m');

%% ── MODELO MATEMÁTICO ────────────────────────────────────────────────────
mat_file = 'segway_modelo_resultados.mat';
if ~exist('A_num', 'var')
    if exist(mat_file, 'file')
        load(mat_file, 'A_num','B_num','A_ca','B_ca','A_giro','B_giro','polos');
        fprintf('Modelo cargado desde %s\n', mat_file);
    else
        fprintf('Corriendo ModelKane_final.m (solo esta vez)...\n');
        run('ModelKane_final.m');
    end
end
run('params.m');   % restaura variables numéricas (syms de ModelKane las sobreescribe)

%% =========================================================================
%  1. CONDICIONES INICIALES Y ESCENARIO
% =========================================================================
theta0_deg = 0;      % CI balanceo [deg] — rider sube vertical
alpha0_deg = 0;      % CI giro     [deg]
theta0     = theta0_deg*pi/180;
alpha0     = alpha0_deg*pi/180;

t_sim   = 20;
V_sat_a = 24;
V_sat_d = 24;
V_sat_f = 24;

% -- Lean-to-speed --
Kv        = 5.0;          % ganancia [m/s / rad]
th_lean   = deg2rad(5);   % inclinacion que aplica el rider [rad]
t_lean    = 3.0;          % rider se inclina [s]
t_back    = 13.0;         % rider se incorpora [s]
tau_rider = 0.3;          % constante de tiempo filtro rider [s]
v_max     = 2.0;          % velocidad maxima [m/s]
int_max   = 4.0;          % limite anti-windup integrador [m]

% -- Maniobra de giro --
alpha_ref_deg = 30;   % angulo de giro deseado [deg]
t_avance_a    = 8.0;  % instante en que gira  [s]
t_stop_a      = 12.0; % instante en que vuelve recto [s]
tau_alpha     = 1.2;  % filtro 1er orden sobre alpha_ref [s] — suaviza el escalon

fprintf('=========================================================\n');
fprintf(' TESTBENCH Segway v5 | Lean-to-Speed en Simscape\n');
fprintf(' Kv=%.1f m/s/rad  th_lean=%.0f deg  tau_rider=%.2f s  tau_alpha=%.2f s\n', Kv, rad2deg(th_lean), tau_rider, tau_alpha);
fprintf('=========================================================\n');

%% =========================================================================
%  2. LINEALIZACION Y LQR
%  M11, M12, M22, M33, det0 ya vienen de params.m
% =========================================================================

b21 = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41 = ( M12*(2*alm) + M11*(2*alm/r)) / det0;

% K1_aug: [theta_err, dtheta, dx, int_ev]
A1r = [0,              1, 0;
       M22*M*g*l/det0, 0, 0;
      -M12*M*g*l/det0, 0, 0];
B1r = [0; b21; b41];
A1a = [A1r, zeros(3,1); 0 0 1 0];
B1a = [B1r; 0];

rango1 = rank(ctrb(A1a, B1a));
fprintf('[K1aug] Controlabilidad: %d/4 %s\n', rango1, iif(rango1==4,'OK','REVISAR'));

Q1a = diag([2000, 100, 50, 50/(1.5*5)]);
R1  = 1;
[K1a, ~, eigs1] = lqr(A1a, B1a, Q1a, R1);
fprintf('[K1aug] K1a = [%.3f  %.3f  %.3f  %.3f]\n', K1a);
fprintf('[K1aug] Polos: '); fprintf('%.3f  ', real(eigs1)); fprintf('\n');

% K2: giro
A2=[0 1;0 0]; B2=[0; d*alm/(r*M33)];
rango2 = rank(ctrb(A2,B2));
fprintf('[K2]    Controlabilidad: %d/2 %s\n', rango2, iif(rango2==2,'OK','REVISAR'));
Q2=diag([1800,120]); R2=1;
[K2,~,eigs2] = lqr(A2,B2,Q2,R2);
fprintf('[K2]    K2 = [%.3f  %.3f]\n', K2);
fprintf('[K2]    Polos: '); fprintf('%.3f  ', real(eigs2)); fprintf('\n');

%% =========================================================================
%  3. ODE REFERENCIA — 8 estados [theta,dtheta,x,dx,alpha,dalpha,int_ev,rider]
% =========================================================================
X0_ode = zeros(9,1);
ode_fn = @(t,X) ode_lts_6(t, X, K1a, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    Kv, th_lean, t_lean, t_back, tau_rider, v_max, int_max, ...
    alpha_ref_deg*pi/180, t_avance_a, t_stop_a, tau_alpha);

[t_ode, X_ode] = ode45(ode_fn, [0 t_sim], X0_ode, odeset('RelTol',1e-6));

% Reconstruir Va, Vd, VR, VL
n = length(t_ode);
Va_ode=zeros(n,1); Vd_ode=zeros(n,1); VR_ode=zeros(n,1); VL_ode=zeros(n,1);
for i=1:n
    th_r   = X_ode(i,8);
    int_ev = max(min(X_ode(i,7), int_max), -int_max);
    Xe1 = [X_ode(i,1)-th_r; X_ode(i,2); X_ode(i,4); int_ev];
    Xe2 = [X_ode(i,5)-X_ode(i,9); X_ode(i,6)];
    Va_ode(i) = max(min(-K1a*Xe1, V_sat_a), -V_sat_a);
    Vd_ode(i) = max(min(-K2*Xe2,  V_sat_d), -V_sat_d);
    VR_ode(i) = max(min(Va_ode(i)+Vd_ode(i), V_sat_f), -V_sat_f);
    VL_ode(i) = max(min(Va_ode(i)-Vd_ode(i), V_sat_f), -V_sat_f);
end

v_ref_ode = min(Kv * X_ode(:,8), v_max);
fprintf('[ODE] dx_final=%.3f m/s  v_ref_final=%.3f  theta_final=%.3f deg\n',...
    X_ode(end,4), v_ref_ode(end), rad2deg(X_ode(end,1)));

%% =========================================================================
%  4-12. CONSTRUIR MODELO SIMSCAPE Y SIMULAR
% =========================================================================
modelName = 'Segway_Testbench_v5';
run('SimscapeConstruct.m');

%% =========================================================================
%  13. DIAGNOSTICO + GRAFICAS
% =========================================================================
try
    % ── Extraer desde ToWS (robusto, sin dependencia de simlog paths) ──
    t_sm     = out.tout;
    theta_sm = out.get('th_log');
    dtheta_sm= out.get('dth_log');
    x_sm     = out.get('x_log');
    dx_sm    = out.get('dx_log');
    alpha_sm = out.get('al_log');
    dalpha_sm= out.get('dal_log');
    t_U      = out.tout;
    intev_sm = out.get('intev_log');

    % ── Reconstruir Vd_ode ──────────────────────────────────────────────
    Vd_ode_local = zeros(length(t_ode),1);
    for ii = 1:length(t_ode)
        tref_al = alpha_ref_deg*pi/180 * (t_ode(ii)>=t_avance_a && t_ode(ii)<t_stop_a);
        Xe2 = [X_ode(ii,5)-X_ode(ii,9); X_ode(ii,6)];
        Vd_ode_local(ii) = max(min(-K2*Xe2, V_sat_d), -V_sat_d);
    end

    % ── Referencias vectorizadas ────────────────────────────────────────
    theta_rider_t = th_lean * double(t_ode >= t_lean & t_ode < t_back);
    v_ref_t       = min(Kv * X_ode(:,8), v_max);
    alpha_ref_t   = X_ode(:,9);  % alpha_ref filtrada (estado 9)

    fprintf('\n[Diagnostico]\n');
    fprintf('  theta_final  = %.3f deg  %s\n', rad2deg(theta_sm(end)), iif(abs(rad2deg(theta_sm(end)))<2,'OK','REVISAR'));
    fprintf('  alpha_final  = %.3f deg  %s\n', rad2deg(alpha_sm(end)), iif(abs(rad2deg(alpha_sm(end)))<5,'OK','REVISAR'));
    fprintf('  dx_final     = %.3f m/s\n', dx_sm(end));
    fprintf('  x_final      = %.3f m\n',  x_sm(end));

    % ── Colores ─────────────────────────────────────────────────────────
    BG    = [0.10 0.10 0.12];
    AX    = [0.16 0.16 0.19];
    GR    = [0.28 0.28 0.32];
    C_ode = [1.00 0.40 0.40];
    C_sm  = [0.35 0.75 1.00];
    C_ref = [0.65 0.65 0.65];
    C_vrf = [1.00 0.75 0.20];
    C_arf = [0.90 0.55 0.10];
    C_ev  = [0.30 0.90 0.60];
    C_gi  = [0.80 0.40 1.00];
    TXT   = [0.92 0.92 0.92];

    set(0,'DefaultFigureColor',    BG);
    set(0,'DefaultAxesColor',      AX);
    set(0,'DefaultAxesXColor',     TXT);
    set(0,'DefaultAxesYColor',     TXT);
    set(0,'DefaultAxesGridColor',  GR);
    set(0,'DefaultTextColor',      TXT);

    % Reconstruir VR, VL, Va, Vd desde Simscape logs
    VRVL_sm = out.get('VRVL_log');   % [VR VL]
    VaVd_sm = out.get('VaVd_log');   % [Va Vd]
    VR_sm = VRVL_sm(:,1);  VL_sm = VRVL_sm(:,2);
    Va_sm = VaVd_sm(:,1);  Vd_sm = VaVd_sm(:,2);

    fig1 = figure('Name','Segway v5 — Estados y Control',...
           'Color',BG,'Position',[40 30 1400 980]);

    LS = [0.18 0.18 0.22];  % color fondo leyenda
    AXs = {'Color',AX,'XColor',TXT,'YColor',TXT,'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',8};

    fig1 = figure('Name','Segway v5 — Estados y Control',...
           'Color',BG,'Position',[40 30 1400 980]);

    % ── 1. theta ────────────────────────────────────────────────────────
    ax1 = subplot(4,2,1); set(ax1,AXs{:}); hold on; grid on;
    plot(t_ode,rad2deg(X_ode(:,1)),'--','Color',C_ode,'LineWidth',1.6,'DisplayName','\theta ODE');
    plot(t_sm, rad2deg(theta_sm),       'Color',C_sm, 'LineWidth',2.0,'DisplayName','\theta Sim');
    plot(t_ode,rad2deg(theta_rider_t),':','Color',C_ref,'LineWidth',1.4,'DisplayName','\theta_{rider}');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    ylabel('[deg]','Color',TXT); title('\theta — pitch','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 2. dtheta ───────────────────────────────────────────────────────
    ax2 = subplot(4,2,3); set(ax2,AXs{:}); hold on; grid on;
    plot(t_ode,rad2deg(X_ode(:,2)),'--','Color',C_ode,'LineWidth',1.6,'DisplayName','d\theta ODE');
    plot(t_sm, rad2deg(dtheta_sm),      'Color',C_sm, 'LineWidth',2.0,'DisplayName','d\theta Sim');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    ylabel('[deg/s]','Color',TXT); title('d\theta/dt — vel. pitch','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 3. alpha ────────────────────────────────────────────────────────
    ax3 = subplot(4,2,5); set(ax3,AXs{:}); hold on; grid on;
    plot(t_ode,rad2deg(X_ode(:,5)),'--','Color',C_ode,'LineWidth',1.6,'DisplayName','\alpha ODE');
    plot(t_sm, rad2deg(alpha_sm),       'Color',C_sm, 'LineWidth',2.0,'DisplayName','\alpha Sim');
    plot(t_ode,rad2deg(alpha_ref_t),':','Color',C_arf,'LineWidth',1.8,'DisplayName','\alpha_{ref}');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    ylabel('[deg]','Color',TXT); title('\alpha — yaw','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 4. dalpha ───────────────────────────────────────────────────────
    ax4 = subplot(4,2,7); set(ax4,AXs{:}); hold on; grid on;
    plot(t_ode,rad2deg(X_ode(:,6)),'--','Color',C_ode,'LineWidth',1.6,'DisplayName','d\alpha ODE');
    plot(t_sm, rad2deg(dalpha_sm),      'Color',C_sm, 'LineWidth',2.0,'DisplayName','d\alpha Sim');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xlabel('t [s]','Color',TXT); ylabel('[deg/s]','Color',TXT);
    title('d\alpha/dt — vel. yaw','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 5. x ────────────────────────────────────────────────────────────
    ax5 = subplot(4,2,2); set(ax5,AXs{:}); hold on; grid on;
    plot(t_ode,X_ode(:,3),'--','Color',C_ode,'LineWidth',1.6,'DisplayName','x ODE');
    plot(t_sm, x_sm,           'Color',C_sm, 'LineWidth',2.0,'DisplayName','x Sim');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    ylabel('[m]','Color',TXT); title('x — posicion','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northwest','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 6. dx ───────────────────────────────────────────────────────────
    ax6 = subplot(4,2,4); set(ax6,AXs{:}); hold on; grid on;
    plot(t_ode,X_ode(:,4),'--','Color',C_ode,'LineWidth',1.6,'DisplayName','dx ODE');
    plot(t_sm, dx_sm,          'Color',C_sm, 'LineWidth',2.0,'DisplayName','dx Sim');
    plot(t_ode,v_ref_t,   ':',  'Color',C_vrf,'LineWidth',1.8,'DisplayName','v_{ref}');
    yline(v_max,'Color',C_vrf,'LineWidth',0.7,'LineStyle','--','HandleVisibility','off');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    ylabel('[m/s]','Color',TXT); title('dx — vel. avance','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 7. VR / VL ──────────────────────────────────────────────────────
    ax7 = subplot(4,2,6); set(ax7,AXs{:}); hold on; grid on;
    plot(t_ode,VR_ode,'--','Color',C_ode,'LineWidth',1.4,'DisplayName','V_R ODE');
    plot(t_ode,VL_ode,'--','Color',C_arf,'LineWidth',1.4,'DisplayName','V_L ODE');
    plot(t_U,  VR_sm,      'Color',C_sm, 'LineWidth',2.0,'DisplayName','V_R Sim');
    plot(t_U,  VL_sm,      'Color',C_vrf,'LineWidth',2.0,'DisplayName','V_L Sim');
    yline( V_sat_f,'--','Color',TXT,'LineWidth',0.6,'HandleVisibility','off');
    yline(-V_sat_f,'--','Color',TXT,'LineWidth',0.6,'HandleVisibility','off');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    ylabel('[V]','Color',TXT); title('V_R  /  V_L','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 8. Va / Vd ──────────────────────────────────────────────────────
    ax8 = subplot(4,2,8); set(ax8,AXs{:}); hold on; grid on;
    plot(t_ode,Va_ode,      '--','Color',C_ode,'LineWidth',1.4,'DisplayName','V_a ODE');
    plot(t_ode,Vd_ode_local,'--','Color',C_arf,'LineWidth',1.4,'DisplayName','V_d ODE');
    plot(t_U,  Va_sm,            'Color',C_sm, 'LineWidth',2.0,'DisplayName','V_a Sim');
    plot(t_U,  Vd_sm,            'Color',C_vrf,'LineWidth',2.0,'DisplayName','V_d Sim');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,'--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xlabel('t [s]','Color',TXT); ylabel('[V]','Color',TXT);
    title('V_a (avance)  /  V_d (giro)','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',7); set(lg,'Color',LS,'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8],'x');
    sgtitle(fig1, sprintf('Segway v5  |  K_v=%.1f  |  \\theta_{lean}=%.0f%s  |  \\alpha_{ref}=%.0f%s  |  %s',...
        Kv, rad2deg(th_lean), char(176), alpha_ref_deg, char(176), datestr(now,'HH:MM:SS')),...
        'FontSize',11,'FontWeight','bold','Color',TXT);

    % ── FIGURA 2: Trayectoria X vs Y en el plano ────────────────────────
    xw_ode = cumtrapz(t_ode, X_ode(:,4) .* cos(X_ode(:,5)));
    yw_ode = cumtrapz(t_ode, X_ode(:,4) .* sin(X_ode(:,5)));

    % Trayectoria en mundo: reconstruida desde dx_fwd y alpha (consistente con ODE)
    xw_sm = cumtrapz(t_sm, dx_sm .* cos(alpha_sm));
    yw_sm = cumtrapz(t_sm, dx_sm .* sin(alpha_sm));

    [~, i_lean]  = min(abs(t_sm - t_lean));
    [~, i_back]  = min(abs(t_sm - t_back));
    [~, i_giro]  = min(abs(t_sm - t_avance_a));
    [~, i_recto] = min(abs(t_sm - t_stop_a));

    fig2 = figure('Name','Segway v5 — Trayectoria X-Y + Alpha',...
           'Color',BG,'Position',[160 80 900 860]);

    % ── Subplot superior: trayectoria XY con flechas de orientación ─────
    ax_traj = subplot(3,1,[1 2]);
    set(ax_traj,'Color',AX,'XColor',TXT,'YColor',TXT,...
                'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',10);
    hold on; grid on; axis equal;

    plot(xw_ode, yw_ode, '--','Color',C_ode,'LineWidth',1.8,'DisplayName','ODE');

    n_sm = length(t_sm);
    cmap = cool(n_sm);
    for k = 1:n_sm-1
        plot([xw_sm(k) xw_sm(k+1)], [yw_sm(k) yw_sm(k+1)],...
             'Color',cmap(k,:),'LineWidth',2.5,'HandleVisibility','off');
    end
    patch(NaN,NaN,[0.35 0.75 1.00],'DisplayName','Simscape (cool = tiempo)');

    % ── Flechas de orientación alpha (heading) ──────────────────────────
    arr_len = 0.18;   % longitud flecha [m]
    n_arr   = 18;     % número de flechas

    % ODE: muestreo uniforme en tiempo
    idx_ode = round(linspace(1, length(t_ode), n_arr));
    qu_ode = quiver(xw_ode(idx_ode), yw_ode(idx_ode),...
                    arr_len*cos(X_ode(idx_ode,5)), arr_len*sin(X_ode(idx_ode,5)),...
                    0,'Color',C_ode,'LineWidth',1.2,'MaxHeadSize',0.8,...
                    'DisplayName','\alpha ODE');

    % Simscape: muestreo uniforme en tiempo
    idx_sm = round(linspace(1, length(t_sm), n_arr));
    quiver(xw_sm(idx_sm), yw_sm(idx_sm),...
           arr_len*cos(alpha_sm(idx_sm)), arr_len*sin(alpha_sm(idx_sm)),...
           0,'Color',C_sm,'LineWidth',1.2,'MaxHeadSize',0.8,...
           'DisplayName','\alpha Sim');

    plot(xw_sm(1),      yw_sm(1),      'o','Color',TXT,  'MarkerSize',8,'MarkerFaceColor',TXT,  'DisplayName','Inicio');
    plot(xw_sm(i_lean), yw_sm(i_lean), '^','Color',C_ev, 'MarkerSize',9,'MarkerFaceColor',C_ev, 'DisplayName','Inclina');
    plot(xw_sm(i_giro), yw_sm(i_giro), 's','Color',C_gi, 'MarkerSize',9,'MarkerFaceColor',C_gi, 'DisplayName','Gira');
    plot(xw_sm(i_recto),yw_sm(i_recto),'s','Color',C_arf,'MarkerSize',9,'MarkerFaceColor',C_arf,'DisplayName','Recto');
    plot(xw_sm(i_back), yw_sm(i_back), 'v','Color',C_ev, 'MarkerSize',9,'MarkerFaceColor',C_ev, 'DisplayName','Incorpora');
    plot(xw_sm(end),    yw_sm(end),    'p','Color',C_vrf,'MarkerSize',11,'MarkerFaceColor',C_vrf,'DisplayName','Final');

    cb = colorbar(ax_traj); colormap(ax_traj, cool);
    caxis([0 t_sim]);
    cb.Label.String = 't  [s]';
    cb.Color = TXT;
    cb.Label.Color = TXT;

    ylabel('y_{mundo}  [m]','Color',TXT,'FontSize',10);
    title('Trayectoria en el plano  (x_w  vs  y_w)  +  heading \alpha','Color',TXT,'FontSize',11,'FontWeight','bold');
    lg2 = legend('Location','best','FontSize',8);
    set(lg2,'Color',[0.18 0.18 0.22],'TextColor',TXT,'EdgeColor',GR);

    % ── Subplot inferior: alpha vs tiempo ────────────────────────────────
    ax_al = subplot(3,1,3);
    set(ax_al,'Color',AX,'XColor',TXT,'YColor',TXT,...
              'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',10);
    hold on; grid on;
    plot(t_ode, rad2deg(X_ode(:,5)),   '--','Color',C_ode,'LineWidth',1.8,'DisplayName','\alpha ODE');
    plot(t_sm,  rad2deg(alpha_sm),          'Color',C_sm, 'LineWidth',2.0,'DisplayName','\alpha Sim');
    plot(t_ode, rad2deg(alpha_ref_t),  ':' ,'Color',C_arf,'LineWidth',1.8,'DisplayName','\alpha_{ref}');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_avance_a,'--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_stop_a,  '--','Color',C_gi,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_lean,    '--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xline(t_back,    '--','Color',C_ev,'LineWidth',0.8,'HandleVisibility','off');
    xlabel('t  [s]','Color',TXT,'FontSize',10);
    ylabel('\alpha  [deg]','Color',TXT,'FontSize',10);
    title('\alpha — yaw ODE vs Simscape','Color',TXT,'FontSize',10,'FontWeight','bold');
    xlim([0 t_sim]);
    lg3 = legend('Location','northeast','FontSize',8);
    set(lg3,'Color',[0.18 0.18 0.22],'TextColor',TXT,'EdgeColor',GR);

    % enlazar eje X de alpha con el colorbar de tiempo
    xlabel(ax_traj,'x_{mundo}  [m]','Color',TXT,'FontSize',10);

    sgtitle(fig2, sprintf('Segway v5  |  \\alpha_{ref} = %.0f%s  |  K_v = %.1f  |  %s',...
        alpha_ref_deg, char(176), Kv, datestr(now,'HH:MM:SS')),...
        'FontSize',11,'FontWeight','bold','Color',TXT);

catch e
    fprintf('[Graficas] FALLO: %s\n', e.message);
end

% ── GUARDAR FIGURAS (fuera del try — siempre se ejecuta) ─────────────────
script_dir = fileparts(which('LQR_Con_SegVelocidad.m'));
fig_dir = fullfile(script_dir, 'debug');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
if exist('fig1','var') && ishandle(fig1)
    exportgraphics(fig1, fullfile(fig_dir, 'debug_estados.png'),     'Resolution', 150);
    fprintf('[Figuras] debug_estados.png guardada\n');
end
if exist('fig2','var') && ishandle(fig2)
    exportgraphics(fig2, fullfile(fig_dir, 'debug_trayectoria.png'), 'Resolution', 150);
    fprintf('[Figuras] debug_trayectoria.png guardada\n');
end

% ── DEBUG LOG ─────────────────────────────────────────────────────────────
try
    al_sm  = out.get('al_log');
    if ~exist('yw_ode','var')
        yw_ode = cumtrapz(t_ode, X_ode(:,4).*sin(X_ode(:,5)));
    end
    if ~exist('yw_sm','var')
        yw_sm = cumtrapz(t_sm, dx_sm .* sin(alpha_sm));
    end
    fid = fopen(fullfile(fig_dir,'debug_log.txt'),'w');
    fprintf(fid,'RUN: %s | Kv=%.1f th_lean=%.0fdeg alpha_ref=%.0fdeg t_sim=%gs\n',...
        datestr(now,'yyyy-mm-dd HH:MM:SS'), Kv, rad2deg(th_lean), alpha_ref_deg, t_sim);

    % Instantes clave para interpolacion
    tpts = [0, t_lean, t_avance_a, t_stop_a, t_back, t_sim];
    lbl  = {'t=0','t_lean','t_giro','t_recto','t_back','t_fin'};

    fprintf(fid,'\n%-10s %8s %8s %8s %8s %8s %8s %8s %8s %8s\n',...
        'Evento','th_ODE','th_SM','dx_ODE','dx_SM','al_ODE','al_SM','yw_ODE','yw_SM','slip');
    fprintf(fid,'%s\n', repmat('-',1,92));

    for k = 1:length(tpts)
        io = find(t_ode>=tpts(k),1); is = find(t_sm>=tpts(k),1);
        if isempty(io)||isempty(is), continue; end
        slip = 0;  % = 0 por construcción (Joint_fwd en marco heading)
        fprintf(fid,'%-10s %8.3f %8.3f %8.3f %8.3f %8.2f %8.2f %8.3f %8.3f %8.3f\n',...
            lbl{k},...
            rad2deg(X_ode(io,1)), rad2deg(theta_sm(is)),...
            X_ode(io,4),          dx_sm(is),...
            rad2deg(X_ode(io,5)), rad2deg(al_sm(is)),...
            yw_ode(io),           yw_sm(is),...
            slip);
    end

    fprintf(fid,'\nK1a = [%.3f %.3f %.3f %.3f]\n', K1a);
    fprintf(fid,'K2  = [%.3f %.3f]\n', K2);
    fclose(fid);
    fprintf('[Debug] debug_log.txt guardado\n');
catch e
    fprintf('[Debug] FALLO log: %s\n', e.message);
    if fid > 0, fclose(fid); end
end

fprintf('\n=========================================================\n');
fprintf(' LISTO v5\n');
fprintf(' Ajustar: Kv | th_lean | t_lean | t_back | tau_rider | v_max\n');
fprintf('=========================================================\n');

%% =========================================================================
%  FUNCIONES LOCALES
% =========================================================================
function s = iif(c,a,b); if c, s=a; else, s=b; end; end

function ref = get_ref(t, val, t_on, t_off)
    if t>=t_on && t<t_off, ref=val; else, ref=0; end
end

function dX = ode_lts_6(t, X, K1a, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    Kv, th_lean, t_lean, t_back, tau_rider, v_max, int_max, ...
    alpha_ref_rad, t_avance_a, t_stop_a, tau_alpha)

    % X = [theta,dtheta,x,dx,alpha,dalpha,int_ev,rider_filtrado,alpha_ref_filtrado]
    th_target = th_lean * (t>=t_lean && t<t_back);
    dX8 = (th_target - X(8)) / tau_rider;

    % Filtro 1er orden sobre alpha_ref (igual que rider sobre theta_ref)
    al_target = alpha_ref_rad * (t>=t_avance_a && t<t_stop_a);
    dX9 = (al_target - X(9)) / tau_alpha;

    th_r   = X(8);
    vr     = min(Kv*th_r, v_max);
    int_ev = max(min(X(7), int_max), -int_max);

    Xe1 = [X(1)-th_r; X(2); X(4); int_ev];
    Va  = max(min(-K1a*Xe1, V_sat_a), -V_sat_a);

    Xe2 = [X(5)-X(9); X(6)];
    Vd  = max(min(-K2*Xe2,  V_sat_d), -V_sat_d);

    VR = max(min(Va+Vd, V_sat_f), -V_sat_f);
    VL = max(min(Va-Vd, V_sat_f), -V_sat_f);
    tau_R = alm*VR; tau_L = alm*VL;

    theta=X(1); dtheta=X(2);
    M11  = Icy + M*l^2;
    M12  = M*l*cos(theta);
    M22  = M + 2*m + 2*Iw/r^2;
    M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    detM = M11*M22 - M12^2;

    F1 = M*g*l*sin(theta) - (tau_R+tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R+tau_L)/r;
    F3 = d*(tau_R-tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / detM;
    ddx     = (-M12*F1 + M11*F2) / detM;
    ddalpha = F3 / M33;

    ev = X(4) - vr;
    if (X(7)>= int_max && ev>0) || (X(7)<=-int_max && ev<0)
        d_int = 0;
    else
        d_int = ev;
    end

    dX = [dtheta; ddtheta; X(4); ddx; X(6); ddalpha; d_int; dX8; dX9];
end