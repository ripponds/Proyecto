%% =========================================================================
%  ANIMACIÓN 3D INTERACTIVA — Sistema 2 DOF: m1-k1/c1-techo, m2-k2-m1
%
%  Sistema:
%    - k1 y c1 paralelos entre techo y m1
%    - k2 solo entre m1 y m2
%    - m2 libre en la parte inferior (sin restricción al suelo)
%
%  Ecuaciones en desplazamiento desde equilibrio (x positivo = hacia abajo):
%    m1*x1'' = -(k1+k2)*x1 + k2*x2 - c1*x1'
%    m2*x2'' =  k2*x1      - k2*x2
%
%  CONTROLES:
%    - Slider: ajusta posición inicial de m2 (x2_0) desde equilibrio
%    - SOLTAR : lanza oscilación libre desde posición del slider (x1=0, x2=slider)
%    - RESET  : detiene todo, vuelve a equilibrio, funciona siempre
%% =========================================================================
clearvars; clc; close all;

%% =========================================================================
%  PARÁMETROS FÍSICOS
%% =========================================================================
m1   = 1.0;          % kg
m2   = 0.5;          % kg
k1   = 40.0;         % N/m  (techo → m1)
k2   = 20.0;         % N/m  (m1   → m2)
c1   = 3.0;          % N·s/m (techo → m1, paralelo a k1)
g    = 9.81;         % m/s²

% Posiciones de equilibrio estático (medidas desde techo, hacia abajo positivo)
% En equilibrio: k1*y1_eq = m1*g + k2*(y1_eq - y2_eq + L2) → trabajamos en desplaz.
% delta1_eq = (m1+m2)*g / k1   (elongación de k1 en equilibrio)
% delta2_eq = m2*g / k2         (elongación de k2 en equilibrio)
delta1_eq = (m1 + m2) * g / k1;   % = 0.736 m
delta2_eq = m2 * g / k2;           % = 0.491 m

% Frecuencias (verificación)
% Matriz M^-1 K con K=[k1+k2,-k2;-k2,k2], M=diag([m1,m2])
K_mat = [(k1+k2)/m1, -k2/m1; -k2/m2, k2/m2];
omega2 = eig(K_mat);
omega_modes = sqrt(sort(omega2));    % [ω1, ω2] rad/s
zeta1 = c1 / (2 * sqrt((k1+k2)*m1));% amortiguamiento modo dominante approx

%% =========================================================================
%  GEOMETRÍA (eje Y vertical, Y=0 en el techo, Y negativo hacia abajo)
%% =========================================================================
techo_y    = 0;
masa_lado  = 0.08;          % lado del cubo de masa (m)
m1_gap     = 0.04;          % separación mínima entre techo y m1
m2_gap     = 0.04;          % separación mínima entre m1 y m2

% Longitudes naturales de resortes (sin carga)
L1_nat = delta1_eq - masa_lado/2 - m1_gap;   % aprox longitud visual natural k1
L2_nat = delta2_eq - masa_lado/2 - m2_gap;   % aprox longitud visual natural k2
% Asegurar valores positivos
L1_nat = max(L1_nat, 0.10);
L2_nat = max(L2_nat, 0.10);

% Posiciones Y del centro de cada masa en equilibrio (negativo = debajo del techo)
y1_eq = -(masa_lado/2 + m1_gap + L1_nat);    
y2_eq = y1_eq - masa_lado/2 - m2_gap - L2_nat - masa_lado/2;

% Separación en Z para resorte vs amortiguador (visibilidad)
spring_z = -0.028;
damper_z =  0.028;

% Límites físicos para x2 (desplazamiento de m2 desde su equilibrio)
% Sup: m2 no puede subir tanto que toque m1 (cara inf de m1 - margen)
x2_lim_sup =  0.0;    % m2 no sube por encima de equilibrio (conservador)
x2_lim_inf = -0.28;   % m2 puede bajar 28 cm desde equilibrio

% Límites físicos para x1 (se calculan para colisiones en runtime)
x1_hard_sup = -(y1_eq - masa_lado) + techo_y - masa_lado/2 - 0.01;  % no atraviesa techo

%% =========================================================================
%  ESTADO INICIAL
%% =========================================================================
estado.x1    = 0.0;    % desplazamiento m1 desde equilibrio (+ = abajo)
estado.x2    = 0.0;    % desplazamiento m2 desde equilibrio (+ = abajo)
estado.v1    = 0.0;
estado.v2    = 0.0;
estado.libre  = false;
estado.running = true;

%% =========================================================================
%  FIGURA
%% =========================================================================
fig = figure('Name', '2-DOF Mass-Spring-Damper', ...
    'Color', [0.10 0.10 0.13], ...
    'Position', [100 60 1120 780], ...
    'NumberTitle', 'off', ...
    'CloseRequestFcn', @cerrarFig, ...
    'Resize', 'off');

%% --- Axes 3D ---
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.02 0.04 0.70 0.93]);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
set(ax, ...
    'Color',     [0.08 0.08 0.11], ...
    'GridColor', [0.28 0.28 0.32], 'GridAlpha', 0.3, ...
    'XColor',    [0.45 0.45 0.5], ...
    'YColor',    [0.45 0.45 0.5], ...
    'ZColor',    [0.45 0.45 0.5], ...
    'FontSize',  9);
view(ax, [140 22]);
camup(ax, [0 1 0]);

% Rango de vista con margen holgado
y_view_top = techo_y + 0.10;
y_view_bot = y2_eq - 0.30;
xlim(ax, [-0.20 0.20]);
ylim(ax, [y_view_bot, y_view_top]);
zlim(ax, [-0.20 0.20]);
xlabel(ax, 'X', 'Color', [0.5 0.5 0.55]);
ylabel(ax, 'Y', 'Color', [0.5 0.5 0.55]);
zlabel(ax, 'Z', 'Color', [0.5 0.5 0.55]);
title(ax, 'Sistema 2-DOF  |  k_1·c_1 (techo→m_1)  ·  k_2 (m_1→m_2)', ...
    'Color', [0.88 0.88 0.92], 'FontSize', 13, 'FontWeight', 'bold');

% Iluminación
light('Position', [1  1.5  1]);
light('Position', [-1 -0.5 -1]);
lighting(ax, 'gouraud');

%% --- Elementos estáticos ---
% Techo
dibujarPlaca(ax, [0, techo_y + 0.010, 0], [0.22, 0.020, 0.22], [0.32 0.32 0.38], 0.35);

% Líneas de equilibrio (referencias visuales)
plot3(ax, [-0.12 0.12], [y1_eq y1_eq], [0 0], '--', ...
    'Color', [0.25 0.75 0.25 0.35], 'LineWidth', 1.0);
plot3(ax, [-0.12 0.12], [y2_eq y2_eq], [0 0], '--', ...
    'Color', [0.25 0.55 0.90 0.35], 'LineWidth', 1.0);
text(ax,  0.13, y1_eq, 0, 'eq₁', 'Color', [0.3 0.85 0.3],  'FontSize', 8);
text(ax,  0.13, y2_eq, 0, 'eq₂', 'Color', [0.4 0.65 1.0],  'FontSize', 8);

%% --- Handles dinámicos (placeholders) ---
hSpr1  = plot3(ax, 0, 0, 0, 'Color', [0.95 0.55 0.05], 'LineWidth', 2.2);
hSpr2  = plot3(ax, 0, 0, 0, 'Color', [0.95 0.55 0.05], 'LineWidth', 2.2);
hDB1   = surf(ax, zeros(2), zeros(2), zeros(2), 'Visible', 'off');
hDP1   = surf(ax, zeros(2), zeros(2), zeros(2), 'Visible', 'off');
hMasa1 = patch(ax, 'Vertices', zeros(8,3), 'Faces', ones(6,4), 'FaceColor', [0.1 0.35 0.85]);
hMasa2 = patch(ax, 'Vertices', zeros(8,3), 'Faces', ones(6,4), 'FaceColor', [0.75 0.25 0.25]);

% Etiquetas sobre masas
hLm1 = text(ax, 0, y1_eq, 0.07, 'm₁', 'Color', [0.6 0.8 1.0], ...
    'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
hLm2 = text(ax, 0, y2_eq, 0.07, 'm₂', 'Color', [1.0 0.65 0.65], ...
    'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Telemetría
hTime  = text(ax, -0.18, y_view_top - 0.01, 0, 't = 0.00 s', ...
    'Color', [1.0 0.95 0.3], 'FontSize', 11, 'FontWeight', 'bold');
hDisp1 = text(ax, -0.18, y_view_top - 0.03, 0, 'x₁ = 0.0000 m', ...
    'Color', [0.4 0.85 0.4],  'FontSize', 10);
hDisp2 = text(ax, -0.18, y_view_top - 0.05, 0, 'x₂ = 0.0000 m', ...
    'Color', [0.9 0.55 0.55],  'FontSize', 10);
hStatus = text(ax, -0.18, y_view_top - 0.07, 0, '● SOSTENIDA', ...
    'Color', [1.0 0.55 0.25], 'FontSize', 10, 'FontWeight', 'bold');

%% =========================================================================
%  PANEL DE CONTROLES (derecha)
%% =========================================================================
pnl = uipanel('Parent', fig, ...
    'Units', 'normalized', 'Position', [0.745 0.04 0.245 0.93], ...
    'BackgroundColor', [0.14 0.14 0.18], ...
    'ForegroundColor', [0.85 0.85 0.90], ...
    'Title', '  CONTROLES  ', ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'HighlightColor', [0.38 0.38 0.48]);

% ---- Bloque parámetros (info estática) ----
infoStr = sprintf('m₁ = %.1f kg    m₂ = %.1f kg\nk₁ = %.0f N/m   k₂ = %.0f N/m\nc₁ = %.1f N·s/m\nω₁ = %.2f rad/s\nω₂ = %.2f rad/s\nζ  ≈ %.3f', ...
    m1, m2, k1, k2, c1, omega_modes(1), omega_modes(2), zeta1);
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.80 0.92 0.18], ...
    'String', infoStr, ...
    'BackgroundColor', [0.11 0.11 0.15], ...
    'ForegroundColor', [0.65 0.70 0.80], ...
    'FontSize', 9, 'HorizontalAlignment', 'left');

% ---- Separador visual ----
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.785 0.92 0.010], ...
    'String', '', 'BackgroundColor', [0.30 0.30 0.40]);

% ---- Título slider ----
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.730 0.92 0.050], ...
    'String', 'Posición inicial  m₂  (Δ desde eq)', ...
    'BackgroundColor', [0.14 0.14 0.18], ...
    'ForegroundColor', [0.78 0.78 0.85], ...
    'FontSize', 9.5, 'HorizontalAlignment', 'center');

% ---- Etiqueta max ----
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.705 0.92 0.028], ...
    'String', sprintf('▲  Máx: %.3f m', x2_lim_sup), ...
    'BackgroundColor', [0.14 0.14 0.18], ...
    'ForegroundColor', [0.50 0.50 0.58], ...
    'FontSize', 8.5, 'HorizontalAlignment', 'center');

% ---- Etiqueta valor actual slider ----
hSliderVal = uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.675 0.92 0.032], ...
    'String', '0.0000 m', ...
    'BackgroundColor', [0.14 0.14 0.18], ...
    'ForegroundColor', [1.0 0.70 0.25], ...
    'FontSize', 15, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

% ---- Slider vertical ----
hSlider = uicontrol('Parent', pnl, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.30 0.22 0.40 0.45], ...
    'Min', x2_lim_inf, 'Max', x2_lim_sup, ...
    'Value', 0.0, ...
    'SliderStep', [0.005 0.04], ...
    'BackgroundColor', [0.28 0.28 0.38], ...
    'Callback', @sliderCallback);

% ---- Etiqueta min ----
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04 0.190 0.92 0.028], ...
    'String', sprintf('▼  Mín: %.3f m', x2_lim_inf), ...
    'BackgroundColor', [0.14 0.14 0.18], ...
    'ForegroundColor', [0.50 0.50 0.58], ...
    'FontSize', 8.5, 'HorizontalAlignment', 'center');

% ---- Botón SOLTAR ----
hBtnSoltar = uicontrol('Parent', pnl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.08 0.095 0.84 0.088], ...
    'String', '▶  SOLTAR', ...
    'FontSize', 13, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.18 0.65 0.28], ...
    'ForegroundColor', 'w', ...
    'Callback', @btnSoltarCallback);

% ---- Botón RESET ----
hBtnReset = uicontrol('Parent', pnl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.08 0.010 0.84 0.075], ...
    'String', '↺  RESET', ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.65 0.20 0.18], ...
    'ForegroundColor', 'w', ...
    'Callback', @btnResetCallback);

%% =========================================================================
%  GUARDAR APPDATA
%% =========================================================================
setappdata(fig, 'estado', estado);

setappdata(fig, 'params', struct( ...
    'm1', m1, 'm2', m2, 'k1', k1, 'k2', k2, 'c1', c1, 'g', g, ...
    'delta1_eq', delta1_eq, 'delta2_eq', delta2_eq, ...
    'y1_eq', y1_eq, 'y2_eq', y2_eq, ...
    'masa_lado', masa_lado, ...
    'techo_y', techo_y, ...
    'spring_z', spring_z, 'damper_z', damper_z, ...
    'x2_lim_sup', x2_lim_sup, 'x2_lim_inf', x2_lim_inf, ...
    'x1_hard_sup', x1_hard_sup));

setappdata(fig, 'handles', struct( ...
    'ax', ax, ...
    'hSpr1', hSpr1, 'hSpr2', hSpr2, ...
    'hDB1', hDB1,   'hDP1', hDP1, ...
    'hMasa1', hMasa1, 'hMasa2', hMasa2, ...
    'hLm1', hLm1, 'hLm2', hLm2, ...
    'hTime', hTime, 'hDisp1', hDisp1, 'hDisp2', hDisp2, ...
    'hStatus', hStatus, ...
    'hSlider', hSlider, 'hSliderVal', hSliderVal));

%% =========================================================================
%  LOOP PRINCIPAL
%% =========================================================================
dt      = 0.012;     % paso de integración (s) — tranquilo y fluido
t_total = 0.0;

while ishandle(fig)
    est = getappdata(fig, 'estado');
    if ~est.running; break; end
    par = getappdata(fig, 'params');

    if est.libre
        %-- Fuerzas (desplazamiento desde equilibrio, + = hacia abajo) --%
        F1 = -(par.k1 + par.k2) * est.x1 + par.k2 * est.x2 - par.c1 * est.v1;
        F2 =   par.k2 * est.x1  - par.k2 * est.x2;

        %-- Euler semi-implícito (vel primero, luego pos) --%
        est.v1 = est.v1 + (F1 / par.m1) * dt;
        est.v2 = est.v2 + (F2 / par.m2) * dt;
        est.x1 = est.x1 + est.v1 * dt;
        est.x2 = est.x2 + est.v2 * dt;

        %-- Límite físico: m2 no sube por encima de cara inferior de m1 --%
        % y_center_m2 = y2_eq - est.x2  (negativo = abajo)
        % y_center_m1 = y1_eq - est.x1
        % gap = (y_center_m1 - masa_lado/2) - (y_center_m2 + masa_lado/2)
        gap_12 = (par.y1_eq - est.x1 - par.masa_lado/2) - ...
                 (par.y2_eq - est.x2 + par.masa_lado/2);
        if gap_12 < 0.005
            % Colisión suave m1-m2
            est.v2 = -0.30 * est.v2;
            est.x2 = est.x2 + gap_12 - 0.005;
        end

        %-- Límite superior: x2 no excede x2_lim_sup --%
        if est.x2 > par.x2_lim_sup
            est.x2 = par.x2_lim_sup;
            est.v2 = -0.30 * abs(est.v2);
        end
        %-- Límite inferior: x2 no excede x2_lim_inf --%
        if est.x2 < par.x2_lim_inf
            est.x2 = par.x2_lim_inf;
            est.v2 =  0.30 * abs(est.v2);
        end

        %-- Límite m1: no atraviesa techo --%
        if est.x1 < -( abs(par.y1_eq) - par.masa_lado/2 - 0.02 )
            est.x1 = -( abs(par.y1_eq) - par.masa_lado/2 - 0.02 );
            est.v1 = -0.30 * est.v1;
        end

        t_total = t_total + dt;

        % Actualizar slider visualmente
        val_sl = max(min(est.x2, par.x2_lim_sup), par.x2_lim_inf);
        h_tmp  = getappdata(fig, 'handles');
        set(h_tmp.hSlider, 'Value', val_sl);
        h_tmp.hSliderVal.String = sprintf('%.4f m', est.x2);
        setappdata(fig, 'handles', h_tmp);
    else
        % Modo sostenida: m2 sigue al slider, m1 en equilibrio
        h_tmp = getappdata(fig, 'handles');
        est.x2 = get(h_tmp.hSlider, 'Value');
        est.x1 = 0.0;
        est.v1 = 0.0;
        est.v2 = 0.0;
        t_total = 0.0;
    end

    setappdata(fig, 'estado', est);
    actualizarGraficos(fig, t_total);
    drawnow limitrate;
    pause(0.008);
end

%% =========================================================================
%  CALLBACKS
%% =========================================================================

function sliderCallback(src, ~)
    fig = ancestor(src, 'figure');
    est = getappdata(fig, 'estado');
    if ~est.libre
        est.x2 = get(src, 'Value');
        setappdata(fig, 'estado', est);
        h = getappdata(fig, 'handles');
        h.hSliderVal.String = sprintf('%.4f m', est.x2);
        setappdata(fig, 'handles', h);
    end
end

function btnSoltarCallback(src, ~)
    fig = ancestor(src, 'figure');
    est = getappdata(fig, 'estado');
    est.libre = true;
    est.v1 = 0;
    est.v2 = 0;
    setappdata(fig, 'estado', est);
    h = getappdata(fig, 'handles');
    h.hStatus.String = '● OSCILANDO';
    h.hStatus.Color  = [0.3 1.0 0.45];
    setappdata(fig, 'handles', h);
end

function btnResetCallback(src, ~)
    fig = ancestor(src, 'figure');
    est = getappdata(fig, 'estado');
    est.x1   = 0;  est.x2   = 0;
    est.v1   = 0;  est.v2   = 0;
    est.libre = false;
    setappdata(fig, 'estado', est);
    h = getappdata(fig, 'handles');
    par = getappdata(fig, 'params');
    set(h.hSlider, 'Value', 0);
    h.hSliderVal.String = '0.0000 m';
    h.hStatus.String    = '● SOSTENIDA';
    h.hStatus.Color     = [1.0 0.55 0.25];
    setappdata(fig, 'handles', h);
end

function cerrarFig(src, ~)
    est = getappdata(src, 'estado');
    est.running = false;
    setappdata(src, 'estado', est);
    delete(src);
end

%% =========================================================================
%  ACTUALIZAR GRÁFICOS
%% =========================================================================

function actualizarGraficos(fig, t_total)
    est = getappdata(fig, 'estado');
    par = getappdata(fig, 'params');
    h   = getappdata(fig, 'handles');

    % Posición Y actual de cada masa (Y negativo = debajo del techo)
    y1 = par.y1_eq - est.x1;   % x positivo = baja = Y más negativo
    y2 = par.y2_eq - est.x2;

    y1_top = y1 + par.masa_lado/2;
    y1_bot = y1 - par.masa_lado/2;
    y2_top = y2 + par.masa_lado/2;

    % Borrar handles dinámicos anteriores
    delete(h.hSpr1); delete(h.hSpr2);
    delete(h.hDB1);  delete(h.hDP1);
    delete(h.hMasa1); delete(h.hMasa2);

    % k1 + c1: techo → tope de m1
    h.hSpr1 = dibujarResorte(h.ax, 0, par.spring_z, par.techo_y, y1_top, 0.020, 9);
    [h.hDB1, h.hDP1] = dibujarAmortiguador(h.ax, 0, par.damper_z, par.techo_y, y1_top);

    % k2: fondo de m1 → tope de m2
    h.hSpr2 = dibujarResorte(h.ax, 0, 0, y1_bot, y2_top, 0.018, 7);

    % Masas
    h.hMasa1 = dibujarCubo(h.ax, [0, y1, 0], par.masa_lado, [0.12 0.38 0.88]);
    h.hMasa2 = dibujarCubo(h.ax, [0, y2, 0], par.masa_lado * 0.85, [0.82 0.22 0.22]);

    % Etiquetas posición masas
    h.hLm1.Position = [0, y1, 0.07];
    h.hLm2.Position = [0, y2, 0.065];

    % Telemetría
    h.hTime.String  = sprintf('t = %.2f s', t_total);
    h.hDisp1.String = sprintf('x₁ = %+.4f m', est.x1);
    h.hDisp2.String = sprintf('x₂ = %+.4f m', est.x2);

    setappdata(fig, 'handles', h);
end

%% =========================================================================
%  FUNCIONES DE DIBUJO
%% =========================================================================

function h = dibujarResorte(ax, cx, cz, y_top, y_bot, radio, nVueltas)
    nPts  = nVueltas * 36;
    theta = linspace(0, nVueltas * 2 * pi, nPts);
    % Segmento recto inicial y final (extremos del resorte)
    n_recto = round(nPts * 0.05);
    y_helic = linspace(y_top, y_bot, nPts);
    r = zeros(1, nPts);
    r(n_recto+1 : end-n_recto) = radio;
    x = cx + r .* cos(theta);
    z = cz + r .* sin(theta);
    h = plot3(ax, x, y_helic, z, '-', ...
        'Color', [0.95 0.58 0.05], 'LineWidth', 2.4);
end

function [hBody, hPiston] = dibujarAmortiguador(ax, cx, cz, y_top, y_bot)
    nSeg  = 18;
    r_ext = 0.011;
    r_int = 0.005;
    theta = linspace(0, 2*pi, nSeg);
    xc = cx + r_ext * cos(theta);
    zc = cz + r_ext * sin(theta);

    % Cuerpo exterior (40% superior del recorrido)
    y_mid = y_top + (y_bot - y_top) * 0.42;
    X  = [xc; xc];
    Y  = [repmat(y_top, 1, nSeg); repmat(y_mid, 1, nSeg)];
    Z  = [zc; zc];
    hBody = surf(ax, X, Y, Z, ...
        'FaceColor', [0.48 0.50 0.56], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.85, 'FaceLighting', 'gouraud');

    % Pistón interior (60% inferior)
    xp = cx + r_int * cos(theta);
    zp = cz + r_int * sin(theta);
    Xp = [xp; xp];
    Yp = [repmat(y_mid, 1, nSeg); repmat(y_bot, 1, nSeg)];
    Zp = [zp; zp];
    hPiston = surf(ax, Xp, Yp, Zp, ...
        'FaceColor', [0.72 0.72 0.76], 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud');
end

function h = dibujarCubo(ax, centro, lado, color)
    l  = lado / 2;
    cx = centro(1); cy = centro(2); cz = centro(3);
    V = [ cx-l cy-l cz-l;  cx+l cy-l cz-l;  cx+l cy+l cz-l;  cx-l cy+l cz-l; ...
          cx-l cy-l cz+l;  cx+l cy-l cz+l;  cx+l cy+l cz+l;  cx-l cy+l cz+l ];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    h = patch(ax, 'Vertices', V, 'Faces', F, ...
        'FaceColor', color, 'EdgeColor', [0.04 0.04 0.06], ...
        'FaceAlpha', 0.94, 'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.38, 'DiffuseStrength', 0.82, 'SpecularStrength', 0.32);
end

function h = dibujarPlaca(ax, centro, dims, color, alfa)
    if nargin < 5; alfa = 0.9; end
    lx = dims(1)/2; ly = dims(2)/2; lz = dims(3)/2;
    cx = centro(1); cy = centro(2); cz = centro(3);
    V = [ cx-lx cy-ly cz-lz;  cx+lx cy-ly cz-lz; ...
          cx+lx cy+ly cz-lz;  cx-lx cy+ly cz-lz; ...
          cx-lx cy-ly cz+lz;  cx+lx cy-ly cz+lz; ...
          cx+lx cy+ly cz+lz;  cx-lx cy+ly cz+lz ];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    h = patch(ax, 'Vertices', V, 'Faces', F, ...
        'FaceColor', color, 'EdgeColor', [0.18 0.18 0.22], ...
        'FaceAlpha', alfa, 'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
end
