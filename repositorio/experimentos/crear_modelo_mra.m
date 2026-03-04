%% =========================================================================
%  ANIMACIÓN 3D INTERACTIVA — Masa-Resorte-Amortiguador
%  
%  CONTROLES:
%  - Slider derecho: ajusta la posición inicial de la masa
%  - Botón "SOLTAR": suelta la masa desde la posición del slider
%  - Botón "DETENER": congela la masa en su posición actual
%  - La masa oscila con física real (resorte + amortiguador + gravedad)
%% =========================================================================
clearvars; clc; close all;

%% --- Parámetros físicos ---
m   = 3;
k   = 100;
c   = 10;
g   = 9.81;
delta_eq = m*g/k;
wn   = sqrt(k/m);
zeta = c/(2*sqrt(k*m));
wd   = wn*sqrt(1 - zeta^2);

%% --- Geometría ---
techo_y   = 0;
masa_lado = 0.06;
suelo_y   = -0.55;
spring_z  = -0.025;
damper_z  =  0.025;

% Límites físicos (posición Y del centro de masa)
y_lim_sup = techo_y - masa_lado/2 - 0.02;
y_lim_inf = suelo_y + masa_lado/2 + 0.02;

%% --- Estado ---
estado.pos       = -delta_eq;
estado.vel       = 0;
estado.libre     = false;    % false = sostenida por slider, true = oscilando
estado.running   = true;

%% --- Crear figura ---
fig = figure('Name', 'MRA Interactivo', ...
    'Color', [0.12 0.12 0.15], ...
    'Position', [150 80 1050 750], ...
    'NumberTitle', 'off', ...
    'CloseRequestFcn', @cerrarFig, ...
    'Resize', 'off');

%% --- Panel 3D (izquierda) ---
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.02 0.05 0.68 0.90]);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
set(ax, 'Color', [0.1 0.1 0.13], ...
    'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.3, ...
    'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], 'ZColor', [0.5 0.5 0.5]);
view(ax, [135 25]);
camup(ax, [0 1 0]);
xlim(ax, [-0.15 0.15]);
ylim(ax, [suelo_y - 0.05, 0.12]);
zlim(ax, [-0.15 0.15]);
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
title(ax, 'Masa – Resorte – Amortiguador', 'Color', 'w', 'FontSize', 14);
light('Position', [1 1 1]);
light('Position', [-1 -0.5 -1]);
lighting(ax, 'gouraud');

% Elementos estáticos
dibujarPlaca(ax, [0, techo_y+0.008, 0], [0.16, 0.016, 0.16], [0.35 0.35 0.4], 0.3);
dibujarPlaca(ax, [0, suelo_y, 0], [0.16, 0.016, 0.16], [0.5 0.5 0.55], 0.9);

% Línea de equilibrio (referencia visual)
plot3(ax, [-0.08 0.08], [-delta_eq -delta_eq], [0 0], '--', ...
    'Color', [0.3 0.8 0.3 0.4], 'LineWidth', 1);
text(ax, 0.09, -delta_eq, 0, 'eq', 'Color', [0.3 0.8 0.3], 'FontSize', 8);

% Elementos dinámicos (placeholders)
hSpring  = plot3(ax, 0, 0, 0);
hDBody   = surf(ax, [0 0;0 0], [0 0;0 0], [0 0;0 0], 'Visible', 'off');
hDPiston = surf(ax, [0 0;0 0], [0 0;0 0], [0 0;0 0], 'Visible', 'off');
hMasa    = patch(ax, 'Vertices', zeros(8,3), 'Faces', ones(6,4), 'FaceColor', 'b');

hTime = text(ax, -0.13, techo_y+0.09, 0, 't = 0.00 s', ...
    'Color', [1 1 0.3], 'FontSize', 11, 'FontWeight', 'bold');
hDisp = text(ax, -0.13, techo_y+0.07, 0, 'x = 0.000 m', ...
    'Color', [0.3 1 0.3], 'FontSize', 10);
hStatus = text(ax, -0.13, techo_y+0.05, 0, 'SOSTENIDA', ...
    'Color', [1 0.5 0.3], 'FontSize', 10, 'FontWeight', 'bold');

%% --- Panel de controles (derecha) ---
pnl = uipanel('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.73 0.05 0.25 0.90], ...
    'BackgroundColor', [0.18 0.18 0.22], ...
    'ForegroundColor', 'w', ...
    'Title', '  CONTROLES  ', ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'HighlightColor', [0.4 0.4 0.5]);

% --- Título slider ---
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05 0.92 0.9 0.06], ...
    'String', 'Posición inicial (m)', ...
    'BackgroundColor', [0.18 0.18 0.22], ...
    'ForegroundColor', [0.8 0.8 0.8], ...
    'FontSize', 10, 'HorizontalAlignment', 'center');

% --- Etiqueta valor actual del slider ---
hSliderVal = uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05 0.86 0.9 0.06], ...
    'String', sprintf('%.4f m', -delta_eq), ...
    'BackgroundColor', [0.18 0.18 0.22], ...
    'ForegroundColor', [0.3 1 0.5], ...
    'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

% --- Slider vertical ---
% Min = y_lim_inf, Max = y_lim_sup, inicial = -delta_eq
hSlider = uicontrol('Parent', pnl, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.3 0.22 0.4 0.62], ...
    'Min', y_lim_inf, 'Max', y_lim_sup, ...
    'Value', -delta_eq, ...
    'SliderStep', [0.005 0.05], ...
    'BackgroundColor', [0.3 0.3 0.4], ...
    'Callback', @sliderCallback);

% --- Etiquetas min/max ---
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05 0.16 0.9 0.05], ...
    'String', sprintf('Mín: %.3f m', y_lim_inf), ...
    'BackgroundColor', [0.18 0.18 0.22], ...
    'ForegroundColor', [0.6 0.6 0.6], ...
    'FontSize', 9, 'HorizontalAlignment', 'center');

uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05 0.84 0.9 0.03], ...
    'String', sprintf('Máx: %.3f m', y_lim_sup), ...
    'BackgroundColor', [0.18 0.18 0.22], ...
    'ForegroundColor', [0.6 0.6 0.6], ...
    'FontSize', 9, 'HorizontalAlignment', 'center');

% --- Botón SOLTAR ---
hBtnSoltar = uicontrol('Parent', pnl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.1 0.06 0.8 0.09], ...
    'String', 'SOLTAR', ...
    'FontSize', 14, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.2 0.7 0.3], ...
    'ForegroundColor', 'w', ...
    'Callback', @btnSoltarCallback);

% --- Botón DETENER ---
hBtnDetener = uicontrol('Parent', pnl, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.1 0.005 0.8 0.05], ...
    'String', 'DETENER', ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.8 0.3 0.2], ...
    'ForegroundColor', 'w', ...
    'Callback', @btnDetenerCallback);

%% --- Info parámetros ---
infoStr = sprintf('m=%.0f kg  k=%.0f N/m\nc=%.1f N·s/m\n\\omega_n=%.1f rad/s\n\\zeta=%.3f', ...
    m, k, c, wn, zeta);
uicontrol('Parent', pnl, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05 0.84 0.2 0.08], ...
    'String', '', ...
    'BackgroundColor', [0.18 0.18 0.22]);

%% --- Guardar appdata ---
setappdata(fig, 'estado', estado);
setappdata(fig, 'params', struct('m',m,'k',k,'c',c,'g',g,'wn',wn,...
    'zeta',zeta,'wd',wd,'delta_eq',delta_eq,'masa_lado',masa_lado,...
    'techo_y',techo_y,'spring_z',spring_z,'damper_z',damper_z,...
    'y_lim_sup',y_lim_sup,'y_lim_inf',y_lim_inf));
setappdata(fig, 'handles', struct('ax',ax,'hSpring',hSpring,...
    'hDBody',hDBody,'hDPiston',hDPiston,'hMasa',hMasa,...
    'hTime',hTime,'hDisp',hDisp,'hStatus',hStatus,...
    'hSlider',hSlider,'hSliderVal',hSliderVal));

%% =========================================================================
%  LOOP PRINCIPAL
%% =========================================================================
dt_sim  = 0.01;
t_total = 0;

while ishandle(fig)
    est = getappdata(fig, 'estado');
    if ~est.running; break; end
    
    if est.libre
        % --- Física: integración numérica ---
        F_spring  = -k * est.pos;
        F_gravity = -m * g;
        F_damper  = -c * est.vel;
        F_total   = F_spring + F_gravity + F_damper;
        
        acc = F_total / m;
        est.vel = est.vel + acc * dt_sim;
        est.pos = est.pos + est.vel * dt_sim;
        
        % Límites
        if est.pos > y_lim_sup
            est.pos = y_lim_sup;
            est.vel = -0.3 * est.vel;
        end
        if est.pos < y_lim_inf
            est.pos = y_lim_inf;
            est.vel = -0.3 * est.vel;
        end
        
        t_total = t_total + dt_sim;
    else
        % --- Modo sostenida: sigue al slider ---
        est.pos = get(hSlider, 'Value');
        est.vel = 0;
        t_total = 0;
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
        est.pos = get(src, 'Value');
        est.vel = 0;
        setappdata(fig, 'estado', est);
        h = getappdata(fig, 'handles');
        h.hSliderVal.String = sprintf('%.4f m', est.pos);
        setappdata(fig, 'handles', h);
    end
end

function btnSoltarCallback(src, ~)
    fig = ancestor(src, 'figure');
    est = getappdata(fig, 'estado');
    est.libre = true;
    est.vel = 0;
    setappdata(fig, 'estado', est);
    h = getappdata(fig, 'handles');
    h.hStatus.String = 'OSCILANDO';
    h.hStatus.Color = [0.3 1 0.5];
    setappdata(fig, 'handles', h);
end

function btnDetenerCallback(src, ~)
    fig = ancestor(src, 'figure');
    est = getappdata(fig, 'estado');
    est.libre = false;
    est.vel = 0;
    setappdata(fig, 'estado', est);
    
    % Actualizar slider a la posición actual
    h = getappdata(fig, 'handles');
    par = getappdata(fig, 'params');
    val = max(min(est.pos, par.y_lim_sup), par.y_lim_inf);
    set(h.hSlider, 'Value', val);
    h.hSliderVal.String = sprintf('%.4f m', val);
    h.hStatus.String = 'SOSTENIDA';
    h.hStatus.Color = [1 0.5 0.3];
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
    
    y_center = est.pos;
    y_top    = y_center + par.masa_lado/2;
    y_techo  = par.techo_y;
    
    delete(h.hSpring); delete(h.hDBody); delete(h.hDPiston); delete(h.hMasa);
    
    h.hSpring = dibujarResorte(h.ax, 0, par.spring_z, y_techo, y_top, 0.018, 10);
    [h.hDBody, h.hDPiston] = dibujarAmortiguador(h.ax, 0, par.damper_z, y_techo, y_top);
    h.hMasa = dibujarCubo(h.ax, [0, y_center, 0], par.masa_lado, [0.1 0.35 0.9]);
    
    desp = est.pos - (-par.delta_eq);
    h.hTime.String = sprintf('t = %.2f s', t_total);
    h.hDisp.String = sprintf('x = %.4f m', desp);
    
    % Actualizar slider visual si está oscilando
    if est.libre
        val = max(min(est.pos, par.y_lim_sup), par.y_lim_inf);
        set(h.hSlider, 'Value', val);
        h.hSliderVal.String = sprintf('%.4f m', est.pos);
    end
    
    setappdata(fig, 'handles', h);
end

%% =========================================================================
%  FUNCIONES DE DIBUJO
%% =========================================================================

function h = dibujarResorte(ax, cx, cz, y_top, y_bot, radio, nVueltas)
    nPts = nVueltas * 40;
    theta = linspace(0, nVueltas*2*pi, nPts);
    y = linspace(y_top, y_bot, nPts);
    x = cx + radio * cos(theta);
    z = cz + radio * sin(theta);
    h = plot3(ax, x, y, z, '-', 'Color', [0.9 0.55 0.05], 'LineWidth', 2.5);
end

function [hBody, hPiston] = dibujarAmortiguador(ax, cx, cz, y_top, y_bot)
    y_mid = y_top + (y_bot - y_top) * 0.4;
    nSeg = 16;
    r_ext = 0.010;
    theta = linspace(0, 2*pi, nSeg);
    xc = cx + r_ext * cos(theta);
    zc = cz + r_ext * sin(theta);
    X = [xc; xc]; Y = [repmat(y_top,1,nSeg); repmat(y_mid,1,nSeg)]; Z = [zc; zc];
    hBody = surf(ax, X, Y, Z, 'FaceColor', [0.5 0.5 0.55], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.8, 'FaceLighting', 'gouraud');
    r_int = 0.004;
    xp = cx + r_int * cos(theta);
    zp = cz + r_int * sin(theta);
    Xp = [xp; xp]; Yp = [repmat(y_mid,1,nSeg); repmat(y_bot,1,nSeg)]; Zp = [zp; zp];
    hPiston = surf(ax, Xp, Yp, Zp, 'FaceColor', [0.7 0.7 0.75], ...
        'EdgeColor', 'none', 'FaceLighting', 'gouraud');
end

function h = dibujarCubo(ax, centro, lado, color)
    l = lado/2;
    cx = centro(1); cy = centro(2); cz = centro(3);
    V = [cx-l cy-l cz-l; cx+l cy-l cz-l; cx+l cy+l cz-l; cx-l cy+l cz-l;
         cx-l cy-l cz+l; cx+l cy-l cz+l; cx+l cy+l cz+l; cx-l cy+l cz+l];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    h = patch(ax, 'Vertices', V, 'Faces', F, ...
        'FaceColor', color, 'EdgeColor', [0.05 0.05 0.05], ...
        'FaceAlpha', 0.95, 'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.4, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.3);
end

function h = dibujarPlaca(ax, centro, dims, color, alfa)
    if nargin < 5; alfa = 0.9; end
    lx = dims(1)/2; ly = dims(2)/2; lz = dims(3)/2;
    cx = centro(1); cy = centro(2); cz = centro(3);
    V = [cx-lx cy-ly cz-lz; cx+lx cy-ly cz-lz; cx+lx cy+ly cz-lz; cx-lx cy+ly cz-lz;
         cx-lx cy-ly cz+lz; cx+lx cy-ly cz+lz; cx+lx cy+ly cz+lz; cx-lx cy+ly cz+lz];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    h = patch(ax, 'Vertices', V, 'Faces', F, ...
        'FaceColor', color, 'EdgeColor', [0.2 0.2 0.2], ...
        'FaceAlpha', alfa, 'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
end