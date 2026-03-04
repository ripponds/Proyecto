%% doble_resorte.m
% Masa colgante con dos resortes (3D isométrico) + UI (sliders + botones)
% Z es vertical. x(t) es desplazamiento respecto a equilibrio (positivo hacia abajo).

clearvars; clc; close all;

%% -------------------- Parámetros base --------------------
P.m    = 1.0;     % kg
P.k1   = 5.0;     % N/m
P.k2   = 5.0;     % N/m
P.k    = P.k1 + P.k2;

P.zeta = 0.10;    % 0.05 a 0.30 típico
P.c    = 2*P.zeta*sqrt(P.m*P.k);

P.dt   = 0.008;   % s
P.tMax = 12.0;    % s

P.x0   = 0.05;    % m (5 cm abajo)
P.v0   = 0.00;    % m/s

%% -------------------- Geometría visual (Z vertical) --------------------
G.zCeil  = 0.60;
G.zEq    = 0.20;
G.zFloor = -0.25;

G.blkW = 0.28;  % X
G.blkD = 0.20;  % Y
G.blkH = 0.16;  % Z

G.xA1 = -0.16; G.yA1 = 0.0;
G.xA2 =  0.16; G.yA2 = 0.0;

G.xB1 = G.xA1; G.yB1 = 0.0;
G.xB2 = G.xA2; G.yB2 = 0.0;

G.zMax = G.zCeil  - 0.12;
G.zMin = G.zFloor + 0.10;

G.springRadius = 0.020;
G.nTurns       = 10;
G.nPts         = max(240, G.nTurns*40);

%% -------------------- Estado --------------------
S.x = P.x0;
S.v = P.v0;
S.running = true;   % animación activa
S.free    = true;   % libre = integra; no libre = sigue slider

%% -------------------- UI layout --------------------
fig = figure('Name','Masa + 2 Resortes (3D) + Sliders','Color',[0.06 0.06 0.07], ...
             'NumberTitle','off');

ax = axes('Parent',fig,'Position',[0.05 0.08 0.68 0.87]);
hold(ax,'on'); axis(ax,'equal');

panel = uipanel('Parent',fig,'Title','Controles','ForegroundColor',[1 1 1], ...
                'BackgroundColor',[0.10 0.10 0.12], 'Position',[0.76 0.08 0.22 0.87]);

set(ax,'Color',[0.05 0.05 0.06], ...
       'XColor',[1 1 1], 'YColor',[1 1 1], 'ZColor',[1 1 1], ...
       'GridColor',[0.75 0.75 0.75], 'GridAlpha',0.15, ...
       'MinorGridColor',[0.75 0.75 0.75], 'MinorGridAlpha',0.07);
grid(ax,'on'); grid(ax,'minor');
xlabel(ax,'X','Color',[1 1 1]);
ylabel(ax,'Y','Color',[1 1 1]);
zlabel(ax,'Z (vertical)','Color',[1 1 1]);

view(ax,[-35 18]);
xlim(ax,[-0.55 0.55]);
ylim(ax,[-0.40 0.40]);
zlim(ax,[G.zFloor-0.05 G.zCeil+0.12]);

%% -------------------- Elementos estáticos --------------------
drawPlateXZ(ax, [0 0 G.zCeil],  [1.05 0.70 0.02], 0.80);
drawPlateXZ(ax, [0 0 G.zFloor], [1.05 0.70 0.02], 0.25);
plot3(ax, [-0.55 0.55], [0 0], [G.zEq G.zEq], '--', 'LineWidth', 1.1, 'Color',[1 1 1 0.55]);

plot3(ax, G.xA1, G.yA1, G.zCeil, '.', 'MarkerSize', 20, 'Color',[1 1 1]);
plot3(ax, G.xA2, G.yA2, G.zCeil, '.', 'MarkerSize', 20, 'Color',[1 1 1]);

%% -------------------- Elementos dinámicos --------------------
zMass = G.zEq + S.x;
hBlk = drawBlockXYZ(ax, [0 0 zMass], [G.blkW G.blkD G.blkH], [0.25 0.45 0.95]);

springColor = [1.00 0.72 0.15];
hS1 = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2.2, 'Color', springColor);
hS2 = plot3(ax, nan, nan, nan, '-', 'LineWidth', 2.2, 'Color', springColor);

hTxt = text(ax, -0.53, 0.33, G.zCeil+0.08, '', 'FontName','Consolas', ...
            'FontSize',10,'Color',[1 1 1]);

%% -------------------- Controles --------------------
uicontrol(panel,'Style','text','String','Posición inicial x0 (m)', ...
    'Units','normalized','Position',[0.08 0.86 0.84 0.08], ...
    'BackgroundColor',panel.BackgroundColor,'ForegroundColor',[1 1 1], ...
    'HorizontalAlignment','left');

sldX0 = uicontrol(panel,'Style','slider','Min',-0.12,'Max',0.12,'Value',P.x0, ...
    'Units','normalized','Position',[0.08 0.80 0.84 0.06]);

valX0 = uicontrol(panel,'Style','text','String',sprintf('%+.3f',P.x0), ...
    'Units','normalized','Position',[0.08 0.74 0.84 0.05], ...
    'BackgroundColor',panel.BackgroundColor,'ForegroundColor',[1 1 1], ...
    'HorizontalAlignment','left');

uicontrol(panel,'Style','text','String','Amortiguamiento ζ', ...
    'Units','normalized','Position',[0.08 0.64 0.84 0.08], ...
    'BackgroundColor',panel.BackgroundColor,'ForegroundColor',[1 1 1], ...
    'HorizontalAlignment','left');

sldZeta = uicontrol(panel,'Style','slider','Min',0,'Max',0.35,'Value',P.zeta, ...
    'Units','normalized','Position',[0.08 0.58 0.84 0.06]);

valZeta = uicontrol(panel,'Style','text','String',sprintf('%0.2f',P.zeta), ...
    'Units','normalized','Position',[0.08 0.52 0.84 0.05], ...
    'BackgroundColor',panel.BackgroundColor,'ForegroundColor',[1 1 1], ...
    'HorizontalAlignment','left');

btnFree = uicontrol(panel,'Style','togglebutton','String','SOLTAR (libre)', ...
    'Units','normalized','Position',[0.08 0.38 0.84 0.08], 'Value',1);

btnRun = uicontrol(panel,'Style','togglebutton','String','PAUSA', ...
    'Units','normalized','Position',[0.08 0.28 0.84 0.08], 'Value',0);

btnReset = uicontrol(panel,'Style','pushbutton','String','RESET', ...
    'Units','normalized','Position',[0.08 0.18 0.84 0.08]);

uicontrol(panel,'Style','text','String','Tip: desactiva SOLTAR para arrastrar con slider.', ...
    'Units','normalized','Position',[0.08 0.06 0.84 0.10], ...
    'BackgroundColor',panel.BackgroundColor,'ForegroundColor',[0.85 0.85 0.85], ...
    'HorizontalAlignment','left');

%% -------------------- Guardar todo en guidata --------------------
D.fig = fig; D.ax = ax;
D.P = P; D.G = G; D.S = S;
D.hBlk = hBlk; D.hS1 = hS1; D.hS2 = hS2; D.hTxt = hTxt;
D.sldX0 = sldX0; D.sldZeta = sldZeta;
D.valX0 = valX0; D.valZeta = valZeta;
D.btnFree = btnFree; D.btnRun = btnRun; D.btnReset = btnReset;
guidata(fig, D);

% Callbacks (funciones locales al final)
set(sldX0,  'Callback', @onSliderX0);
set(sldZeta,'Callback', @onSliderZeta);
set(btnFree,'Callback', @onToggleFree);
set(btnRun, 'Callback', @onToggleRun);
set(btnReset,'Callback', @onReset);

% Primera actualización
updateScene(fig);

%% -------------------- Loop principal --------------------
nStep = ceil(P.tMax/P.dt);
for i = 1:nStep
    if ~ishandle(fig); break; end

    D = guidata(fig);
    P = D.P; G = D.G; S = D.S;

    if S.running && S.free
        a = (-P.c*S.v - P.k*S.x) / P.m;
        S.v = S.v + a*P.dt;
        S.x = S.x + S.v*P.dt;
    else
        S.x = get(D.sldX0,'Value');
        S.v = 0;
    end

    zMass = G.zEq + S.x;

    if zMass > G.zMax
        zMass = G.zMax;
        S.x = zMass - G.zEq;
        S.v = -0.25*S.v;
    elseif zMass < G.zMin
        zMass = G.zMin;
        S.x = zMass - G.zEq;
        S.v = -0.25*S.v;
    end

    D.S = S;
    guidata(fig, D);

    updateScene(fig);
    drawnow limitrate;
    pause(0.001);
end

%% ==================== Funciones locales ====================
function onSliderX0(src,~)
    fig = ancestor(src,'figure');
    D = guidata(fig);
    v = get(D.sldX0,'Value');
    set(D.valX0,'String',sprintf('%+.3f',v));
    if ~D.S.free
        D.S.x = v; D.S.v = 0;
        guidata(fig,D);
        updateScene(fig);
    end
end

function onSliderZeta(src,~)
    fig = ancestor(src,'figure');
    D = guidata(fig);
    D.P.zeta = get(D.sldZeta,'Value');
    D.P.c = 2*D.P.zeta*sqrt(D.P.m*D.P.k);
    set(D.valZeta,'String',sprintf('%0.2f',D.P.zeta));
    guidata(fig,D);
end

function onToggleFree(src,~)
    fig = ancestor(src,'figure');
    D = guidata(fig);
    D.S.free = logical(get(src,'Value'));
    if D.S.free
        set(src,'String','SOLTAR (libre)');
    else
        set(src,'String','ARRASTRAR (slider)');
        D.S.v = 0;
    end
    guidata(fig,D);
end

function onToggleRun(src,~)
    fig = ancestor(src,'figure');
    D = guidata(fig);
    isPressed = logical(get(src,'Value'));
    D.S.running = ~isPressed;
    if D.S.running
        set(src,'String','PAUSA','Value',0);
    else
        set(src,'String','REANUDAR','Value',1);
    end
    guidata(fig,D);
end

function onReset(src,~)
    fig = ancestor(src,'figure');
    D = guidata(fig);
    D.S.x = get(D.sldX0,'Value');
    D.S.v = 0;
    D.S.running = true;
    set(D.btnRun,'String','PAUSA','Value',0);
    guidata(fig,D);
    updateScene(fig);
end

function updateScene(fig)
    D = guidata(fig);
    P = D.P; G = D.G; S = D.S;

    zMass = G.zEq + S.x;
    updateBlockXYZ(D.hBlk, [0 0 zMass], [G.blkW G.blkD G.blkH]);

    zTopBlock = zMass + G.blkH/2;

    [x1,y1,z1] = springPoints(G.xA1,G.yA1,G.zCeil, G.xB1,G.yB1,zTopBlock, G.springRadius,G.nTurns,G.nPts);
    [x2,y2,z2] = springPoints(G.xA2,G.yA2,G.zCeil, G.xB2,G.yB2,zTopBlock, G.springRadius,G.nTurns,G.nPts);

    set(D.hS1,'XData',x1,'YData',y1,'ZData',z1);
    set(D.hS2,'XData',x2,'YData',y2,'ZData',z2);

    wn = sqrt(P.k/P.m);
    set(D.hTxt,'String',sprintf('x=%+0.3f m   v=%+0.3f m/s   wn=%0.2f rad/s   zeta=%0.2f', S.x, S.v, wn, P.zeta));
end

function h = drawPlateXZ(ax, center, dims, gray)
% Placa horizontal en plano X-Y con grosor en Z
W=dims(1); D=dims(2); T=dims(3);
cx=center(1); cy=center(2); cz=center(3);
x = cx + W/2*[-1  1  1 -1];
y = cy + D/2*[-1 -1  1  1];
zTop = cz + T/2;
zBot = cz - T/2;
patch(ax, x, y, zTop*ones(1,4), gray*[1 1 1], 'EdgeColor','none', 'FaceAlpha',1.0);
h = patch(ax, x, y, zBot*ones(1,4), gray*[1 1 1], 'EdgeColor','none', 'FaceAlpha',0.45);
end

function h = drawBlockXYZ(ax, center, dims, colorRGB)
% dims = [W (X), D (Y), H (Z)]
cx=center(1); cy=center(2); cz=center(3);
W=dims(1); D=dims(2); H=dims(3);
X = cx + (W/2)*[-1 1 1 -1 -1 1 1 -1];
Y = cy + (D/2)*[-1 -1 1 1 -1 -1 1 1];
Z = cz + (H/2)*[-1 -1 -1 -1 1 1 1 1];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
h = patch(ax,'Vertices',[X(:) Y(:) Z(:)],'Faces',F,...
    'FaceColor',colorRGB,'EdgeColor',[0 0 0],'LineWidth',0.5,'FaceAlpha',0.98);
end

function updateBlockXYZ(h, center, dims)
cx=center(1); cy=center(2); cz=center(3);
W=dims(1); D=dims(2); H=dims(3);
X = cx + (W/2)*[-1 1 1 -1 -1 1 1 -1];
Y = cy + (D/2)*[-1 -1 1 1 -1 -1 1 1];
Z = cz + (H/2)*[-1 -1 -1 -1 1 1 1 1];
set(h,'Vertices',[X(:) Y(:) Z(:)]);
end

function [x,y,z] = springPoints(xTop,yTop,zTop, xBot,yBot,zBot, radius,nTurns,nPts)
s = linspace(0,1,nPts);
xAxis = xTop + (xBot-xTop)*s;
yAxis = yTop + (yBot-yTop)*s;
zAxis = zTop + (zBot-zTop)*s;
theta = linspace(0, 2*pi*nTurns, nPts);
x = xAxis + radius*cos(theta);
y = yAxis + radius*sin(theta);
z = zAxis;
end