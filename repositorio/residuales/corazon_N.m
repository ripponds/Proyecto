% Corazon con las letras CN adentro
% Para mi novia <3

clear; clc; close all;

figure('Color', [0.05 0.05 0.1], 'Position', [200 100 700 650]);
ax = axes('Color', [0.05 0.05 0.1]);
hold on; axis equal; axis off;

%% --- Corazon (parametrica) ---
t = linspace(0, 2*pi, 1000);
x =  16 * sin(t).^3;
y =  13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);

% Relleno con gradiente naranja: varias capas
n_capas = 60;
for k = n_capas:-1:1
    scale = k / n_capas;
    r = 0.85 + 0.15*(1 - scale);   % rojo alto
    g = 0.35 * scale;               % verde medio (hace naranja)
    b = 0.0;
    fill(x*scale, y*scale, [r g b], 'EdgeColor', 'none');
end

% Borde brillante naranja
plot(x, y, 'Color', [1 0.55 0.1], 'LineWidth', 2.5);

%% --- Letras CN adentro ---
lw = 4;        % grosor de linea
col = [1 1 1]; % blanco

% ----- C -----
cx = -4.5; cy = 0; ch = 5; cw = 3.5;
ang = linspace(pi*0.25, 2*pi - pi*0.25, 200);
xc = cx + cw * cos(ang);
yc = cy + ch * sin(ang);
plot(xc, yc, 'Color', col, 'LineWidth', lw);

% ----- N -----
nx = 1.5; ny = -5; nh = 10; nw = 4.5;  % origen abajo izquierda
% Trazo izquierdo vertical
plot([nx, nx],        [ny, ny+nh], 'Color', col, 'LineWidth', lw);
% Diagonal
plot([nx, nx+nw],     [ny+nh, ny], 'Color', col, 'LineWidth', lw);
% Trazo derecho vertical
plot([nx+nw, nx+nw],  [ny, ny+nh], 'Color', col, 'LineWidth', lw);

%% --- Texto romantico ---
text(0, -16.5, 'Para ti, siempre \heartsuit', ...
    'HorizontalAlignment', 'center', ...
    'FontSize', 14, ...
    'FontWeight', 'bold', ...
    'Color', [1 0.75 0.4], ...
    'FontName', 'Arial');

xlim([-20 20]);
ylim([-20 16]);

title('', 'Color', 'w');
set(gcf, 'Name', 'Corazon para mi novia');
