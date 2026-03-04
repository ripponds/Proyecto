% =========================================================================
%  twip_params.m — Parámetros del TWIP + LQR
%  CORRER ANTES de build_sim.m y antes de cada simulación
%  Todo lo que quieras variar entre experimentos está aquí.
% =========================================================================
clear all
close all
if bdIsLoaded('TWIP_NL')
    close_system('TWIP_NL', 0);
end

%% Parámetros físicos
twip_p.mB = 45;    twip_p.mW = 2;    twip_p.r  = 0.23;
twip_p.dw = 0.60;  twip_p.l  = 0.35; twip_p.g  = 9.81;
twip_p.I1 = 1.9;   twip_p.I2 = 2.1;  twip_p.I3 = 1.6;
twip_p.J  = 0.02;  twip_p.K  = 0.04;

%% Bache — modificar para cada experimento
twip_p.bache_pos = 0.25;   % posición [m]
twip_p.bache_h   = 0.04;   % altura   [m]
twip_p.bache_w   = 0.15;   % ancho    [m]

%% Linealización numérica en equilibrio (th=0, todas velocidades=0, u=0)
%  Garantiza que A,B sean consistentes con twip_plant_fcn.m
p_phys_lin = [twip_p.mB, twip_p.mW, twip_p.r,  twip_p.dw, twip_p.l, twip_p.g, ...
              twip_p.I1,  twip_p.I2,  twip_p.I3, twip_p.J,  twip_p.K]';
x_eq = zeros(6,1);
u_eq = zeros(2,1);
d_eq = zeros(2,1);
eps_fd = 1e-5;

A = zeros(6,6);
for i = 1:6
    xp = x_eq; xp(i) = xp(i) + eps_fd;
    xm = x_eq; xm(i) = xm(i) - eps_fd;
    A(:,i) = (twip_plant_fcn(xp,u_eq,d_eq,p_phys_lin) - ...
              twip_plant_fcn(xm,u_eq,d_eq,p_phys_lin)) / (2*eps_fd);
end

B = zeros(6,2);
for i = 1:2
    up = u_eq; up(i) = up(i) + eps_fd;
    um = u_eq; um(i) = um(i) - eps_fd;
    B(:,i) = (twip_plant_fcn(x_eq,up,d_eq,p_phys_lin) - ...
              twip_plant_fcn(x_eq,um,d_eq,p_phys_lin)) / (2*eps_fd);
end

%% LQR — solo equilibrio (regulación en origen)
%  Ajustar Q para comparar controladores

%       x     theta   psi   x_dot  theta_dot  psi_dot
Q = diag([50,  8000,   500,    1,    2000,       500]);
R = eye(2) * 0.5;

twip_p.K_lqr = lqr(A, B, Q, R);
fprintf('K_lqr max = %.2f\n', max(abs(twip_p.K_lqr(:))));

%% Vectores planos para Constant blocks de Simulink
%  (los MATLAB Function blocks no aceptan structs, sí vectores numéricos)
%  Orden fijo — no cambiar sin actualizar twip_plant_fcn.m
p_phys  = [twip_p.mB, twip_p.mW, twip_p.r,  twip_p.dw, twip_p.l,  twip_p.g, ...
           twip_p.I1,  twip_p.I2,  twip_p.I3, twip_p.J,  twip_p.K]';   % [11x1]

p_bache = [twip_p.bache_pos; twip_p.bache_h; twip_p.bache_w];            % [3x1]

K_lqr   = twip_p.K_lqr;   % [2x6] — Constant block lo pasa directo

%% Condiciones iniciales
X0 = [0; 0.05; 0; 0.0; 0; 0];  % [x; theta; psi; xd; thd; psd]
% x_dot=1.2 m/s causaba inestabilidad (saturación del actuador antes de recuperar)

%% Perturbaciones externas (0 = sin perturbación)
d1_ext = 0;   % torque en cabeceo [N·m]
d2_ext = 0;   % torque en guiñada [N·m]

fprintf('Parámetros listos. Corre build_sim.m\n');