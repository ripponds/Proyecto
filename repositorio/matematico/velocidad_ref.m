%% velocidad_ref.m
%  Seguimiento de velocidad lineal — estado aumentado
%  X = [theta, dtheta, x, dx, alpha, dalpha, integral(dx-v_ref)]
%  K1 (1x5): avance aumentado  |  K2 (1x2): giro (igual que v4.1)
%  Escenario: robot quieto y vertical, escalon de v_ref a t=3s
%  Sin back-EMF, sin Simscape — ODE puro
% =========================================================================
clear; clc; close all;

%% 1. PARAMETROS (identicos a v4.1)
M=80; r=0.20; d=0.60; l=0.90; g=9.81;
m=2; Icy=10; Icz=12; Iw=0.08; Iwz=0.04; alm=2.0;
V_sat_a=24; V_sat_d=24; V_sat_f=24;
t_sim=20;
v_ref_val = 1.0;   % velocidad crucero deseada [m/s]
t_vref    = 3.0;   % instante del escalon [s]

%% 2. MATRICES DE MASA
M11 = Icy + M*l^2;
M12 = M*l;
M22 = M + 2*m + 2*Iw/r^2;
M33 = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
det0 = M11*M22 - M12^2;

%% 3. LQR K1 AUMENTADO — [theta, dtheta, dx, integral_ev]
%  Se elimina x: para velocidad no importa la posicion, y x e integral_ev
%  son ambos integrales de dx -> dependencia lineal -> sistema no controlable
b21 = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41 = ( M12*(2*alm) + M11*(2*alm/r)) / det0;

% Sistema reducido [theta, dtheta, dx]
A1r = [0,               1, 0;
       M22*M*g*l/det0,  0, 0;
      -M12*M*g*l/det0,  0, 0];
B1r = [0; b21; b41];

% Aumentado con integral_ev: d(int)/dt = dx (estado 3)
A1a = [A1r, zeros(3,1); 0 0 1 0];
B1a = [B1r; 0];

rango = rank(ctrb(A1a, B1a));
fprintf('[K1aug] Controlabilidad: %d/4 %s\n', rango, iif(rango==4,'OK','REVISAR'));

q_int = 50 / (1.5 * 5);
Q1a = diag([2000, 100, 50, q_int]);
R1  = 1;
[K1a, ~, eigs1] = lqr(A1a, B1a, Q1a, R1);
fprintf('[K1aug] K1 = [%.3f  %.3f  %.3f  %.3f]\n', K1a);
fprintf('[K1aug] Polos: '); fprintf('%.3f  ', real(eigs1)); fprintf('\n');

%% 4. LQR K2 GIRO (identico a v4.1)
A2=[0 1;0 0]; B2=[0; d*alm/(r*M33)];
Q2=diag([800,50]); R2=1;
[K2,~,eigs2] = lqr(A2,B2,Q2,R2);
fprintf('[K2]    K2 = [%.3f  %.3f]\n', K2);
fprintf('[K2]    Polos: '); fprintf('%.3f  ', real(eigs2)); fprintf('\n');

%% 5. ODE — 7 estados: [theta,dtheta,x,dx,alpha,dalpha,integral_ev]
X0 = zeros(7,1);   % quieto y vertical
ode_fn = @(t,X) ode_vel(t, X, K1a, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, v_ref_val, t_vref);

[t, X] = ode45(ode_fn, [0 t_sim], X0, odeset('RelTol',1e-6));

% Reconstruir Va, Vd
n = length(t);
Va = zeros(n,1); Vd = zeros(n,1);
for i = 1:n
    Xe1 = [X(i,1); X(i,2); X(i,4); X(i,7)];  % [theta,dtheta,dx,int_ev]
    Xe2 = [X(i,5); X(i,6)];
    Va(i) = max(min(-K1a*Xe1, V_sat_a), -V_sat_a);
    Vd(i) = max(min(-K2 *Xe2, V_sat_d), -V_sat_d);
end
v_ref_t = double(t >= t_vref) * v_ref_val;

fprintf('[ODE] dx_final=%.3f m/s (ref=%.1f)  theta_final=%.3f deg\n', ...
    X(end,4), v_ref_val, rad2deg(X(end,1)));

%% 6. GRAFICAS — una figura, 4 subplots
figure('Name','Seguimiento de velocidad — estado aumentado','Position',[50 50 1000 900]);

subplot(4,1,1); hold on; grid on;
plot(t, rad2deg(X(:,1)),'r','LineWidth',2);
yline(0,'--k'); xlabel('t [s]'); ylabel('\theta [deg]');
title('\theta(t) — el robot se inclina para acelerar'); xlim([0 t_sim]);

subplot(4,1,2); hold on; grid on;
plot(t, X(:,4),'b','LineWidth',2,'DisplayName','dx simulado');
plot(t, v_ref_t,'--k','LineWidth',1.5,'DisplayName','v_{ref}');
xlabel('t [s]'); ylabel('dx [m/s]');
title('Velocidad lineal vs referencia'); legend('Location','best'); xlim([0 t_sim]);

subplot(4,1,3); hold on; grid on;
plot(t, Va,'g','LineWidth',2);
yline( V_sat_a,'--k'); yline(-V_sat_a,'--k');
xlabel('t [s]'); ylabel('Va [V]');
title('Va — entrada de avance'); xlim([0 t_sim]);

subplot(4,1,4); hold on; grid on;
plot(t, X(:,7),'m','LineWidth',2);
yline(0,'--k'); xlabel('t [s]'); ylabel('\int e_v dt');
title('Estado integral — acumulacion del error de velocidad'); xlim([0 t_sim]);

%% FUNCIONES LOCALES
function s = iif(c,a,b); if c, s=a; else, s=b; end; end

function dX = ode_vel(t, X, K1a, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, v_ref_val, t_vref)

    vr = iif(t >= t_vref, v_ref_val, 0);

    Xe1 = [X(1); X(2); X(4); X(7)];  % [theta,dtheta,dx,int_ev] sin x
    Va  = max(min(-K1a*Xe1, V_sat_a), -V_sat_a);

    Xe2 = [X(5); X(6)];
    Vd  = max(min(-K2*Xe2, V_sat_d), -V_sat_d);

    VR = max(min(Va+Vd, V_sat_f), -V_sat_f);
    VL = max(min(Va-Vd, V_sat_f), -V_sat_f);
    tau_R = alm*VR; tau_L = alm*VL;

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
    d_int   = X(4) - vr;   % error de velocidad

    dX = [dtheta; ddtheta; X(4); ddx; X(6); ddalpha; d_int];
end
