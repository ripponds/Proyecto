%% run_comparacion_LQR.m
%  Comparación LQR: ODE Lineal vs ODE No Lineal (Kane)
%  Mismo K, mismas CI, mismos parámetros
%  Figura: theta, x, alpha

clear; clc; close all;

%% ── 1. PARÁMETROS ────────────────────────────────────────────────────────
M   = 80;    r   = 0.20;  d   = 0.60;  l   = 0.90;  g   = 9.81;
m   = 2;     Icy = 10;    Icz = 12;    Icx = 12;
Iw  = 0.08;  Iwz = 0.04;
alm = 2.0;   bem = 1.5;

theta0_deg = 5;
theta0     = theta0_deg * pi/180;
t_sim      = 10;

%% ── 2. LINEALIZACIÓN ─────────────────────────────────────────────────────
M11  = Icy + M*l^2;
M12  = M*l;
M22  = M + 2*m + 2*Iw/r^2;
M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
det0 = M11*M22 - M12^2;

dF1_dth=M*g*l; dF1_doth=-2*bem; dF1_dox=2*bem/r; dF1_VR=-alm; dF1_VL=-alm;
dF2_doth=2*bem/r; dF2_dox=-2*bem/r^2; dF2_VR=alm/r; dF2_VL=alm/r;
dF3_doal=-bem*d^2/(2*r^2); dF3_VR=alm*d/(2*r); dF3_VL=-alm*d/(2*r);

A = zeros(6);
A(1,2)=1; A(3,4)=1; A(5,6)=1;
A(2,1)=(M22*dF1_dth)/det0;
A(2,2)=(M22*dF1_doth - M12*dF2_doth)/det0;
A(2,4)=(M22*dF1_dox  - M12*dF2_dox)/det0;
A(4,1)=(-M12*dF1_dth)/det0;
A(4,2)=(-M12*dF1_doth + M11*dF2_doth)/det0;
A(4,4)=(-M12*dF1_dox  + M11*dF2_dox)/det0;
A(6,6)=dF3_doal/M33;

B = zeros(6,2);
B(2,1)=(M22*dF1_VR - M12*dF2_VR)/det0; B(2,2)=(M22*dF1_VL - M12*dF2_VL)/det0;
B(4,1)=(-M12*dF1_VR + M11*dF2_VR)/det0; B(4,2)=(-M12*dF1_VL + M11*dF2_VL)/det0;
B(6,1)=dF3_VR/M33; B(6,2)=dF3_VL/M33;

%% ── 3. LQR ───────────────────────────────────────────────────────────────
Q = diag([2000, 100, 50, 50, 800, 50]);
R = diag([1, 1]);
[K, ~, cl_poles] = lqr(A, B, Q, R);
fprintf('Polos en lazo cerrado:\n'); disp(cl_poles.')
fprintf('K (fila 1 = VR): [%.3f  %.3f  %.3f  %.3f  %.3f  %.3f]\n', K(1,:));
fprintf('K (fila 2 = VL): [%.3f  %.3f  %.3f  %.3f  %.3f  %.3f]\n', K(2,:));

V_sat = 12;   % [V]
X0    = [theta0; 0; 0; 0; 0; 0];

%% ── 4. ODE NO LINEAL + LQR (Kane) ───────────────────────────────────────
ode_nl = @(t,X) ode_nolineal(t, X, K, V_sat, M, m, r, d, l, g, ...
                               Icy, Icz, Icx, Iw, Iwz, alm, bem);
[t_nl, X_nl] = ode45(ode_nl, [0 t_sim], X0);
fprintf('ODE No Lineal: theta_final=%.3f°  x_final=%.3f m\n', ...
    rad2deg(X_nl(end,1)), X_nl(end,3));

%% ── 5. ODE LINEAL + LQR (espacio de estados) ────────────────────────────
Acl = A - B*K;
ode_lin = @(t,X) Acl * X;
[t_lin, X_lin] = ode45(ode_lin, [0 t_sim], X0);
fprintf('ODE Lineal:    theta_final=%.3f°  x_final=%.3f m\n', ...
    rad2deg(X_lin(end,1)), X_lin(end,3));

%% ── 6. VOLTAJES APLICADOS (no lineal) ───────────────────────────────────
U_nl = zeros(numel(t_nl), 2);
for i = 1:numel(t_nl)
    u = -K * X_nl(i,:)';
    U_nl(i,:) = max(min(u', V_sat), -V_sat);
end

%% ── 7. FIGURA ────────────────────────────────────────────────────────────
figure('Name','LQR: Lineal vs No Lineal','Position',[80 80 1200 700]);
col_nl  = [0.00 0.45 0.74];   % azul
col_lin = [0.85 0.33 0.10];   % rojo-naranja

% ── Subplot 1: theta ──
subplot(2,3,1);
plot(t_nl,  rad2deg(X_nl(:,1)),  'Color',col_nl,  'LineWidth',2,   'DisplayName','No Lineal');
hold on;
plot(t_lin, rad2deg(X_lin(:,1)), 'Color',col_lin, 'LineWidth',1.6, 'LineStyle','--','DisplayName','Lineal');
yline(0,'--k'); grid on;
xlabel('t [s]'); ylabel('\theta [°]');
title('Cabeceo \theta');
legend('Location','best');

% ── Subplot 2: dtheta ──
subplot(2,3,2);
plot(t_nl,  rad2deg(X_nl(:,2)),  'Color',col_nl,  'LineWidth',2);
hold on;
plot(t_lin, rad2deg(X_lin(:,2)), 'Color',col_lin, 'LineWidth',1.6, 'LineStyle','--');
yline(0,'--k'); grid on;
xlabel('t [s]'); ylabel('d\theta/dt [°/s]');
title('Velocidad cabeceo');

% ── Subplot 3: x ──
subplot(2,3,3);
plot(t_nl,  X_nl(:,3),  'Color',col_nl,  'LineWidth',2);
hold on;
plot(t_lin, X_lin(:,3), 'Color',col_lin, 'LineWidth',1.6, 'LineStyle','--');
yline(0,'--k'); grid on;
xlabel('t [s]'); ylabel('x [m]');
title('Avance x');

% ── Subplot 4: dx ──
subplot(2,3,4);
plot(t_nl,  X_nl(:,4),  'Color',col_nl,  'LineWidth',2);
hold on;
plot(t_lin, X_lin(:,4), 'Color',col_lin, 'LineWidth',1.6, 'LineStyle','--');
yline(0,'--k'); grid on;
xlabel('t [s]'); ylabel('dx/dt [m/s]');
title('Velocidad avance');

% ── Subplot 5: alpha ──
subplot(2,3,5);
plot(t_nl,  rad2deg(X_nl(:,5)),  'Color',col_nl,  'LineWidth',2);
hold on;
plot(t_lin, rad2deg(X_lin(:,5)), 'Color',col_lin, 'LineWidth',1.6, 'LineStyle','--');
yline(0,'--k'); grid on;
xlabel('t [s]'); ylabel('\alpha [°]');
title('Giro \alpha');

% ── Subplot 6: voltajes V_R y V_L (no lineal) ──
subplot(2,3,6);
plot(t_nl, U_nl(:,1), 'b-',  'LineWidth',1.8, 'DisplayName','V_R');
hold on;
plot(t_nl, U_nl(:,2), 'r--', 'LineWidth',1.4, 'DisplayName','V_L');
yline( V_sat,'--k','12 V');
yline(-V_sat,'--k','-12 V');
grid on;
xlabel('t [s]'); ylabel('V [V]');
title('Voltajes de control');
legend('Location','best');

sgtitle(sprintf('Controlador LQR — \\theta_0 = %d°  |  Q=diag([2000,100,50,50,800,50])  R=I', ...
    theta0_deg), 'FontSize', 11);

fprintf('\n=== COMPLETADO ===\n');

%% ── ODE NO LINEAL CON LQR ────────────────────────────────────────────────
function dX = ode_nolineal(~, X, K, V_sat, M, m, r, d, l, g, Icy, Icz, Icx, Iw, Iwz, alm, bem)
    U = -K * X;
    U = max(min(U, V_sat), -V_sat);

    theta  = X(1);  dtheta = X(2);
    dx     = X(4);  dalpha = X(6);
    VR = U(1);  VL = U(2);

    M11 = Icy + M*l^2;
    M12 = M*l*cos(theta);
    M22 = M + 2*m + 2*Iw/r^2;
    M33 = Icx*sin(theta)^2 + Icz*cos(theta)^2 + ...
          2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    det_ca = M11*M22 - M12^2;

    tau_R = alm*VR - bem*(dx/r + d*dalpha/(2*r) - dtheta);
    tau_L = alm*VL - bem*(dx/r - d*dalpha/(2*r) - dtheta);

    F1 = M*g*l*sin(theta) - (tau_R + tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R + tau_L)/r;
    F3 = d*(tau_R - tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / det_ca;
    ddx     = (-M12*F1 + M11*F2) / det_ca;
    ddalpha = F3 / M33;

    dX = [dtheta; ddtheta; dx; ddx; dalpha; ddalpha];
end
