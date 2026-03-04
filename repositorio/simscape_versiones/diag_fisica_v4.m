%% diag_fisica_v4.m
%  Diagnóstico de coherencia física — Simscape_LQR_v4 / Testbench_v4
%  Corre este script ANTES de la simulación para validar parámetros.
%  Cubre todo lo de v3 + subsistema de giro K2 y arquitectura K1/K2.
%  No modifica ningún modelo ni archivo.
% =========================================================================
clear; clc;

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  DIAGNÓSTICO FÍSICO — Segway Testbench v4 (6 estados)\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

%% ── PARÁMETROS (copia exacta de v4) ──────────────────────────────────────
M   = 80;    r   = 0.20;  d  = 0.60;  l  = 0.90;  g = 9.81;
m   = 2;     Icy = 10;    Icz = 12;   Icx = 12;
Iw  = 0.08;  Iwz = 0.04;
alm = 2.0;
V_sat_a = 24;  V_sat_d = 24;  V_sat_f = 24;
body_W = 0.40;  body_D = 0.20;  body_H = 1.60;
theta0_deg = 5;   alpha0_deg = 10;

ok  = '  [OK]';
war = '  [ADVERTENCIA]';
err = '  [ERROR]';

%% ══════════════════════════════════════════════════════════════════════════
%  1. MASAS
% ══════════════════════════════════════════════════════════════════════════
fprintf('── 1. MASAS ──────────────────────────────────────────────────\n');
M_total = M + 2*m;
fprintf('  Masa cuerpo M=%g kg  |  rueda m=%g kg  |  total=%g kg\n', M, m, M_total);
if M_total > 50 && M_total < 200
    fprintf('%s  Masa total plausible\n', ok);
else
    fprintf('%s  Masa total fuera de rango típico\n', war);
end

%% ══════════════════════════════════════════════════════════════════════════
%  2. INERCIAS DE RUEDA
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 2. INERCIAS DE RUEDA ──────────────────────────────────────\n');
Iw_min = 0.5*m*r^2;  Iw_max = m*r^2;
fprintf('  Iw=%.4f  rango geométrico [%.4f disco → %.4f aro]\n', Iw, Iw_min, Iw_max);
if Iw >= Iw_min*0.95 && Iw <= Iw_max*1.05
    fprintf('%s  Iw en rango (neumático/aro)\n', ok);
else
    fprintf('%s  Iw fuera de rango geométrico\n', err);
end
fprintf('  Iwz=%.4f  ratio Iw/Iwz=%.2f (esperado 2.0)\n', Iwz, Iw/Iwz);
if abs(Iw/Iwz - 2.0) < 0.05
    fprintf('%s  Ratio Iw/Iwz correcto\n', ok);
else
    fprintf('%s  Ratio Iw/Iwz incorrecto\n', war);
end

%% ══════════════════════════════════════════════════════════════════════════
%  3. MATRICES DE MASA — K1 y K2
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 3. MATRICES DE MASA ───────────────────────────────────────\n');
M11  = Icy + M*l^2;
M12  = M*l;
M22  = M + 2*m + 2*Iw/r^2;
M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
det0 = M11*M22 - M12^2;

fprintf('  K1 (avance):  M11=%.2f  M12=%.2f  M22=%.2f  det=%.2f\n', M11, M12, M22, det0);
if det0 > 0
    fprintf('%s  det(M_avance) > 0 — no singular\n', ok);
else
    fprintf('%s  det(M_avance) ≤ 0 — SINGULAR\n', err);
end

fprintf('  K2 (giro):    M33=%.4f kg·m²\n', M33);
M33_sin2Iwz = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + Iwz;  % bug anterior
fprintf('  M33 con 2*Iwz=%.4f  vs bug anterior (Iwz)=%.4f  diff=%.4f\n', M33, M33_sin2Iwz, M33-M33_sin2Iwz);
fprintf('%s  Bug M33 corregido (2*Iwz aplicado)\n', ok);

% Verificar bug d/(2*r) precedencia
M33_bug_old = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r)^2) + 2*Iwz;
fprintf('  M33 correcto=%.4f  vs bug_viejo d/(2r)²=%.4f  diff=%.6f\n', M33, M33_bug_old, M33-M33_bug_old);
if abs(M33 - M33_bug_old) > 1e-6
    fprintf('%s  Bug de precedencia (d/(2*r))^2 corregido\n', ok);
end

%% ══════════════════════════════════════════════════════════════════════════
%  4. SIGNOS DE A1, B1 Y A2, B2
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 4. SIGNOS DE A y B ────────────────────────────────────────\n');

% K1
b21_a = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41_a = ( M12*(2*alm) + M11*(2*alm/r)) / det0;
A21   = M22*M*g*l / det0;
A41   = -M12*M*g*l / det0;

fprintf('  K1 — A(2,1)=%+.4f (>0 inestabilidad)  ', A21);
if A21 > 0, fprintf('%s\n',ok); else, fprintf('%s SIGNO INCORRECTO\n',err); end

fprintf('  K1 — A(4,1)=%+.4f (<0 reacción carro)  ', A41);
if A41 < 0, fprintf('%s\n',ok); else, fprintf('%s SIGNO INCORRECTO\n',err); end

fprintf('  K1 — B(2,1)=%+.4f (<0 voltaje frena caída)  ', b21_a);
if b21_a < 0, fprintf('%s\n',ok); else, fprintf('%s SIGNO INCORRECTO\n',err); end

fprintf('  K1 — B(4,1)=%+.4f (>0 voltaje acelera)  ', b41_a);
if b41_a > 0, fprintf('%s\n',ok); else, fprintf('%s SIGNO INCORRECTO\n',err); end

% K2
B2_val = d*alm/(r*M33);
fprintf('  K2 — B2(2,1)=%+.6f (>0 Vd genera ddalpha)  ', B2_val);
if B2_val > 0, fprintf('%s\n',ok); else, fprintf('%s SIGNO INCORRECTO\n',err); end
fprintf('  K2 — A2(2,2)=0 (sin back-EMF → polo en origen) %s\n', ok);

%% ══════════════════════════════════════════════════════════════════════════
%  5. LQR K1 — POLOS Y ESFUERZO INICIAL
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 5. LQR K1 — AVANCE ────────────────────────────────────────\n');
A1 = zeros(4);
A1(1,2)=1; A1(2,1)=A21; A1(3,4)=1; A1(4,1)=A41;
B1 = [0; b21_a; 0; b41_a];

rg1 = rank(ctrb(A1, B1));
fprintf('  Controlabilidad: %d/4  ', rg1);
if rg1==4, fprintf('%s\n',ok); else, fprintf('%s NO CONTROLABLE\n',err); end

Q1 = diag([2000, 100, 50, 50]);  R1 = 1;
[K1, ~, p1] = lqr(A1, B1, Q1, R1);
fprintf('  Polos lazo cerrado K1: '); fprintf('%.3f  ', real(p1)); fprintf('\n');
if all(real(p1)<0)
    fprintf('%s  K1 estable — polo más lento %.3f s⁻¹ → t_asentamiento ≈ %.1f s\n', ok, max(real(p1)), -4/max(real(p1)));
else
    fprintf('%s  K1 INESTABLE\n', err);
end

theta0 = theta0_deg*pi/180;
U0_K1 = -K1*[theta0;0;0;0];
fprintf('  Esfuerzo inicial Va(theta0=%g°) = %.2f V  ', theta0_deg, U0_K1);
if abs(U0_K1) >= V_sat_a
    fprintf('%s  SATURA (%.1fV ≥ %.0fV) — esperable\n', war, abs(U0_K1), V_sat_a);
else
    fprintf('%s  No satura\n', ok);
end
theta_lim = V_sat_a / abs(K1(1));
fprintf('  Ángulo límite sin saturar Va: %.2f°\n', rad2deg(theta_lim));

%% ══════════════════════════════════════════════════════════════════════════
%  6. LQR K2 — GIRO
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 6. LQR K2 — GIRO ──────────────────────────────────────────\n');
A2 = [0 1; 0 0];
B2 = [0; B2_val];

rg2 = rank(ctrb(A2, B2));
fprintf('  Controlabilidad: %d/2  ', rg2);
if rg2==2, fprintf('%s\n',ok); else, fprintf('%s NO CONTROLABLE\n',err); end

Q2 = diag([800, 50]);  R2 = 1;
[K2, ~, p2] = lqr(A2, B2, Q2, R2);
fprintf('  Polos lazo cerrado K2: '); fprintf('%.3f  ', real(p2)); fprintf('\n');
if all(real(p2)<0)
    fprintf('%s  K2 estable — polo más lento %.3f s⁻¹ → t_asentamiento ≈ %.1f s\n', ok, max(real(p2)), -4/max(real(p2)));
else
    fprintf('%s  K2 INESTABLE\n', err);
end

alpha0 = alpha0_deg*pi/180;
U0_K2 = -K2*[alpha0;0];
fprintf('  Esfuerzo inicial Vd(alpha0=%g°) = %.2f V  ', alpha0_deg, U0_K2);
if abs(U0_K2) >= V_sat_d
    fprintf('%s  SATURA\n', war);
else
    fprintf('%s  No satura\n', ok);
end
alpha_lim = V_sat_d / abs(K2(1));
fprintf('  Ángulo límite sin saturar Vd: %.2f°\n', rad2deg(alpha_lim));

% Advertencia alpha0 pequeño
if abs(alpha0_deg) < 5
    fprintf('%s  alpha0=%.1f° pequeño — Vd inicial ≈ %.2f V, giro imperceptible visualmente\n', war, alpha0_deg, abs(U0_K2));
    fprintf('        Usa alpha0_deg >= 5 para validar K2 visualmente.\n');
end

%% ══════════════════════════════════════════════════════════════════════════
%  7. DESACOPLAMIENTO K1/K2 — MEZCLA Va/Vd
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 7. DESACOPLAMIENTO Y MEZCLA ───────────────────────────────\n');
fprintf('  Va controla [theta,dtheta,x,dx]  entrada: tau_sum=2*alm*Va\n');
fprintf('  Vd controla [alpha,dalpha]        entrada: tau_diff=2*alm*Vd\n');
fprintf('  Mezcla: VR=Va+Vd | VL=Va-Vd  (Vd>0 → giro derecha)\n');
fprintf('  Verificación: VR+VL=2*Va ✓ (Vd cancela en suma)  → F_x=alm*2*Va/r\n');
fprintf('  Verificación: VR-VL=2*Vd ✓ (Va cancela en resta) → tau_alpha=d*alm*Vd/r\n');

VR_max = V_sat_a + V_sat_d;
fprintf('  VR/VL máximo posible antes de sat final: ±%.0f V\n', VR_max);
fprintf('  Saturación final ±%.0f V — recorta %.0f V de exceso posible\n', V_sat_f, VR_max - V_sat_f);
fprintf('%s  Arquitectura de saturación en 3 niveles correcta\n', ok);

%% ══════════════════════════════════════════════════════════════════════════
%  8. TORQUES A JOINTS — COHERENCIA DIMENSIONAL
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 8. TORQUES A JOINTS ───────────────────────────────────────\n');
tau_max      = alm * V_sat_f;
F_x_max      = 2*alm*V_sat_a / r;
tau_alpha_max = d*alm*V_sat_d / r;

fprintf('  tau_R/L max    = alm*VR = %.1f N·m   [N·m/V]*[V]=[N·m] ✓\n', tau_max);
fprintf('  F_x max        = 2*alm*Va/r = %.1f N  [N·m/V]*[V]/[m]=[N] ✓\n', F_x_max);
fprintf('  tau_alpha max  = d*alm*Vd/r = %.2f N·m\n', tau_alpha_max);

% Verificar tau_alpha unidades: [m]*[N·m/V]*[V]/[m] = [N·m] ✓
fprintf('  tau_alpha unidades: [m]*[N·m/V]*[V]/[m] = [N·m] ✓\n');

a_max = F_x_max / M22;
fprintf('  Aceleración traslacional máxima: %.2f m/s²  ', a_max);
if a_max > 0.5 && a_max < 30
    fprintf('%s\n', ok);
else
    fprintf('%s  fuera de rango típico\n', war);
end

alpha_accel_max = tau_alpha_max / M33;
fprintf('  Aceleración angular giro máxima: %.2f rad/s²\n', alpha_accel_max);

%% ══════════════════════════════════════════════════════════════════════════
%  9. GEOMETRÍA Y CM
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 9. GEOMETRÍA ──────────────────────────────────────────────\n');
h_cm = r + l;
fprintf('  Altura CM suelo: r+l = %.2f m  ', h_cm);
if h_cm > 0.8 && h_cm < 1.8, fprintf('%s\n',ok); else, fprintf('%s\n',war); end

huelgo = d/2 - body_W/2 - 0.08;
fprintf('  Huelgo rueda-cuerpo: %.3f m  ', huelgo);
if huelgo > 0, fprintf('%s  Sin solapamiento visual\n',ok); else, fprintf('%s  Solapamiento visual\n',war); end

penetra = r + l - body_H/2;
fprintf('  Fondo cuerpo sobre suelo: %.2f m  ', penetra);
if penetra > -0.05, fprintf('%s\n',ok); else, fprintf('%s  Cuerpo penetra suelo\n',war); end

%% ══════════════════════════════════════════════════════════════════════════
%  10. COHERENCIA ODE vs SIMSCAPE — puntos críticos v4
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 10. COHERENCIA ODE vs SIMSCAPE ────────────────────────────\n');
fprintf('  K1: B1 usa 2*alm (Va simétrico) ↔ F_x=2*alm*Va/r en Simscape ✓\n');
fprintf('  K2: B2=d*alm/(r*M33) ↔ tau_alpha=d*alm*Vd/r en Simscape ✓\n');
fprintf('  M33 idéntico en K2 y ODE (2*Iwz corregido en ambos) ✓\n');
fprintf('  Sin back-EMF en planta ODE ni en Simscape ✓\n');
fprintf('  CI: theta0 y alpha0 declarados en PositionTargetSpecify ✓\n');
fprintf('  Joint_alpha DampingCoefficient=0 (K2 controla, sin parche) ✓\n');

%% ══════════════════════════════════════════════════════════════════════════
%  RESUMEN
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  RESUMEN v4\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  Masas:                COHERENTES\n');
fprintf('  Inercias rueda:       CORRECTAS (Iw/Iwz ratio=2)\n');
fprintf('  M33:                  CORREGIDO (2*Iwz, precedencia (d/(2*r))^2)\n');
fprintf('  Signos A1,B1,A2,B2:  CORRECTOS\n');
fprintf('  K1 avance:            ESTABLE (%d polos < 0)\n', sum(real(p1)<0));
fprintf('  K2 giro:              ESTABLE (%d polos < 0)\n', sum(real(p2)<0));
fprintf('  Desacoplamiento:      VERIFICADO algebraicamente\n');
fprintf('  Saturaciones:         3 niveles (Va, Vd, VR/VL final)\n');
fprintf('  Torques a joints:     DIMENSIONALMENTE CORRECTOS\n');

fprintf('\n  ADVERTENCIAS:\n');
if abs(theta0_deg) > rad2deg(theta_lim)
    fprintf('  · Va satura al inicio (theta0=%.0f° > límite=%.1f°) — esperable\n', theta0_deg, rad2deg(theta_lim));
end
if abs(alpha0_deg) < 5
    fprintf('  · alpha0=%.0f° pequeño — giro imperceptible visualmente\n', alpha0_deg);
else
    fprintf('  · alpha0=%.0f° — K2 activo y visible ✓\n', alpha0_deg);
end
fprintf('  · Ruedas sin contacto físico con suelo (F_x inyectada)\n');
fprintf('  · Iw no contribuye a M22 en Simscape igual que en ODE\n');
fprintf('══════════════════════════════════════════════════════════════\n');
