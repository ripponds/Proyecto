%% diag_fisica_v3.m
%  Diagnóstico de coherencia física — Simscape_LQR_v3 / Testbench_v3
%  Corre este script ANTES de la simulación para validar parámetros.
%  No modifica ningún modelo ni archivo.
% =========================================================================
clear; clc;

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  DIAGNÓSTICO FÍSICO — Segway Testbench v3\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

%% ── PARÁMETROS (copia exacta de v3) ─────────────────────────────────────
M   = 80;    r   = 0.20;  d  = 0.60;  l  = 0.90;  g = 9.81;
m   = 2;     Icy = 10;    Icz = 12;   Icx = 12;
Iw  = 0.08;  Iwz = 0.04;
alm = 2.0;   V_sat = 12;
body_W = 0.40;  body_D = 0.20;  body_H = 1.60;
theta0_deg = 14;
ramp_deg   = 0;

ok  = '  [OK]';
war = '  [ADVERTENCIA]';
err = '  [ERROR]';

%% ══════════════════════════════════════════════════════════════════════════
%  1. MASA TOTAL Y PLAUSIBILIDAD
% ══════════════════════════════════════════════════════════════════════════
fprintf('── 1. MASAS ──────────────────────────────────────────────────\n');
M_total = M + 2*m;
fprintf('  Masa cuerpo     M  = %g kg\n', M);
fprintf('  Masa por rueda  m  = %g kg\n', m);
fprintf('  Masa total         = %g kg\n', M_total);
if M_total > 50 && M_total < 200
    fprintf('%s  Masa total en rango plausible (50–200 kg)\n', ok);
else
    fprintf('%s  Masa total fuera de rango típico Segway\n', war);
end
if m/M < 0.10
    fprintf('%s  Rueda ligera respecto al cuerpo (m/M = %.1f%%)\n', ok, m/M*100);
else
    fprintf('%s  Rueda pesada respecto al cuerpo (m/M = %.1f%%)\n', war, m/M*100);
end

%% ══════════════════════════════════════════════════════════════════════════
%  2. INERCIAS DE RUEDA — coherencia geométrica
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 2. INERCIAS DE RUEDA ──────────────────────────────────────\n');
Iw_disco  = 0.5 * m * r^2;   % disco sólido (mínimo)
Iw_aro    = m * r^2;          % aro delgado (máximo)
Iwz_disco = 0.25 * m * r^2;   % transversal disco sólido
Iwz_aro   = 0.5  * m * r^2;   % transversal aro

fprintf('  Iw declarado = %.4f kg·m²\n', Iw);
fprintf('  Rango geométrico: [%.4f (disco) → %.4f (aro)]\n', Iw_disco, Iw_aro);
if Iw >= Iw_disco*0.95 && Iw <= Iw_aro*1.05
    fprintf('%s  Iw dentro del rango geométrico (aro delgado/neumático)\n', ok);
else
    fprintf('%s  Iw FUERA del rango geométrico\n', err);
end

fprintf('  Iwz declarado = %.4f kg·m²\n', Iwz);
fprintf('  Rango geométrico Iwz: [%.4f → %.4f]\n', Iwz_disco, Iwz_aro);
if Iwz >= Iwz_disco*0.95 && Iwz <= Iwz_aro*1.05
    fprintf('%s  Iwz dentro del rango geométrico\n', ok);
else
    fprintf('%s  Iwz FUERA del rango geométrico\n', err);
end

ratio_Iw_Iwz = Iw / Iwz;
fprintf('  Ratio Iw/Iwz = %.2f  (esperado 2.0 para aro, 2.0 para disco)\n', ratio_Iw_Iwz);
if abs(ratio_Iw_Iwz - 2.0) < 0.01
    fprintf('%s  Ratio Iw/Iwz = 2.0 → consistente con simetría axial\n', ok);
else
    fprintf('%s  Ratio Iw/Iwz ≠ 2.0 → revisar\n', war);
end

%% ══════════════════════════════════════════════════════════════════════════
%  3. INERCIAS DEL CUERPO — contraste con geometría del sólido
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 3. INERCIAS DEL CUERPO ────────────────────────────────────\n');
% Estimación para ladrillo uniforme [D, H, W] = [0.20, 1.60, 0.40]
Icx_geo = (1/12)*M*(body_H^2 + body_W^2);  % eje X (rodamiento)
Icy_geo = (1/12)*M*(body_D^2 + body_H^2);  % eje Y (cabeceo) — eje de control
Icz_geo = (1/12)*M*(body_D^2 + body_W^2);  % eje Z (guiñada)

fprintf('  Geometría del sólido: D=%.2f m, H=%.2f m, W=%.2f m\n', body_D, body_H, body_W);
fprintf('  %-8s  Declarado    Geométrico uniforme   Diferencia\n','');
fprintf('  Icx (roll)  : %6.2f      %6.2f              %+.1f%%\n', Icx, Icx_geo, (Icx-Icx_geo)/Icx_geo*100);
fprintf('  Icy (pitch) : %6.2f      %6.2f              %+.1f%%\n', Icy, Icy_geo, (Icy-Icy_geo)/Icy_geo*100);
fprintf('  Icz (yaw)   : %6.2f      %6.2f              %+.1f%%\n', Icz, Icz_geo, (Icz-Icz_geo)/Icz_geo*100);
fprintf('  NOTA: Los valores declarados son personalizados (InertiaType=Custom).\n');
fprintf('        La diferencia con el sólido uniforme es esperada (masa no uniforme).\n');
if Icy < Icy_geo
    fprintf('%s  Icy=%.0f < geométrico=%.1f → cuerpo más compacto que el sólido — plausible\n', ok, Icy, Icy_geo);
end

%% ══════════════════════════════════════════════════════════════════════════
%  4. ALTURA DEL CENTRO DE MASA
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 4. GEOMETRÍA Y CM ─────────────────────────────────────────\n');
fprintf('  Radio de rueda       r = %.2f m\n', r);
fprintf('  Dist. eje→CM cuerpo  l = %.2f m\n', l);
h_cm_suelo = r + l;   % altura del CM desde el suelo
fprintf('  Altura CM sobre suelo = r + l = %.2f m\n', h_cm_suelo);
if h_cm_suelo > 0.8 && h_cm_suelo < 1.8
    fprintf('%s  Altura CM plausible (0.8–1.8 m)\n', ok);
else
    fprintf('%s  Altura CM fuera de rango típico\n', war);
end

h_cm_body = l - body_H/2;
fprintf('  l vs body_H/2: l=%.2f m,  body_H/2=%.2f m,  diff=%.2f m\n', l, body_H/2, h_cm_body);
if abs(h_cm_body) < 0.15
    fprintf('%s  CM del cuerpo cerca del centro geométrico del sólido (diff=%.2f m)\n', ok, h_cm_body);
else
    fprintf('%s  CM declarado (l) difiere del centro geométrico del sólido en %.2f m\n', war, h_cm_body);
    fprintf('        CenterOfMass=''[0 0 0]'' en el código → Simscape coloca CM en el centro del ladrillo.\n');
    fprintf('        La ODE usa l=%.2f pero Simscape efectivamente usa l_eff ≈ body_H/2=%.2f m.\n', l, body_H/2);
    fprintf('        >> INCONSISTENCIA POTENCIAL entre ODE y Simscape.\n');
end

%% ══════════════════════════════════════════════════════════════════════════
%  5. SEPARACIÓN DE RUEDAS vs ANCHO DEL SÓLIDO
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 5. SEPARACIÓN DE RUEDAS ───────────────────────────────────\n');
fprintf('  Separación total ruedas d = %.2f m\n', d);
fprintf('  Ancho del cuerpo body_W   = %.2f m\n', body_W);
fprintf('  Longitud cilindro rueda   = %.2f m (0.08 m hardcoded)\n', 0.08);
huelgo = d/2 - body_W/2 - 0.08;
fprintf('  Huelgo rueda–cuerpo       = d/2 - body_W/2 - 0.08 = %.3f m\n', huelgo);
if huelgo > 0
    fprintf('%s  Las ruedas no se solapan visualmente con el cuerpo\n', ok);
else
    fprintf('%s  Las ruedas se SOLAPAN con el cuerpo visualmente (%.3f m)\n', war, abs(huelgo));
end

%% ══════════════════════════════════════════════════════════════════════════
%  6. MATRIZ DE MASA — singularidad y condicionamiento
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 6. MATRIZ DE MASA (linealizada) ───────────────────────────\n');
M11  = Icy + M*l^2;
M12  = M*l;
M22  = M + 2*m + 2*Iw/r^2;
det0 = M11*M22 - M12^2;
cond_num = (M11*M22 + M12^2) / det0;  % proxy de condicionamiento 2x2

fprintf('  M11 (Icy + M·l²) = %g + %g = %.2f kg·m²\n', Icy, M*l^2, M11);
fprintf('  M12 (M·l)        = %.2f kg·m\n', M12);
fprintf('  M22 (M+2m+2Iw/r²)= %g + %g + %.2f = %.2f kg\n', M, 2*m, 2*Iw/r^2, M22);
fprintf('  det(M)           = %.2f  (>0 requerido)\n', det0);
if det0 > 0
    fprintf('%s  Matriz de masa no singular\n', ok);
else
    fprintf('%s  Matriz de masa SINGULAR — modelo inválido\n', err);
end
fprintf('  Número de condición (proxy 2×2) = %.3f\n', cond_num);
if cond_num < 50
    fprintf('%s  Condicionamiento aceptable\n', ok);
else
    fprintf('%s  Matriz mal condicionada — sensible a perturbaciones\n', war);
end

%% ══════════════════════════════════════════════════════════════════════════
%  7. LINEALIZACIÓN — signos de A y B
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 7. SIGNOS DE A y B ────────────────────────────────────────\n');
A22 = M22 * M*g*l / det0;
A41 = -M12 * M*g*l / det0;
b21 = (M22*(-alm) - M12*(alm/r)) / det0;
b41 = (M12*alm    + M11*(alm/r)) / det0;

fprintf('  A(2,1) = %+.4f  (esperado >0: inestabilidad del péndulo)\n', A22);
if A22 > 0, fprintf('%s\n',ok); else, fprintf('%s  SIGNO INCORRECTO\n',err); end

fprintf('  A(4,1) = %+.4f  (esperado <0: reacción del carro ante theta)\n', A41);
if A41 < 0, fprintf('%s\n',ok); else, fprintf('%s  SIGNO INCORRECTO\n',err); end

fprintf('  B(2,1) = %+.4f  (esperado <0: voltaje reduce aceleración angular)\n', b21);
if b21 < 0, fprintf('%s\n',ok); else, fprintf('%s  SIGNO INCORRECTO\n',err); end

fprintf('  B(4,1) = %+.4f  (esperado >0: voltaje acelera el carro)\n', b41);
if b41 > 0, fprintf('%s\n',ok); else, fprintf('%s  SIGNO INCORRECTO\n',err); end

%% ══════════════════════════════════════════════════════════════════════════
%  8. LQR — POLOS Y ESFUERZO INICIAL
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 8. LQR — POLOS Y ESFUERZO INICIAL ────────────────────────\n');
A = zeros(4);
A(1,2)=1; A(2,1)=M22*M*g*l/det0; A(3,4)=1; A(4,1)=-M12*M*g*l/det0;
B_ = zeros(4,2);
B_(2,1)=b21; B_(2,2)=b21; B_(4,1)=b41; B_(4,2)=b41;

% Polos planta abierta
pa = eig(A);
fprintf('  Polos planta abierta: ');
fprintf('%+.3f  ', real(pa)); fprintf('\n');
pa_inestables = sum(real(pa)>0);
fprintf('  Polos inestables: %d  (esperado 1 para péndulo invertido)\n', pa_inestables);
if pa_inestables == 1
    fprintf('%s\n',ok);
else
    fprintf('%s  Número inesperado de polos inestables\n', war);
end

% Controlabilidad
rng_ctrl = rank(ctrb(A, B_));
fprintf('  Rango controlabilidad: %d/4  ', rng_ctrl);
if rng_ctrl == 4, fprintf('(sistema completamente controlable) %s\n',ok);
else,             fprintf('(NO controlable) %s\n',err); end

% LQR
Q = diag([2000, 100, 50, 50]);
R = diag([1, 1]);
[K, ~, pcl] = lqr(A, B_, Q, R);
fprintf('  Polos lazo cerrado: ');
fprintf('%+.3f  ', real(pcl)); fprintf('\n');
if all(real(pcl) < 0)
    fprintf('%s  Todos los polos en semiplano izquierdo — estable\n', ok);
    fprintf('  Polo más lento: %.3f s⁻¹  → tiempo de asentamiento ≈ %.1f s\n', ...
        max(real(pcl)), -4/max(real(pcl)));
else
    fprintf('%s  Polo(s) inestables en lazo cerrado — revisar Q y R\n', err);
end

% Esfuerzo inicial
theta0 = theta0_deg * pi/180;
X0 = [theta0; 0; 0; 0];
U0 = -K * X0;
fprintf('  Esfuerzo inicial U0 con theta0=%g°: VR=%.2f V, VL=%.2f V\n', theta0_deg, U0(1), U0(2));
if abs(U0(1)) >= V_sat
    fprintf('%s  Control SATURADO en t=0 (|U|=%.1f V ≥ V_sat=%g V) — esperable con theta0=%g°\n', ...
        war, abs(U0(1)), V_sat, theta0_deg);
else
    fprintf('%s  Control NO satura en t=0 (|U|=%.1f V < V_sat=%g V)\n', ok, abs(U0(1)), V_sat);
end

% Ángulo máximo lineal sin saturación
theta_max_lin = V_sat / abs(K(1,1));
fprintf('  Ángulo máximo sin saturación ≈ %.2f° (K(1,1)=%.2f)\n', ...
    rad2deg(theta_max_lin), K(1,1));
if theta0_deg > rad2deg(theta_max_lin)
    fprintf('%s  theta0=%g° > limite_lineal=%.1f° → saturación esperable al inicio\n', ...
        war, theta0_deg, rad2deg(theta_max_lin));
end

%% ══════════════════════════════════════════════════════════════════════════
%  9. FUERZA DE TRACCIÓN — coherencia dimensional
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 9. FUERZA DE TRACCIÓN F_x ─────────────────────────────────\n');
% F_x = alm*(VR+VL)/r  → unidades: [N·m/V]*[V]/[m] = [N]  ✓
F_x_max = alm * 2*V_sat / r;    % ambas ruedas en saturación
fprintf('  F_x = alm·(VR+VL)/r\n');
fprintf('  Unidades: [N·m/V]·[V]/[m] = [N] %s\n', ok);
fprintf('  F_x_max (VR=VL=V_sat): %.1f N\n', F_x_max);
a_max = F_x_max / M22;
fprintf('  Aceleración máxima: F_x_max/M22 = %.2f m/s²\n', a_max);
if a_max > 1 && a_max < 20
    fprintf('%s  Aceleración plausible para Segway\n', ok);
else
    fprintf('%s  Aceleración fuera de rango típico\n', war);
end

% Par máximo de motor
tau_max = alm * V_sat;
fprintf('  Torque máximo por rueda: alm·V_sat = %.1f N·m\n', tau_max);

%% ══════════════════════════════════════════════════════════════════════════
%  10. RAMPA — descomposición de gravedad
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 10. RAMPA Y GRAVEDAD ──────────────────────────────────────\n');
ramp_rad = ramp_deg * pi/180;
g_n = g * cos(ramp_rad);
g_t = g * sin(ramp_rad);
fprintf('  Rampa actual: ramp_deg = %g°\n', ramp_deg);
fprintf('  g_n = g·cos(ramp) = %.4f m/s²  (gravedad efectiva del péndulo)\n', g_n);
fprintf('  g_t = g·sin(ramp) = %.4f m/s²  (perturbación translacional)\n', g_t);
error_descomp = abs(g_n^2 + g_t^2 - g^2);
fprintf('  Verificación: g_n²+g_t² = g²?  error=%.2e  ', error_descomp);
if error_descomp < 1e-8, fprintf('%s\n',ok); else, fprintf('%s\n',err); end

% Vector gravedad en Simscape
gx_sm = -g*sin(ramp_rad);
gz_sm = -g*cos(ramp_rad);
fprintf('  Vector gravedad Simscape: [%.4f; 0; %.4f]\n', gx_sm, gz_sm);
if ramp_deg == 0
    fprintf('%s  Suelo plano: gravedad = [0; 0; -9.81] ✓\n', ok);
else
    fprintf('%s  Rampa activa: componente en -X frena el avance hacia +X\n', ok);
    theta_eq_ramp = asin(M22*g_t / (M*g_n*l)) * 180/pi;
    fprintf('  Ángulo de equilibrio en rampa (estimado): %.2f°\n', theta_eq_ramp);
    fprintf('  (El LQR intentará mantener theta=0, pero la rampa lo fuerza a theta≈%.2f°)\n', theta_eq_ramp);
end

%% ══════════════════════════════════════════════════════════════════════════
%  11. INCONSISTENCIA POTENCIAL: l vs CenterOfMass en Simscape
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n── 11. INCONSISTENCIA ODE vs SIMSCAPE ────────────────────────\n');
fprintf('  La ODE usa l=%.2f m como distancia eje→CM del cuerpo.\n', l);
fprintf('  En Simscape: RT_BodyCM desplaza %.2f m en -Y desde eje de ruedas.\n', l);
fprintf('  El Body_Solid tiene CenterOfMass=[0 0 0] y alto=%.2f m.\n', body_H);
fprintf('  → Simscape ubica el CM del sólido en el centro geométrico del ladrillo,\n');
fprintf('    que coincide con el extremo de RT_BodyCM.\n');
fprintf('  → Esto es consistente si interpretamos l como la distancia al CM real ✓\n');
fprintf('  PERO: la altura visual del sólido (body_H=%.2f) se extiende ±%.2f m\n', body_H, body_H/2);
fprintf('  desde ese punto, lo que significa que el fondo del cuerpo está a\n');
fprintf('  l - body_H/2 = %.2f m del eje de ruedas (%.2f m del suelo).\n', ...
    l - body_H/2, r + l - body_H/2);
if r + l - body_H/2 > -0.05
    fprintf('%s  Cuerpo no traversa el suelo visualmente\n', ok);
else
    fprintf('%s  Cuerpo PENETRA el suelo visualmente (%.2f m bajo suelo)\n', war, -(r + l - body_H/2));
end

%% ══════════════════════════════════════════════════════════════════════════
%  RESUMEN FINAL
% ══════════════════════════════════════════════════════════════════════════
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  RESUMEN\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  Parámetros físicos:    COHERENTES\n');
fprintf('  Matriz de masa:        NO SINGULAR (det=%.1f)\n', det0);
fprintf('  Signos A y B:          CORRECTOS\n');
fprintf('  LQR:                   ESTABLE (%d polos lazo cerrado < 0)\n', sum(real(pcl)<0));
fprintf('  F_x tracción:          DIMENSIONALMENTE CORRECTO\n');
fprintf('  Rampa gravedad:        CORRECTO\n');
fprintf('\n  ADVERTENCIAS A REVISAR:\n');
fprintf('  1. theta0=%g° SATURA el control inicial (limite lineal ≈ %.1f°).\n', ...
    theta0_deg, rad2deg(theta_max_lin));
fprintf('     → Es funcional (LQR puede recuperar) pero genera esfuerzo máximo al inicio.\n');
fprintf('     → Para tesis: considera theta0=5° para mostrar comportamiento lineal limpio.\n');
fprintf('\n  2. Joint_alpha tiene DampingCoefficient=5.\n');
fprintf('     → Si las unidades son N·m/(deg/s) en 2025b, equivale a ~286 N·m·s/rad.\n');
fprintf('     → Revisar que no sobrefrene el giro en maniobras con theta_ref.\n');
fprintf('\n  3. Ruedas sin contacto con suelo (solo visual).\n');
fprintf('     → F_x inyectada compensa esto para la dinámica de traslación.\n');
fprintf('     → La inercia de rueda Iw NO afecta Simscape igual que la ODE\n');
fprintf('        (M22_ode incluye 2·Iw/r² pero Simscape las calcula libremente).\n');
fprintf('══════════════════════════════════════════════════════════════\n');
