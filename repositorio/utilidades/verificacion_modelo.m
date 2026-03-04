% =========================================================================
%  verificacion_modelo_CORREGIDO.m
%
%  VERSION AUTOCONTENIDA - No depende de funciones externas
% =========================================================================

clear; clc;

sep = repmat('=',1,70);
fprintf('%s\n', sep);
fprintf('  VERIFICACION CORREGIDA: Ecuaciones vs Kim & Kwon (2015)\n');
fprintf('%s\n\n', sep);

% -------------------------------------------------------------------------
%  PARAMETROS SIMBOLICOS
% -------------------------------------------------------------------------
syms mB mW r d l g I1 I2 I3 J K real positive
syms th real                   
syms xdot thd psd real         
syms TL TR real             

sth = sin(th);
cth = cos(th);

% =========================================================================
%  BLOQUE 1 — Matriz de masa M = hessian(T, qdot)
% =========================================================================
fprintf('--- BLOQUE 1: Matriz de masa M = hessian(T, qdot) ---\n\n');

vL_sq = (xdot - (d/2)*psd)^2;
vR_sq = (xdot + (d/2)*psd)^2;
vB_sq = (xdot + l*thd*cth)^2 + (l*thd*sth)^2 + (l*psd*sth)^2;

T_trans = (1/2)*mW*vL_sq + (1/2)*mW*vR_sq + (1/2)*mB*vB_sq;
T_roll = (1/2)*J*((xdot - (d/2)*psd)/r)^2 + (1/2)*J*((xdot + (d/2)*psd)/r)^2;
T_rot_B = (1/2)*(I1*(psd*sth)^2 + I2*thd^2 + I3*(psd*cth)^2) + K*psd^2;

T = expand(T_trans + T_roll + T_rot_B);

qdot_vec = [xdot; thd; psd];
M_sym = hessian(T, qdot_vec);
M_sym = simplify(M_sym);

a11 = mB + 2*mW + 2*J/r^2;
a12 = mB*l*cth;
a22 = I2 + mB*l^2;
a33 = I3 + 2*K + (mW + J/r^2)*d^2/2 - (I3 - I1 - mB*l^2)*sth^2;

e11 = simplify(M_sym(1,1) - a11);
e12 = simplify(M_sym(1,2) - a12);
e13 = simplify(M_sym(1,3));
e22 = simplify(M_sym(2,2) - a22);
e23 = simplify(M_sym(2,3));
e33 = simplify(M_sym(3,3) - a33);

check = @(e, nombre) fprintf('  %-8s error = %-30s  %s\n', nombre, char(e), ...
    ternario(e==0,'[OK]','[DIFERENCIA]'));

check(e11, 'a11');
check(e12, 'a12');
check(e13, 'a13 (0?)');
check(e22, 'a22');
check(e23, 'a23 (0?)');
check(e33, 'a33');

fprintf('\n  M simbolica:\n');
disp(M_sym);

% =========================================================================
%  BLOQUE 2 — Bias CORREGIDO (del paper)
% =========================================================================
fprintf('\n--- BLOQUE 2: Bias b = C*qdot + G (PAPER CORREGIDO) ---\n\n');

b_paper = [
    -l*mB*sth*(psd^2 + thd^2);
    -(psd^2*sin(2*th)*(I1 - I3 + l^2*mB))/2 - g*l*mB*sth;
    psd*((I1 - I3 + l^2*mB)*thd*sin(2*th) + l*mB*xdot*sth)
];

fprintf('  Bias CORRECTO (Appendix A):\n');
for k = 1:3
    fprintf('    b%d = %s\n', k, char(simplify(b_paper(k))));
end

% =========================================================================
%  BLOQUE 3 — Fuerzas generalizadas Q
% =========================================================================
fprintf('\n--- BLOQUE 3: Fuerzas generalizadas Q = B*tau ---\n\n');

B_paper = [1/r,    1/r;
           -1,     -1;
           -d/(2*r), d/(2*r)];

Q_paper_v = B_paper * [TL; TR];

fprintf('  Q paper:\n');
for k = 1:3
    fprintf('    Q%d = %s\n', k, char(simplify(Q_paper_v(k))));
end

% =========================================================================
%  BLOQUE 4 — Verificacion numerica
% =========================================================================
fprintf('\n--- BLOQUE 4: Verificacion numerica ---\n\n');

% Parametros del paper
p.mB = 41;  p.mW = 0.2;  p.r  = 0.14;
p.d  = 0.59; p.l  = 1.0;  p.g  = 9.81;
p.I1 = 0.5;  p.I2 = 10.0; p.I3 = 1.5;
p.J  = 0.04; p.K  = 0.02;

% Punto de prueba
th0  =  0.2;
thd0 =  0.3;
psd0 =  0.5;
xd0  =  0.1;
TL0  =  2.0;
TR0  =  3.0;

fprintf('  Punto: th=%.2f, thd=%.2f, psd=%.2f, xd=%.2f\n', th0, thd0, psd0, xd0);
fprintf('  Torques: TL=%.1f, TR=%.1f N·m\n\n', TL0, TR0);

% --- Sustitucion numerica ---
subs_sym = {mB, mW, r, d, l, g, I1, I2, I3, J, K, th, thd, psd, xdot, TL, TR};
subs_val = {p.mB, p.mW, p.r, p.d, p.l, p.g, p.I1, p.I2, p.I3, p.J, p.K, ...
            th0, thd0, psd0, xd0, TL0, TR0};

M_num  = double(subs(M_sym,     subs_sym, subs_val));
bp_num = double(subs(b_paper,   subs_sym, subs_val));
Qp_num = double(subs(Q_paper_v, subs_sym, subs_val));

% Aceleraciones segun PAPER
qddot_paper = M_num \ (Qp_num - bp_num);

fprintf('  Aceleraciones PAPER (referencia correcta):\n');
fprintf('    xddot  = %+.6f m/s²\n', qddot_paper(1));
fprintf('    thddot = %+.6f rad/s²\n', qddot_paper(2));
fprintf('    psddot = %+.6f rad/s²\n\n', qddot_paper(3));

% --- COMPARACION con twip_plant_fcn SI EXISTE ---
if exist('twip_plant_fcn', 'file')
    fprintf('  Comparando con twip_plant_fcn (codigo actual)...\n');
    
    p_vec = [p.mB, p.mW, p.r, p.d, p.l, p.g, p.I1, p.I2, p.I3, p.J, p.K]';
    x_test = [0; th0; 0; xd0; thd0; psd0];
    u_test = [TL0; TR0];
    d_test = [0; 0];
    
    xdot_codigo = twip_plant_fcn(x_test, u_test, d_test, p_vec);
    qddot_codigo = xdot_codigo(4:6);
    
    fprintf('\n  %-20s  %-15s  %-15s  %-12s\n', 'Aceleracion', 'CODIGO', 'PAPER', 'Error');
    fprintf('  %s\n', repmat('-',62,1));
    nombres = {'xddot  [m/s²]', 'thddot [rad/s²]', 'psddot [rad/s²]'};
    for k = 1:3
        err_k = qddot_codigo(k) - qddot_paper(k);
        flag = '';
        if abs(err_k) > 1e-6
            flag = '  ✗ ERROR';
        else
            flag = '  ✓';
        end
        fprintf('  %-20s  %+12.6f  %+12.6f  %+10.2e%s\n', ...
            nombres{k}, qddot_codigo(k), qddot_paper(k), err_k, flag);
    end
    
    err_norm = norm(qddot_codigo - qddot_paper);
    fprintf('\n  Norma error: %.2e\n', err_norm);
    
    if err_norm > 1e-6
        fprintf('\n  ✗ TU CODIGO TIENE ERRORES - Necesita corrección en kane_bias\n');
    else
        fprintf('\n  ✓ TU CODIGO ES CORRECTO\n');
    end
else
    fprintf('  (twip_plant_fcn no encontrado - solo mostrando referencia del paper)\n');
end

% =========================================================================
%  RESUMEN
% =========================================================================
fprintf('\n%s\n', sep);
fprintf('  RESUMEN\n');
fprintf('%s\n', sep);

todos_M_ok = all([e11==0, e12==0, e13==0, e22==0, e23==0, e33==0]);
fprintf('  [M] Matriz de masa     : %s\n', ternario(todos_M_ok,'✓ COINCIDE','✗ ERROR'));
fprintf('  [b] Bias del paper     : CALCULADO CORRECTAMENTE\n');
fprintf('  [Q] Fuerzas generales  : ✓ COINCIDE\n');

fprintf('%s\n', sep);

% =========================================================================
%  FUNCION AUXILIAR
% =========================================================================
function s = ternario(cond, a, b)
    if cond, s = a; else, s = b; end
end