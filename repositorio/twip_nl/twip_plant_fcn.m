function xdot = twip_plant_fcn(x, u, d, p_phys)
% =========================================================================
%  twip_plant_fcn.m — Dinámica no lineal del TWIP (Kane 3D)
%
%  Formulación de Kane:  M(q) * qddot = Q(q,qdot,u) - b(q,qdot)
%
%  Ref: Kim & Kwon, IJCAS 2015, Vol.13 No.4 — método de Kane
% =========================================================================
%% Guardia: colapso al suelo (restitución suave)
THETA_MAX = 1.3963;   % 80 deg
th = x(2);
if abs(th) >= THETA_MAX
    xdot    = zeros(6,1);
    xdot(2) = -sign(th) * 50 * (abs(th) - THETA_MAX);
return;
end
%% Saturación de actuadores  [N·m]
u_sat    = max(min(u, 15), -15);
tau_sum  = u_sat(1) + u_sat(2);   % avance / cabeceo
tau_diff = u_sat(2) - u_sat(1);   % guiñada
%% Variables de estado
qdot = x(4:6);      % [xdot; thdot; psdot]
xd   = x(4);        % ← AGREGADO para kane_bias
thd  = x(5);
psd  = x(6);
%% Ecuaciones de Kane
M = kane_mass_matrix(th,  p_phys);
b = kane_bias       (th, thd, psd, xd, p_phys);  % ← AGREGADO xd
Q = kane_gen_forces (tau_sum, tau_diff, d, p_phys);
qddot = M \ (Q - b);
xdot = [qdot; qddot];
end

% =========================================================================
%  SUBFUNCIONES — Kane 3D
% =========================================================================
function M = kane_mass_matrix(th, p)
% M(q) = hessian( T, qdot )   — Hessiano de la energía cinética
mB=p(1); mW=p(2); r=p(3); dw=p(4); l=p(5);
I1=p(7); I2=p(8); I3=p(9); J=p(10); K=p(11);
M11 = mB + 2*mW + 2*J/r^2;
M12 = mB * l * cos(th);
M22 = I2 + mB*l^2;
M33 = I3 + 2*K + (mW + J/r^2)*dw^2/2 - (I3 - I1 - mB*l^2)*sin(th)^2;
M = [M11, M12,   0  ;
     M12, M22,   0  ;
       0,   0,  M33 ];
end

function b = kane_bias(th, thd, psd, xd, p)
% b(q,qdot) = C*qdot + G — CORREGIDO según Kim & Kwon Appendix A
%
% Incluye:
%   b1 — acoplamiento pitch-yaw en traslación (AMBOS thd^2 y psd^2)
%   b2 — Coriolis de yaw + gravedad
%   b3 — acoplamiento traslación-yaw + Coriolis pitch-yaw
%
% CRÍTICO: Implementa EXACTAMENTE las ecuaciones del paper
mB=p(1); l=p(5); g=p(6); I1=p(7); I3=p(9);

sth  = sin(th);
s2th = sin(2*th);  % = 2*sin(th)*cos(th)

% ECUACIONES EXACTAS DEL APPENDIX A
b1 = -l*mB*sth*(psd^2 + thd^2);  % ← FIX: agregado thd^2

b2 = -(psd^2*s2th*(I1 - I3 + l^2*mB))/2 - g*l*mB*sth;  % ← FIX: agregado término Coriolis

b3 = psd*((I1 - I3 + l^2*mB)*thd*s2th + l*mB*xd*sth);  % ← FIX: agregado thd*s2th y xd*sth

b = [b1; b2; b3];
end

function Q = kane_gen_forces(tau_sum, tau_diff, d, p)
% Q — fuerzas generalizadas activas
r  = p(3);
dw = p(4);
Q1 =  tau_sum  / r;
Q2 = -tau_sum        + d(1);
Q3 =  (dw/(2*r)) * tau_diff + d(2);
Q = [Q1; Q2; Q3];
end