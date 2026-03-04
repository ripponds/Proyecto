function d_pitch = twip_bache_fcn(x_pos, x_dot, p_bache)
% =========================================================================
%  twip_bache_fcn.m — Perturbación geométrica del bache
%
%  p_bache [3x1] = [bache_pos; bache_h; bache_w]
%
%  Física: rueda sobre perfil parabólico z(x)
%  tau = mB·l·xdot²·d²z/dx²   (d²z/dx² constante dentro del bache)
% =========================================================================
bpos = p_bache(1);
bh   = p_bache(2);
bw   = p_bache(3);

if abs(x_pos - bpos) < bw/2
    d2z     = 2*bh / (bw/2)^2;          % curvatura positiva del perfil z(x)
    % Torque de perturbación: mB*l * xdot^2 * d2z  (reacción normal → cabeceo)
    % mB*l = 45*0.35 = 15.75  (hardcodeado para codegen)
    d_pitch = max(min(15.75 * x_dot^2 * d2z, 15), -15);  % saturado a rango actuador
else
    d_pitch = 0;
end
end