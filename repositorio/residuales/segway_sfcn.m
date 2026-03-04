function Xdot = segway_sfcn(u)
% Planta no lineal Segway — Interpreted MATLAB Function
% u = [V_R(1); V_L(2); params(3..15); X(16..21)]

Xdot  = zeros(6,1);
V_R   = u(1);  V_L   = u(2);
M_b   = u(3);  m_w   = u(4);  r     = u(5);
d     = u(6);  l     = u(7);  g     = u(8);
Icy   = u(9);  Icz   = u(10); Icx   = u(11);
Iw    = u(12); Iwz   = u(13);
alp_m = u(14); bet_m = u(15);
theta  = u(16); dtheta = u(17);
dx     = u(19); dalpha = u(21);

omR   = dx/r + d/(2*r)*dalpha;
omL   = dx/r - d/(2*r)*dalpha;
tau_R = alp_m*V_R - bet_m*(omR - dtheta);
tau_L = alp_m*V_L - bet_m*(omL - dtheta);

M11 = M_b*l^2 + Icy;
M12 = M_b*l*cos(theta);
M22 = (M_b + 2*m_w) + 2*Iw/r^2;
M33 = (M_b + 2*m_w)*d^2/4 + 2*Iwz + Icz;

h1 = M_b*g*l*sin(theta) - (tau_R + tau_L);
h2 = M_b*l*sin(theta)*dtheta^2 + (tau_R + tau_L)/r;
h3 = d*(tau_R - tau_L)/(2*r);

det_M   = M11*M22 - M12*M12;
Xdot(1) = dtheta;
Xdot(2) = ( M22*h1 - M12*h2) / det_M;
Xdot(3) = dx;
Xdot(4) = (-M12*h1 + M11*h2) / det_M;
Xdot(5) = dalpha;
Xdot(6) = h3 / M33;
