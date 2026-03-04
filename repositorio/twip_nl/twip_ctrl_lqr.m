function u = twip_ctrl_lqr(x, K_lqr)
% =========================================================================
%  twip_ctrl_lqr.m — LQR de equilibrio
%  Cambiar esta función para probar otros controladores.
%  Firma fija: recibe x[6x1], devuelve u[2x1]
% =========================================================================
u = max(min(-K_lqr * x, 15), -15);
end