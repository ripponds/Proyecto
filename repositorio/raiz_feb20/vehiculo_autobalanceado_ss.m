%% =========================================================================
%  MODELO COMPLETO — VEHICULO AUTOBALANCEADO DE DOS RUEDAS
%  Motor: ZD101AZ / MY1016Z (250W, 24V)
%  6 estados, 2 entradas (VL, VR), ecuaciones linealizadas
%  =========================================================================
clear; clc; close all;

%% =====================================================================
%  PARAMETROS MODIFICABLES
%  =====================================================================

% --- Motor ZD101AZ (referidos al rotor) ---
Rm   = 0.399;       % [Ohm]      Resistencia del devanado
Kt   = 0.0650;      % [Nm/A]     Constante de par (rotor)
Ke   = 0.0650;      % [Vs/rad]   Constante de back-EMF (rotor)
n_g  = 9.77;        % [-]        Reduccion interna (engranajes)
n_ch = 44/9;        % [-]        Reduccion por cadena (corona/sprocket)
eta  = 0.78;        % [-]        Eficiencia global de la transmision

% --- Reduccion total ---
n_t = n_g * n_ch;   % [-] Reduccion total motor-a-rueda

% --- Constantes del motor referidas al eje de la rueda ---
%  tau_rueda = n_t * eta * Kt * i       =>  Kt_w = n_t * eta * Kt
%  back-EMF:  e = Ke * omega_rotor = Ke * n_t * omega_rueda
%                                       =>  Ke_w = Ke * n_t
%  Ecuacion del par en la rueda:
%    tau = (Kt_w/Rm)*V - (Kt_w*Ke_w/Rm)*omega_rel
%  donde omega_rel = dphi_rueda - dtheta (velocidad relativa rueda-cuerpo)

Kt_w = n_t * eta * Kt;       % [Nm/A]    Par por amperio en la rueda
Ke_w = Ke * n_t;              % [Vs/rad]  Back-EMF referida a la rueda
k1   = Kt_w / Rm;            % [Nm/V]    Par en la rueda por voltio
k2   = Kt_w * Ke_w / Rm;     % [Nms/rad] Amortiguamiento electrico

% --- Masas ---
mb = 104;            % [kg]   Masa del cuerpo (plataforma 24 + persona 80)
mr = 3.24;           % [kg]   Masa de cada rueda

% --- Geometria ---
Rw = 0.178;          % [m]    Radio de la rueda (rin 14")
Lc = 0.55;           % [m]    Distancia eje ruedas al centro de masa
dw = 0.25;           % [m]    Semi-separacion entre ruedas

% --- Inercias ---
Ib_theta = 8.67;     % [kg m^2] Inercia cabeceo del cuerpo respecto a su c.m.
Ib_psi   = 2.17;     % [kg m^2] Inercia giro del cuerpo respecto a su c.m.
Ir       = 0.5*mr*Rw^2;  % [kg m^2] Inercia de cada rueda (disco solido)

% --- Gravedad ---
g = 9.81;            % [m/s^2]

%% =====================================================================
%  CONSTANTES AGRUPADAS (ecuaciones de movimiento)
%  =====================================================================
%
%  Subsistema cabeceo-avance (phi, theta):
%    alpha = (2*mr + mb)*Rw^2 + 2*Ir
%    beta  = mb*Lc*Rw
%    gamma = Ib_theta + mb*Lc^2
%    Delta = alpha*gamma - beta^2
%
%  Subsistema giro (psi):
%    I_psi = Ib_psi + 2*mr*dw^2 + 2*Ir*dw^2/Rw^2

alpha = (2*mr + mb)*Rw^2 + 2*Ir;        % [kg m^2]
beta  = mb*Lc*Rw;                         % [kg m^2]
gamma = Ib_theta + mb*Lc^2;              % [kg m^2]
Delta = alpha*gamma - beta^2;            % [kg^2 m^4]
I_psi = Ib_psi + 2*mr*dw^2 + 2*Ir*dw^2/Rw^2;  % [kg m^2]

%% =====================================================================
%  ECUACIONES NO LINEALES (para referencia)
%  =====================================================================
%  Sean:
%    phi   = rotacion media de ruedas (avance: x = Rw*phi)
%    theta = angulo de cabeceo (0 = vertical)
%    psi   = angulo de giro (yaw)
%    V_sum = VL + VR  (voltaje suma)
%    V_dif = VR - VL  (voltaje diferencial)
%    tau_sum = par total sobre las ruedas
%    tau_dif = par diferencial
%
%  Par del motor en funcion del voltaje:
%    tau_L = k1*VL - k2*(dphi - dtheta)    (motor izquierdo)
%    tau_R = k1*VR - k2*(dphi - dtheta)    (motor derecho)
%    tau_sum = tau_L + tau_R = k1*V_sum - 2*k2*(dphi - dtheta)
%    tau_dif = tau_R - tau_L = k1*V_dif
%
%  Ecuacion 1 — Avance (phi):
%    alpha*ddphi + beta*ddtheta*cos(theta) - beta*dtheta^2*sin(theta)
%      + 2*k2*(dphi - dtheta) = k1*V_sum
%
%  Ecuacion 2 — Cabeceo (theta):
%    gamma*ddtheta + beta*ddphi*cos(theta) - mb*g*Lc*sin(theta)
%      - 2*k2*(dphi - dtheta) = -k1*V_sum
%
%  Ecuacion 3 — Giro (psi):
%    I_psi*ddpsi = (dw/Rw)*k1*V_dif
%
%  Signos verificados:
%    - Gravedad mb*g*Lc*sin(theta): desestabiliza (correcto)
%    - Reaccion del motor sobre cuerpo: -k1*V_sum (correcto)
%    - Par en las ruedas: +k1*V_sum/Rw como fuerza de contacto (correcto)
%    - Amortiguamiento 2*k2*(dphi-dtheta): frena velocidad relativa (correcto)

%% =====================================================================
%  LINEALIZACION (theta ~ 0: sin(theta)->theta, cos(theta)->1)
%  =====================================================================
%  Ec.1: alpha*ddphi + beta*ddtheta + 2*k2*dphi - 2*k2*dtheta = k1*V_sum
%  Ec.2: beta*ddphi + gamma*ddtheta - mb*g*Lc*theta - 2*k2*dphi + 2*k2*dtheta = -k1*V_sum
%  Ec.3: I_psi*ddpsi = (dw/Rw)*k1*V_dif
%
%  Resolviendo Ec.1 y Ec.2 para ddphi y ddtheta:
%    [alpha  beta ] [ddphi  ]   [k1*V_sum - 2*k2*(dphi-dtheta)             ]
%    [beta   gamma] [ddtheta] = [-k1*V_sum + mb*g*Lc*theta + 2*k2*(dphi-dtheta)]
%
%  Invirtiendo la matriz de masa (determinante = Delta):
%    ddphi   = (1/Delta)*[ gamma*(k1*Vsum - 2*k2*(dphi-dtheta))
%              - beta*(-k1*Vsum + mb*g*Lc*theta + 2*k2*(dphi-dtheta)) ]
%
%    ddtheta = (1/Delta)*[ -beta*(k1*Vsum - 2*k2*(dphi-dtheta))
%              + alpha*(-k1*Vsum + mb*g*Lc*theta + 2*k2*(dphi-dtheta)) ]

%% =====================================================================
%  COEFICIENTES PARA LAS MATRICES
%  =====================================================================

% --- ddphi = a21*dphi + a22*dtheta + a23*theta + b21*V_sum ---
a21 = -2*k2*(gamma + beta) / Delta;
a22 =  2*k2*(gamma + beta) / Delta;
a23 = -beta*mb*g*Lc / Delta;
b21 =  (gamma + beta)*k1 / Delta;

% --- ddtheta = a41*dphi + a42*dtheta + a43*theta + b41*V_sum ---
a41 =  2*k2*(alpha + beta) / Delta;
a42 = -2*k2*(alpha + beta) / Delta;
a43 =  alpha*mb*g*Lc / Delta;
b41 = -(alpha + beta)*k1 / Delta;

% --- ddpsi = b62*V_dif ---
b62 = dw*k1 / (Rw*I_psi);

%% =====================================================================
%  ESPACIO DE ESTADOS COMPLETO (6 estados, 2 entradas)
%  =====================================================================
%  Estado: x = [phi; dphi; theta; dtheta; psi; dpsi]
%  Entrada: u = [V_sum; V_dif] = [VL+VR; VR-VL]
%  Salida: y = [phi; theta; psi] (las tres posiciones angulares)

A_full = [0    1     0     0     0    0;
          0    a21   a23   a22   0    0;
          0    0     0     1     0    0;
          0    a41   a43   a42   0    0;
          0    0     0     0     0    1;
          0    0     0     0     0    0];

B_full = [0     0;
          b21   0;
          0     0;
          b41   0;
          0     0;
          0     b62];

C_full = [1 0 0 0 0 0;
          0 0 1 0 0 0;
          0 0 0 0 1 0];

D_full = zeros(3, 2);

sys_full = ss(A_full, B_full, C_full, D_full);
sys_full.StateName  = {'phi','dphi','theta','dtheta','psi','dpsi'};
sys_full.InputName  = {'V_sum','V_dif'};
sys_full.OutputName = {'phi','theta','psi'};

%% =====================================================================
%  ESPACIO DE ESTADOS: CABECEO + AVANCE (4 estados, 1 entrada)
%  =====================================================================
%  Estado: x = [phi; dphi; theta; dtheta]
%  Entrada: u = V_sum = VL + VR
%  Salida: y = [phi; theta]

A_pitch = [0    1     0     0;
           0    a21   a23   a22;
           0    0     0     1;
           0    a41   a43   a42];

B_pitch = [0;
           b21;
           0;
           b41];

C_pitch = [1 0 0 0;
           0 0 1 0];

D_pitch = zeros(2, 1);

sys_pitch = ss(A_pitch, B_pitch, C_pitch, D_pitch);
sys_pitch.StateName  = {'phi','dphi','theta','dtheta'};
sys_pitch.InputName  = {'V_sum'};
sys_pitch.OutputName = {'phi','theta'};

%% =====================================================================
%  ESPACIO DE ESTADOS: GIRO (2 estados, 1 entrada)
%  =====================================================================
%  Estado: x = [psi; dpsi]
%  Entrada: u = V_dif = VR - VL
%  Salida: y = psi

A_yaw = [0  1;
         0  0];

B_yaw = [0;
         b62];

C_yaw = [1 0];

D_yaw = 0;

sys_yaw = ss(A_yaw, B_yaw, C_yaw, D_yaw);
sys_yaw.StateName  = {'psi','dpsi'};
sys_yaw.InputName  = {'V_dif'};
sys_yaw.OutputName = {'psi'};

%% =====================================================================
%  ANALISIS DEL SISTEMA
%  =====================================================================

% Polos en lazo abierto
polos_full  = eig(A_full);
polos_pitch = eig(A_pitch);
polos_yaw   = eig(A_yaw);

rango_ctrl_full  = rank(ctrb(A_full, B_full));
rango_ctrl_pitch = rank(ctrb(A_pitch, B_pitch));
rango_ctrl_yaw   = rank(ctrb(A_yaw, B_yaw));

rango_obs_full  = rank(obsv(A_full, C_full));
rango_obs_pitch = rank(obsv(A_pitch, C_pitch));
rango_obs_yaw   = rank(obsv(A_yaw, C_yaw));

%% =====================================================================
%  GRAFICAS — RESPUESTA EN LAZO ABIERTO
%  =====================================================================

% --- Figura 1: Cabeceo+avance, condicion inicial theta=5 deg ---
%  Salidas: phi y theta (C_pitch ya las selecciona)
figure('Name','Cabeceo + Avance — Lazo Abierto','Position',[50 300 900 400]);
x0_pitch = [0; 0; 5*pi/180; 0];
initial(sys_pitch, x0_pitch, 2);
sgtitle('Cabeceo + Avance — Condicion inicial \theta_0 = 5° (inestable)');

% --- Figura 2: Mapa de polos ---
figure('Name','Mapa de Polos','Position',[50 50 900 400]);
subplot(1,2,1); pzmap(sys_pitch); title('Cabeceo + Avance'); grid on;
subplot(1,2,2); pzmap(sys_yaw);   title('Giro');              grid on;
sgtitle('Polos en lazo abierto');

% --- Figura 3: Giro, escalon V_dif = 1 V ---
figure('Name','Giro — Lazo Abierto','Position',[500 300 700 350]);
step(sys_yaw, 3);
title('Giro — Escalon V_{dif} = 1 V');
grid on;

%% =====================================================================
%  CONVERSION DE ENTRADAS: V_sum/V_dif <-> VL/VR
%  =====================================================================
%  V_sum = VL + VR     V_dif = VR - VL
%  VL = (V_sum - V_dif) / 2
%  VR = (V_sum + V_dif) / 2
%
%  Para usar el sistema con entradas VL, VR:
%  u_fisico = [VL; VR]
%  u_modelo = [V_sum; V_dif] = T * u_fisico
%  donde T = [1  1; -1  1]

T_conv = [1  1;
         -1  1];

B_full_LR = B_full * T_conv;
sys_full_LR = ss(A_full, B_full_LR, C_full, zeros(3,2));
sys_full_LR.StateName  = {'phi','dphi','theta','dtheta','psi','dpsi'};
sys_full_LR.InputName  = {'VL','VR'};
sys_full_LR.OutputName = {'phi','theta','psi'};

% Las matrices quedan en el workspace:
%  A_full, B_full, C_full         (6x6, entradas V_sum, V_dif)
%  A_pitch, B_pitch, C_pitch      (4x4, entrada V_sum)
%  A_yaw, B_yaw, C_yaw            (2x2, entrada V_dif)
%  B_full_LR                       (6x2, entradas VL, VR)