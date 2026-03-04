%% Simscape_Segway_LQR_Fixed_2025b.m
% Visualización 3D del Segway con control LQR inyectado (From Workspace).
% Puertos corregidos para MATLAB 2025b.

clear; clc; close all; bdclose all;

%% ── 1. PARÁMETROS ────────────────────────────────────────────────────────
M=80; r=0.20; d=0.60; l=0.90; g=9.81;
m=2;  Icy=10; Icz=12; Icx=12;
Iw=0.08; Iwz=0.04;
alm=2.0; bem=1.5;
body_W=0.40; body_D=0.20; body_H=1.60;
theta0_deg = 5;
theta0     = theta0_deg * pi/180;
t_sim      = 5;

% Parámetros de simulación en Lazo Abierto
M11=Icy+M*l^2; M12=M*l; M22=M+2*m+2*Iw/r^2;
M33=Icz+2*m*(d/2)^2+2*Iw*(d/(2*r))^2+2*Iwz;
det0=M11*M22-M12^2;
dF1_dth=M*g*l; dF1_doth=-2*bem; dF1_dox=2*bem/r;  dF1_VR=-alm;    dF1_VL=-alm;
dF2_doth=2*bem/r; dF2_dox=-2*bem/r^2; dF2_VR=alm/r; dF2_VL=alm/r;
dF3_doal=-bem*d^2/(2*r^2); dF3_VR=alm*d/(2*r); dF3_VL=-alm*d/(2*r);

A=zeros(6); A(1,2)=1; A(3,4)=1; A(5,6)=1;
A(2,1)=(M22*dF1_dth)/det0;
A(2,2)=(M22*dF1_doth-M12*dF2_doth)/det0;
A(2,4)=(M22*dF1_dox -M12*dF2_dox)/det0;
A(4,1)=(-M12*dF1_dth)/det0;
A(4,2)=(-M12*dF1_doth+M11*dF2_doth)/det0;
A(4,4)=(-M12*dF1_dox +M11*dF2_dox)/det0;
A(6,6)=dF3_doal/M33;

B=zeros(6,2);
B(2,1)=(M22*dF1_VR-M12*dF2_VR)/det0; B(2,2)=(M22*dF1_VL-M12*dF2_VL)/det0;
B(4,1)=(-M12*dF1_VR+M11*dF2_VR)/det0; B(4,2)=(-M12*dF1_VL+M11*dF2_VL)/det0;
B(6,1)=dF3_VR/M33; B(6,2)=dF3_VL/M33;

Q=diag([2000,100,50,50,800,50]); R=diag([1,1]);
[K,~,~]=lqr(A,B,Q,R);
V_sat=12;

%% ── 2. PRE-COMPUTAR ODE PARA From Workspace ────────────────────────────
X0=[theta0;0;0;0;0;0];
ode_fn=@(t,X) ode_nl_lqr(t,X,K,V_sat,M,m,r,d,l,g,Icy,Icz,Icx,Iw,Iwz,alm,bem);
[t_ode, X_ode]=ode45(ode_fn,[0 t_sim],X0);

tau_R_arr = zeros(numel(t_ode),1);
tau_L_arr = zeros(numel(t_ode),1);
for i=1:numel(t_ode)
    Xi = X_ode(i,:)';
    U  = max(min(-K*Xi, V_sat), -V_sat);
    dx_i=Xi(4); dal_i=Xi(6); dth_i=Xi(2);
    tau_R_arr(i) = alm*U(1) - bem*(dx_i/r + d*dal_i/(2*r) - dth_i);
    tau_L_arr(i) = alm*U(2) - bem*(dx_i/r - d*dal_i/(2*r) - dth_i);
end

simin_tauR = timeseries(tau_R_arr, t_ode);
simin_tauL = timeseries(tau_L_arr, t_ode);
assignin('base','simin_tauR', simin_tauR);
assignin('base','simin_tauL', simin_tauL);

%% ── 3. CONSTRUCCIÓN DE SIMSCAPE ─────────────────────────────────────────
modelName = 'Segway_LQR_Valid';
if bdIsLoaded(modelName), close_system(modelName,0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName); open_system(modelName);

set_param(modelName,'Solver','ode23t','StopTime',num2str(t_sim), ...
    'SimscapeLogType','all','SimscapeLogName','simlog', 'SaveOutput','on');

% Componentes base
add_block('nesl_utility/Solver Configuration',[modelName '/Solver'],'Position',[30 30 180 60]);
add_block('sm_lib/Utilities/Mechanism Configuration',[modelName '/Mech'],'Position',[30 90 220 120]);
add_block('sm_lib/Frames and Transforms/World Frame',[modelName '/World'],'Position',[30 160 130 190]);

% Cinemática
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[220 160 340 190]);
set_param([modelName '/RT_AxleHeight'],'TranslationMethod','StandardAxis','TranslationStandardAxis','+Z','TranslationStandardOffset',num2str(r));

add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[400 160 520 190]);
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[580 160 700 190]);

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[760 160 880 190]);
set_param([modelName '/RT_ThetaAxis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[940 160 1060 190]);
set_param([modelName '/Joint_theta'],'PositionTargetSpecify','on','PositionTargetValue',num2str(theta0_deg),'PositionTargetValueUnits','deg','PositionTargetPriority','High');

% IMPORTANTE: Amortiguamiento ligero. Estabiliza el solver implícito frente al torque Open-Loop.
set_param([modelName '/Joint_theta'], 'DampingCoefficient', '0.05');

% Cuerpo Principal
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1120 160 1240 190]);
set_param([modelName '/RT_BodyCM'],'TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(l));
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body'],'Position',[1300 160 1420 190]);
set_param([modelName '/Body'],'BrickDimensions',mat2str([body_D,body_H,body_W]),'Mass',num2str(M),'MomentsOfInertia',mat2str([Icx,Icy,Icz]), 'GraphicDiffuseColor','[0.2 0.5 0.8]');

% Rueda Derecha y Actuación
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[760 260 880 290]);
set_param([modelName '/RT_WheelR'],'TranslationMethod','StandardAxis','TranslationStandardAxis','+Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Ax'],'Position',[940 260 1060 290]);
set_param([modelName '/RT_WheelR_Ax'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[1120 260 1240 290]);
% CRÍTICO: Configurar el puerto exacto para Torque Actuation
set_param([modelName '/Joint_phi_R'], 'TorqueActuationMode','InputTorque'); 

add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[1300 260 1420 290]);
set_param([modelName '/Wheel_R'],'CylinderRadius',num2str(r),'CylinderLength','0.08','Mass',num2str(m),'MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'GraphicDiffuseColor','[0.15 0.15 0.15]');

% Conexiones Básicas (Autorouting directo usando los nombres de los puertos para evitar el bug del índice)
add_line(modelName, 'World/RConn1', 'Solver/RConn1');
add_line(modelName, 'World/RConn1', 'Mech/RConn1');
add_line(modelName, 'World/RConn1', 'RT_AxleHeight/LConn1');
add_line(modelName, 'RT_AxleHeight/RConn1', 'Joint_x/LConn1');
add_line(modelName, 'Joint_x/RConn1', 'Joint_alpha/LConn1');
add_line(modelName, 'Joint_alpha/RConn1', 'RT_ThetaAxis/LConn1');
add_line(modelName, 'RT_ThetaAxis/RConn1', 'Joint_theta/LConn1');
add_line(modelName, 'Joint_theta/RConn1', 'RT_BodyCM/LConn1');
add_line(modelName, 'RT_BodyCM/RConn1', 'Body/RConn1');

add_line(modelName, 'Joint_alpha/RConn1', 'RT_WheelR/LConn1');
add_line(modelName, 'RT_WheelR/RConn1', 'RT_WheelR_Ax/LConn1');
add_line(modelName, 'RT_WheelR_Ax/RConn1', 'Joint_phi_R/LConn1');
add_line(modelName, 'Joint_phi_R/RConn1', 'Wheel_R/RConn1');

% Inyección de Señal (La Solución)
add_block('simulink/Sources/From Workspace',[modelName '/FW_tauR'],'Position',[760 360 880 390]);
set_param([modelName '/FW_tauR'],'VariableName','simin_tauR');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/S2P_R'],'Position',[940 360 1060 390]);

add_line(modelName, 'FW_tauR/1', 'S2P_R/1');
% El puerto de actuación de Torque se llama 't' en la API. Se conecta al LConn del S2P.
add_line(modelName, 'S2P_R/RConn1', 'Joint_phi_R/LConn2'); % El Inport físico suele mapearse a LConn(2) si LConn(1) es la base.

fprintf('✔ Modelo construido con conexiones seguras.\n');

%% ── 4. EJECUCIÓN ────────────────────────────────────────────────────────
set_param(modelName, 'SimulationCommand', 'update');
out = sim(modelName);

%% ── 5. VISUALIZACIÓN DARK MODE ──────────────────────────────────────────
% Forzando extracción segura del simlog
if isprop(out, 'logsout') && ~isempty(out.get('simlog'))
    logData = out.get('simlog');
    t_sm = logData.Joint_theta.Rz.q.series.time;
    theta_sm = logData.Joint_theta.Rz.q.series.values('deg');
    
    set(groot, 'defaultFigureColor', [0.1 0.1 0.1], 'defaultAxesColor', [0.15 0.15 0.15]);
    set(groot, 'defaultAxesXColor', 'w', 'defaultAxesYColor', 'w', 'defaultTextColor', 'w');
    
    figure('Name','Validación Final LQR','NumberTitle','off');
    plot(t_ode, rad2deg(X_ode(:,1)), '--', 'Color', [1 0.4 0.4], 'LineWidth', 2); hold on;
    plot(t_sm, theta_sm, '-', 'Color', [0.4 1 1], 'LineWidth', 2);
    yline(0, 'w--'); grid on; set(gca, 'GridColor', [0.4 0.4 0.4]);
    legend('ODE LQR (Referencia)', 'Simscape (Real)', 'Color', [0.2 0.2 0.2], 'TextColor', 'w');
    title('Comparación de Control: Segway 2025b'); xlabel('Tiempo (s)'); ylabel('\theta (Grados)');
else
    disp('⚠️ Simlog data error. El registro no se guardó en la estructura "out".');
end

function dX = ode_nl_lqr(~,X,K,V_sat,M,m,r,d,l,g,Icy,Icz,Icx,Iw,Iwz,alm,bem)
    U=max(min(-K*X,V_sat),-V_sat);
    theta=X(1); dtheta=X(2); dx=X(4); dalpha=X(6);
    VR=U(1); VL=U(2);
    M11=Icy+M*l^2; M12=M*l*cos(theta); M22=M+2*m+2*Iw/r^2;
    M33=Icx*sin(theta)^2+Icz*cos(theta)^2+2*m*(d/2)^2+2*Iw*(d/(2*r))^2+2*Iwz;
    det_ca=M11*M22-M12^2;
    tau_R=alm*VR-bem*(dx/r+d*dalpha/(2*r)-dtheta);
    tau_L=alm*VL-bem*(dx/r-d*dalpha/(2*r)-dtheta);
    F1=M*g*l*sin(theta)-(tau_R+tau_L);
    F2=M*l*dtheta^2*sin(theta)+(tau_R+tau_L)/r;
    F3=d*(tau_R-tau_L)/(2*r);
    ddtheta=(M22*F1-M12*F2)/det_ca;
    ddx=(-M12*F1+M11*F2)/det_ca;
    ddalpha=F3/M33;
    dX=[dtheta;ddtheta;dx;ddx;dalpha;ddalpha];
end