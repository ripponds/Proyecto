%% Simscape_LQR_v5_lts.m
%  TESTBENCH — Segway Gemelo Digital | Lean-to-Speed en Simscape
%  ─ K1_aug (1x4): LQR avance aumentado [theta_err, dtheta, dx, int_ev]
%  ─ K2     (1x2): LQR giro [alpha, dalpha]
%  ─ v_ref = Kv * theta_rider  dentro del LQR_Controller
%  ─ Integrador Simulink con saturacion +-int_max (anti-windup nativo)
%  ─ Base: Simscape_LQR_v41.m — cambios minimos
%  ─ MATLAB/Simulink R2025b
% =========================================================================
clear; clc; close all; bdclose all;
run('params.m');

%% ── MODELO MATEMÁTICO ────────────────────────────────────────────────────
mat_file = 'segway_modelo_resultados.mat';
if ~exist('A_num', 'var')
    if exist(mat_file, 'file')
        load(mat_file, 'A_num','B_num','A_ca','B_ca','A_giro','B_giro','polos');
        fprintf('Modelo cargado desde %s\n', mat_file);
    else
        fprintf('Corriendo ModelKane_final.m (solo esta vez)...\n');
        run('ModelKane_final.m');
    end
end
run('params.m');   % restaura variables numéricas (syms de ModelKane las sobreescribe)

%% =========================================================================
%  1. CONDICIONES INICIALES Y ESCENARIO
% =========================================================================
theta0_deg = 0;      % CI balanceo [deg] — rider sube vertical
alpha0_deg = 0;      % CI giro     [deg]
theta0     = theta0_deg*pi/180;
alpha0     = alpha0_deg*pi/180;

t_sim   = 20;
V_sat_a = 24;
V_sat_d = 24;
V_sat_f = 24;

% -- Lean-to-speed --
Kv        = 5.0;          % ganancia [m/s / rad]
th_lean   = deg2rad(5);   % inclinacion que aplica el rider [rad]
t_lean    = 3.0;          % rider se inclina [s]
t_back    = 13.0;         % rider se incorpora [s]
tau_rider = 0.3;          % constante de tiempo filtro rider [s]
v_max     = 2.0;          % velocidad maxima [m/s]
int_max   = 4.0;          % limite anti-windup integrador [m]

% -- Maniobra de giro --
% El pasajero empuja el mastil a la derecha en t=8s, vuelve recto en t=12s
alpha_ref_deg = 30;   % angulo de giro deseado [deg]  <- cambia aqui
t_avance_a    = 8.0;  % instante en que gira  [s]
t_stop_a      = 12.0; % instante en que vuelve recto [s]

fprintf('=========================================================\n');
fprintf(' TESTBENCH Segway v5 | Lean-to-Speed en Simscape\n');
fprintf(' Kv=%.1f m/s/rad  th_lean=%.0f deg  tau=%.2f s\n', Kv, rad2deg(th_lean), tau_rider);
fprintf('=========================================================\n');

%% =========================================================================
%  2. LINEALIZACION Y LQR
%  M11, M12, M22, M33, det0 ya vienen de params.m
% =========================================================================

b21 = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41 = ( M12*(2*alm) + M11*(2*alm/r)) / det0;

% K1_aug: [theta_err, dtheta, dx, int_ev]  — sin x (dependencia lineal)
A1r = [0,              1, 0;
       M22*M*g*l/det0, 0, 0;
      -M12*M*g*l/det0, 0, 0];
B1r = [0; b21; b41];
A1a = [A1r, zeros(3,1); 0 0 1 0];
B1a = [B1r; 0];

rango1 = rank(ctrb(A1a, B1a));
fprintf('[K1aug] Controlabilidad: %d/4 %s\n', rango1, iif(rango1==4,'OK','REVISAR'));

Q1a = diag([2000, 100, 50, 50/(1.5*5)]);
R1  = 1;
[K1a, ~, eigs1] = lqr(A1a, B1a, Q1a, R1);
fprintf('[K1aug] K1a = [%.3f  %.3f  %.3f  %.3f]\n', K1a);
fprintf('[K1aug] Polos: '); fprintf('%.3f  ', real(eigs1)); fprintf('\n');

% K2: giro
A2=[0 1;0 0]; B2=[0; d*alm/(r*M33)];
rango2 = rank(ctrb(A2,B2));
fprintf('[K2]    Controlabilidad: %d/2 %s\n', rango2, iif(rango2==2,'OK','REVISAR'));
Q2=diag([800,50]); R2=1;
[K2,~,eigs2] = lqr(A2,B2,Q2,R2);
fprintf('[K2]    K2 = [%.3f  %.3f]\n', K2);
fprintf('[K2]    Polos: '); fprintf('%.3f  ', real(eigs2)); fprintf('\n');

%% =========================================================================
%  3. ODE REFERENCIA — 8 estados [theta,dtheta,x,dx,alpha,dalpha,int_ev,rider]
% =========================================================================
X0_ode = zeros(8,1);
ode_fn = @(t,X) ode_lts_6(t, X, K1a, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    Kv, th_lean, t_lean, t_back, tau_rider, v_max, int_max, ...
    alpha_ref_deg*pi/180, t_avance_a, t_stop_a);

[t_ode, X_ode] = ode45(ode_fn, [0 t_sim], X0_ode, odeset('RelTol',1e-6));

% Reconstruir Va, Vd, VR, VL
n = length(t_ode);
Va_ode=zeros(n,1); Vd_ode=zeros(n,1); VR_ode=zeros(n,1); VL_ode=zeros(n,1);
for i=1:n
    th_r   = X_ode(i,8);
    int_ev = max(min(X_ode(i,7), int_max), -int_max);
    Xe1 = [X_ode(i,1)-th_r; X_ode(i,2); X_ode(i,4); int_ev];
    Xe2 = [X_ode(i,5)-alpha_ref_deg*pi/180; X_ode(i,6)];
    Va_ode(i) = max(min(-K1a*Xe1, V_sat_a), -V_sat_a);
    Vd_ode(i) = max(min(-K2*Xe2,  V_sat_d), -V_sat_d);
    VR_ode(i) = max(min(Va_ode(i)+Vd_ode(i), V_sat_f), -V_sat_f);
    VL_ode(i) = max(min(Va_ode(i)-Vd_ode(i), V_sat_f), -V_sat_f);
end

v_ref_ode = min(Kv * X_ode(:,8), v_max);
fprintf('[ODE] dx_final=%.3f m/s  v_ref_final=%.3f  theta_final=%.3f deg\n',...
    X_ode(end,4), v_ref_ode(end), rad2deg(X_ode(end,1)));

%% =========================================================================
%  4. CONSTRUIR MODELO SIMSCAPE
% =========================================================================
modelName = 'Segway_Testbench_v5';
if bdIsLoaded(modelName),            close_system(modelName,0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName); open_system(modelName);
set_param(modelName,'Solver','ode23t','StopTime',num2str(t_sim),...
    'RelTol','1e-4','AbsTol','1e-6');

% Utilidad
add_block('nesl_utility/Solver Configuration',        [modelName '/Solver_Config'],'Position',[42 105 192 135]);
add_block('sm_lib/Utilities/Mechanism Configuration', [modelName '/Mech_Config'],  'Position',[227 130 417 160]);
set_param([modelName '/Mech_Config'],'GravityVector','[0; 0; -9.81]');
add_block('sm_lib/Frames and Transforms/World Frame', [modelName '/World'],         'Position',[92 40 192 70]);
add_line(modelName,'World/RConn1','Solver_Config/RConn1');
add_line(modelName,'World/RConn1','Mech_Config/RConn1');

% Cadena cinematica: Joint_x (X mundial) + Joint_y (Y mundial) + Joint_alpha (yaw)
% ─ RT_PrismaticAlign:  +Y 90°  → Joint_x Z apunta en X mundial
% ─ RT_YAlign:          +X -90° → Joint_y Z apunta en +Y mundial
% ─ RT_Realign_X:       +X +90° → deshace RT_YAlign
% ─ RT_Realign_Y:       +Y -90° → deshace RT_PrismaticAlign (= RT_Realign original)
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[277 130 377 160]);
set_param([modelName '/RT_AxleHeight'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Z','TranslationStandardOffset',num2str(r));

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_PrismaticAlign'],'Position',[407 130 507 160]);
set_param([modelName '/RT_PrismaticAlign'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','90','TranslationMethod','None');

add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[537 120 657 180]);
set_param([modelName '/Joint_x'],'MotionActuationMode','ComputedMotion','TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_YAlign'],'Position',[687 130 787 160]);
set_param([modelName '/RT_YAlign'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90','TranslationMethod','None');

add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_y'],'Position',[817 120 937 180]);
set_param([modelName '/Joint_y'],'MotionActuationMode','ComputedMotion','TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign_X'],'Position',[967 130 1067 160]);
set_param([modelName '/RT_Realign_X'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign_Y'],'Position',[1097 130 1197 160]);
set_param([modelName '/RT_Realign_Y'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','-90','TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[1227 125 1347 185]);
set_param([modelName '/Joint_alpha'],...
    'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0','SensePosition','on','SenseVelocity','on',...
    'PositionTargetSpecify','on','PositionTargetValue',num2str(alpha0_deg),...
    'PositionTargetValueUnits','deg','PositionTargetPriority','High');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[1377 240 1497 270]);
set_param([modelName '/RT_ThetaAxis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[1547 225 1667 285]);
set_param([modelName '/Joint_theta'],...
    'PositionTargetSpecify','on','PositionTargetValue',num2str(theta0_deg),...
    'PositionTargetValueUnits','deg','PositionTargetPriority','High',...
    'TorqueActuationMode','NoTorque','MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0','SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1727 205 1847 265]);
set_param([modelName '/RT_BodyCM'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(l));
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1900 225 2020 285]);
set_param([modelName '/Body_Solid'],'BrickDimensions',mat2str([body_D,body_H,body_W]),'InertiaType','Custom','Mass',num2str(M),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Icx,Icy,Icz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.2 0.5 0.8]');

add_line(modelName,'World/RConn1',            'RT_AxleHeight/LConn1');
add_line(modelName,'RT_AxleHeight/RConn1',    'RT_PrismaticAlign/LConn1');
add_line(modelName,'RT_PrismaticAlign/RConn1','Joint_x/LConn1');
add_line(modelName,'Joint_x/RConn1',          'RT_YAlign/LConn1');
add_line(modelName,'RT_YAlign/RConn1',         'Joint_y/LConn1');
add_line(modelName,'Joint_y/RConn1',           'RT_Realign_X/LConn1');
add_line(modelName,'RT_Realign_X/RConn1',      'RT_Realign_Y/LConn1');
add_line(modelName,'RT_Realign_Y/RConn1',      'Joint_alpha/LConn1');
add_line(modelName,'Joint_alpha/RConn1',       'RT_ThetaAxis/LConn1');
add_line(modelName,'RT_ThetaAxis/RConn1',      'Joint_theta/LConn1');
add_line(modelName,'Joint_theta/RConn1',       'RT_BodyCM/LConn1');
add_line(modelName,'RT_BodyCM/RConn1',         'Body_Solid/RConn1');

% Rueda derecha
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[1377 120 1497 150]);
set_param([modelName '/RT_WheelR'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Axis'],'Position',[1547 120 1667 150]);
set_param([modelName '/RT_WheelR_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[1727 105 1847 165]);
set_param([modelName '/Joint_phi_R'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[1900 105 2020 165]);
set_param([modelName '/Wheel_R'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelR/LConn1');
add_line(modelName,'RT_WheelR/RConn1',      'RT_WheelR_Axis/LConn1');
add_line(modelName,'RT_WheelR_Axis/RConn1', 'Joint_phi_R/LConn1');
add_line(modelName,'Joint_phi_R/RConn1',    'Wheel_R/RConn1');

% Rueda izquierda
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL'],'Position',[1377 180 1497 210]);
set_param([modelName '/RT_WheelL'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL_Axis'],'Position',[1547 180 1667 210]);
set_param([modelName '/RT_WheelL_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_L'],'Position',[1727 165 1847 225]);
set_param([modelName '/Joint_phi_L'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_L'],'Position',[1900 165 2020 225]);
set_param([modelName '/Wheel_L'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelL/LConn1');
add_line(modelName,'RT_WheelL/RConn1',      'RT_WheelL_Axis/LConn1');
add_line(modelName,'RT_WheelL_Axis/RConn1', 'Joint_phi_L/LConn1');
add_line(modelName,'Joint_phi_L/RConn1',    'Wheel_L/RConn1');

% Suelo
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Ground'],'Position',[477 190 597 220]);
set_param([modelName '/RT_Ground'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Z','TranslationStandardOffset','0.025');
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Ground'],'Position',[647 190 767 220]);
set_param([modelName '/Ground'],'BrickDimensions','[30 8 0.05]','InertiaType','Custom','Mass','1e6','CenterOfMass','[0 0 0]','MomentsOfInertia','[1e6 1e6 1e6]','ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.6 0.6 0.6]');
add_line(modelName,'World/RConn1','RT_Ground/LConn1');
add_line(modelName,'RT_Ground/RConn1','Ground/RConn1');

fprintf('[Build] Estructura fisica OK\n');

%% =========================================================================
%  5. UPDATE → port handles
% =========================================================================
set_param(modelName,'SimulationCommand','update');
pH_Jt  = get_param([modelName '/Joint_theta'], 'PortHandles');
pH_Jx  = get_param([modelName '/Joint_x'],     'PortHandles');
pH_Jy  = get_param([modelName '/Joint_y'],     'PortHandles');
pH_Ja  = get_param([modelName '/Joint_alpha'],  'PortHandles');
pH_JpR = get_param([modelName '/Joint_phi_R'],  'PortHandles');

fprintf('[Ports] theta=%d | x=%d | y=%d | alpha=%d | phi_R=%d\n',...
    numel(pH_Jt.RConn), numel(pH_Jx.RConn), numel(pH_Jy.RConn), numel(pH_Ja.RConn), numel(pH_JpR.RConn));
if numel(pH_Ja.RConn) < 3
    error('[ERROR] Joint_alpha RConn=%d insuficiente', numel(pH_Ja.RConn));
end

%% =========================================================================
%  6. THETA_RIDER — filtro 1er orden como bloque Transfer Fcn
%  H(s) = 1/(tau_rider*s + 1)  con escalon th_lean en [t_lean, t_back]
% =========================================================================
% Step on
add_block('simulink/Sources/Step',[modelName '/Step_rider_on'],'Position',[200 500 270 530]);
set_param([modelName '/Step_rider_on'],'Time',num2str(t_lean),'Before','0','After',num2str(th_lean,'%.6f'));
% Step off
add_block('simulink/Sources/Step',[modelName '/Step_rider_off'],'Position',[200 550 270 580]);
set_param([modelName '/Step_rider_off'],'Time',num2str(t_back),'Before','0','After',num2str(-th_lean,'%.6f'));
% Sum rider
add_block('simulink/Math Operations/Sum',[modelName '/Sum_rider'],'Position',[310 508 350 572]);
set_param([modelName '/Sum_rider'],'Inputs','++');
pH_Srider = get_param([modelName '/Sum_rider'],'PortHandles');
add_line(modelName, get_param([modelName '/Step_rider_on'], 'PortHandles').Outport(1), pH_Srider.Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/Step_rider_off'],'PortHandles').Outport(1), pH_Srider.Inport(2),'autorouting','on');
% Filtro 1er orden
add_block('simulink/Continuous/Transfer Fcn',[modelName '/Filtro_rider'],'Position',[380 508 480 542]);
set_param([modelName '/Filtro_rider'],'Numerator','[1]','Denominator',sprintf('[%.4f 1]',tau_rider));
pH_Frider = get_param([modelName '/Filtro_rider'],'PortHandles');
add_line(modelName, pH_Srider.Outport(1), pH_Frider.Inport(1),'autorouting','on');

% alpha_ref placeholder
aref_str = num2str(alpha_ref_deg*pi/180,'%.6f');
add_block('simulink/Sources/Step',[modelName '/Step_al_on'], 'Position',[200 620 270 650]);
set_param([modelName '/Step_al_on'],'Time',num2str(t_avance_a),'Before','0','After',aref_str);
add_block('simulink/Sources/Step',[modelName '/Step_al_off'],'Position',[200 670 270 700]);
set_param([modelName '/Step_al_off'],'Time',num2str(t_stop_a),'Before','0','After',['-' aref_str]);
add_block('simulink/Math Operations/Sum',[modelName '/Sum_aref'],'Position',[310 628 350 692]);
set_param([modelName '/Sum_aref'],'Inputs','++');
pH_Saref = get_param([modelName '/Sum_aref'],'PortHandles');
add_line(modelName, get_param([modelName '/Step_al_on'], 'PortHandles').Outport(1), pH_Saref.Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/Step_al_off'],'PortHandles').Outport(1), pH_Saref.Inport(2),'autorouting','on');

fprintf('[Rider] filtro 1er orden tau=%.2fs OK\n', tau_rider);

%% =========================================================================
%  7. PS2SL — sensing 6 estados
% =========================================================================
% 6 estados de control: th,dth (Joint_theta) | x,dx (Joint_x) | al,dal (Joint_alpha)
% RConn: [1=follower, 2=pos, 3=vel]
ps_names = {'PS2SL_th','PS2SL_dth','PS2SL_x','PS2SL_dx','PS2SL_al','PS2SL_dal'};
ps_src   = {pH_Jt.RConn(2), pH_Jt.RConn(3), ...
            pH_Jx.RConn(2), pH_Jx.RConn(3), ...
            pH_Ja.RConn(2), pH_Ja.RConn(3)};
ps_pos   = {[580 390 690 420],[580 440 690 470],[580 490 690 520],...
            [580 540 690 570],[580 590 690 620],[580 640 690 670]};
for i=1:6
    add_block('nesl_utility/PS-Simulink Converter',[modelName '/' ps_names{i}],'Position',ps_pos{i});
    add_line(modelName, ps_src{i}, get_param([modelName '/' ps_names{i}],'PortHandles').LConn(1),'autorouting','on');
end

% y e dy — Joint_y, solo para logging de trayectoria
add_block('nesl_utility/PS-Simulink Converter',[modelName '/PS2SL_y'], 'Position',[580 690 690 720]);
add_block('nesl_utility/PS-Simulink Converter',[modelName '/PS2SL_dy'],'Position',[580 730 690 760]);
add_line(modelName, pH_Jy.RConn(2), get_param([modelName '/PS2SL_y'], 'PortHandles').LConn(1),'autorouting','on');
add_line(modelName, pH_Jy.RConn(3), get_param([modelName '/PS2SL_dy'],'PortHandles').LConn(1),'autorouting','on');

%% =========================================================================
%  8. INTEGRADOR DE VELOCIDAD — ∫(dx - v_ref)dt con anti-windup nativo
%  v_ref = Kv * theta_rider (calculado dentro del Sum antes del integrador)
% =========================================================================
% Gain Kv: theta_rider → v_ref
add_block('simulink/Math Operations/Gain',[modelName '/Gain_Kv'],'Position',[510 505 570 535]);
set_param([modelName '/Gain_Kv'],'Gain',num2str(Kv));
pH_GKv = get_param([modelName '/Gain_Kv'],'PortHandles');
add_line(modelName, pH_Frider.Outport(1), pH_GKv.Inport(1),'autorouting','on');

% Saturacion v_ref <= v_max
add_block('simulink/Discontinuities/Saturation',[modelName '/Sat_vref'],'Position',[590 505 650 535]);
set_param([modelName '/Sat_vref'],'UpperLimit',num2str(v_max),'LowerLimit',num2str(-v_max));
pH_Svref = get_param([modelName '/Sat_vref'],'PortHandles');
add_line(modelName, pH_GKv.Outport(1), pH_Svref.Inport(1),'autorouting','on');

% Error velocidad: dx - v_ref
add_block('simulink/Math Operations/Sum',[modelName '/Sum_ev'],'Position',[680 505 720 570]);
set_param([modelName '/Sum_ev'],'Inputs','+-');
pH_Sev = get_param([modelName '/Sum_ev'],'PortHandles');
add_line(modelName, get_param([modelName '/PS2SL_dx'],'PortHandles').Outport(1), pH_Sev.Inport(1),'autorouting','on');
add_line(modelName, pH_Svref.Outport(1), pH_Sev.Inport(2),'autorouting','on');

% Integrador con saturacion = anti-windup nativo de Simulink
add_block('simulink/Continuous/Integrator',[modelName '/Int_ev'],'Position',[750 505 800 545]);
set_param([modelName '/Int_ev'],...
    'LimitOutput','on',...
    'UpperSaturationLimit', num2str( int_max),...
    'LowerSaturationLimit', num2str(-int_max),...
    'InitialCondition','0');
pH_Iev = get_param([modelName '/Int_ev'],'PortHandles');
add_line(modelName, pH_Sev.Outport(1), pH_Iev.Inport(1),'autorouting','on');

fprintf('[Int_ev] Integrador dx-v_ref con anti-windup +-%.1f OK\n', int_max);

%% =========================================================================
%  9. LQR_CONTROLLER — MATLAB Function
%  Entradas (9): theta, dtheta, x, dx, alpha, dalpha, theta_rider, alpha_ref, int_ev
%  Salidas  (4): VR, VL, Va, Vd
% =========================================================================
add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/LQR_Controller'],'Position',[860 380 1020 760]);

func_str = sprintf([...
'function [VR, VL, Va, Vd] = LQR_Controller(theta, dtheta, ~, dx, alpha, dalpha, theta_rider, alpha_ref, int_ev)\n'...
'%% K1_aug avance [theta_err, dtheta, dx, int_ev]\n'...
'%% K2     giro   [alpha_err, dalpha]\n'...
'K1a = %s;\n'...
'K2  = %s;\n'...
'V_sat_a = %.4f; V_sat_d = %.4f; V_sat_f = %.4f;\n'...
'\n'...
'Xe1 = [theta - theta_rider; dtheta; dx; int_ev];\n'...
'Va  = max(min(-K1a * Xe1, V_sat_a), -V_sat_a);\n'...
'\n'...
'Xe2 = [alpha - alpha_ref; dalpha];\n'...
'Vd  = max(min(-K2 * Xe2, V_sat_d), -V_sat_d);\n'...
'\n'...
'VR = max(min(Va + Vd, V_sat_f), -V_sat_f);\n'...
'VL = max(min(Va - Vd, V_sat_f), -V_sat_f);\n'...
'end\n'], mat2str(K1a,6), mat2str(K2,6), V_sat_a, V_sat_d, V_sat_f);

rt = sfroot();
ctrl_chart = rt.find('-isa','Stateflow.EMChart','Path',[modelName '/LQR_Controller']);
ctrl_chart.Script = func_str;

pH_LQR = get_param([modelName '/LQR_Controller'],'PortHandles');

% Conectar 9 entradas
for i=1:6
    add_line(modelName, get_param([modelName '/' ps_names{i}],'PortHandles').Outport(1), pH_LQR.Inport(i),'autorouting','on');
end
add_line(modelName, pH_Frider.Outport(1), pH_LQR.Inport(7),'autorouting','on');  % theta_rider filtrado
add_line(modelName, pH_Saref.Outport(1),  pH_LQR.Inport(8),'autorouting','on');  % alpha_ref
add_line(modelName, pH_Iev.Outport(1),    pH_LQR.Inport(9),'autorouting','on');  % int_ev

fprintf('[LQR_Controller] 9 entradas conectadas OK\n');

%% =========================================================================
%  10. TORQUES A JOINTS — identico a v4.1
% =========================================================================
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauR'],'Position',[1060 400 1110 430]);
set_param([modelName '/Gain_tauR'],'Gain',num2str(alm));
add_line(modelName, pH_LQR.Outport(1), get_param([modelName '/Gain_tauR'],'PortHandles').Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauR'],'Position',[1130 400 1210 430]);
add_line(modelName, get_param([modelName '/Gain_tauR'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_tauR'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauR/RConn1','Joint_phi_R/LConn2');

add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauL'],'Position',[1060 450 1110 480]);
set_param([modelName '/Gain_tauL'],'Gain',num2str(alm));
add_line(modelName, pH_LQR.Outport(2), get_param([modelName '/Gain_tauL'],'PortHandles').Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauL'],'Position',[1130 450 1210 480]);
add_line(modelName, get_param([modelName '/Gain_tauL'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_tauL'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauL/RConn1','Joint_phi_L/LConn2');

% Fuerza de avance proyectada en X e Y del mundo segun heading alpha
% Fx = (2*alm/r)*Va*cos(alpha) → Joint_x
% Fy = (2*alm/r)*Va*sin(alpha) → Joint_y
% Tau_al = (d*alm/r)*Vd       → Joint_alpha

add_block('simulink/Math Operations/Gain',[modelName '/Gain_F'],'Position',[1060 500 1110 530]);
set_param([modelName '/Gain_F'],'Gain',num2str(2*alm/r));
add_line(modelName, pH_LQR.Outport(3), get_param([modelName '/Gain_F'],'PortHandles').Inport(1),'autorouting','on');

add_block('simulink/Math Operations/Trigonometric Function',[modelName '/cos_al'],'Position',[1060 545 1110 575]);
set_param([modelName '/cos_al'],'Operator','cos');
add_line(modelName, get_param([modelName '/PS2SL_al'],'PortHandles').Outport(1), get_param([modelName '/cos_al'],'PortHandles').Inport(1),'autorouting','on');

add_block('simulink/Math Operations/Trigonometric Function',[modelName '/sin_al'],'Position',[1060 590 1110 620]);
set_param([modelName '/sin_al'],'Operator','sin');
add_line(modelName, get_param([modelName '/PS2SL_al'],'PortHandles').Outport(1), get_param([modelName '/sin_al'],'PortHandles').Inport(1),'autorouting','on');

% Negar sin(alpha): Joint_y eje Z apunta en -Y mundial por la cadena RT
add_block('simulink/Math Operations/Gain',[modelName '/Neg_sin'],'Position',[1060 625 1110 655]);
set_param([modelName '/Neg_sin'],'Gain','-1');
add_line(modelName, get_param([modelName '/sin_al'],'PortHandles').Outport(1), get_param([modelName '/Neg_sin'],'PortHandles').Inport(1),'autorouting','on');

% Fx → Joint_x
add_block('simulink/Math Operations/Product',[modelName '/Prod_Fx'],'Position',[1135 505 1180 545]);
add_line(modelName, get_param([modelName '/Gain_F'],  'PortHandles').Outport(1), get_param([modelName '/Prod_Fx'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/cos_al'],  'PortHandles').Outport(1), get_param([modelName '/Prod_Fx'],'PortHandles').Inport(2),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fx'],'Position',[1200 508 1280 542]);
add_line(modelName, get_param([modelName '/Prod_Fx'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_Fx'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_Fx/RConn1','Joint_x/LConn2');

% Fy → Joint_y  (usa -sin para compensar orientacion del joint)
add_block('simulink/Math Operations/Product',[modelName '/Prod_Fy'],'Position',[1135 555 1180 595]);
add_line(modelName, get_param([modelName '/Gain_F'],   'PortHandles').Outport(1), get_param([modelName '/Prod_Fy'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/Neg_sin'],  'PortHandles').Outport(1), get_param([modelName '/Prod_Fy'],'PortHandles').Inport(2),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fy'],'Position',[1200 558 1280 592]);
add_line(modelName, get_param([modelName '/Prod_Fy'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_Fy'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_Fy/RConn1','Joint_y/LConn2');

% Tau_alpha → Joint_alpha
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tau_al'],'Position',[1060 630 1110 660]);
set_param([modelName '/Gain_tau_al'],'Gain',num2str(d*alm/r));
add_line(modelName, pH_LQR.Outport(4), get_param([modelName '/Gain_tau_al'],'PortHandles').Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tau_al'],'Position',[1135 633 1215 657]);
add_line(modelName, get_param([modelName '/Gain_tau_al'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_tau_al'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tau_al/RConn1','Joint_alpha/LConn2');

fprintf('[Forces] Fx=F*cos(al)→Joint_x | Fy=F*sin(al)→Joint_y | Tau→Joint_alpha OK\n');

%% =========================================================================
%  11. LOGGING
% =========================================================================
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VRVL'],'Position',[1060 620 1090 660]);
set_param([modelName '/Mux_VRVL'],'Inputs','2');
pH_MVRVL = get_param([modelName '/Mux_VRVL'],'PortHandles');
add_line(modelName, pH_LQR.Outport(1), pH_MVRVL.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(2), pH_MVRVL.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VRVL'],'Position',[1110 625 1200 645]);
set_param([modelName '/ToWS_VRVL'],'VariableName','VRVL_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVRVL.Outport(1), get_param([modelName '/ToWS_VRVL'],'PortHandles').Inport(1),'autorouting','on');

add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VaVd'],'Position',[1060 670 1090 710]);
set_param([modelName '/Mux_VaVd'],'Inputs','2');
pH_MVaVd = get_param([modelName '/Mux_VaVd'],'PortHandles');
add_line(modelName, pH_LQR.Outport(3), pH_MVaVd.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(4), pH_MVaVd.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VaVd'],'Position',[1110 675 1200 695]);
set_param([modelName '/ToWS_VaVd'],'VariableName','VaVd_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVaVd.Outport(1), get_param([modelName '/ToWS_VaVd'],'PortHandles').Inport(1),'autorouting','on');

% int_ev log
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_intev'],'Position',[830 555 920 575]);
set_param([modelName '/ToWS_intev'],'VariableName','intev_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_Iev.Outport(1), get_param([modelName '/ToWS_intev'],'PortHandles').Inport(1),'autorouting','on');

% ToWS para los 6 estados fisicos (evita paths fragiles de simlog)
state_ws_names  = {'th_log','dth_log','x_log','dx_log','al_log','dal_log'};
state_blk_names = {'ToWS_th','ToWS_dth','ToWS_x','ToWS_dx','ToWS_al','ToWS_dal'};
state_ws_pos    = {[580 700 680 720],[580 730 680 750],[580 760 680 780],...
                   [580 790 680 810],[580 820 680 840],[580 850 680 870]};
for i=1:6
    blk = [modelName '/' state_blk_names{i}];
    add_block('simulink/Sinks/To Workspace', blk, 'Position', state_ws_pos{i});
    set_param(blk,'VariableName',state_ws_names{i},'SaveFormat','Array','SampleTime','-1');
    add_line(modelName, get_param([modelName '/' ps_names{i}],'PortHandles').Outport(1), ...
             get_param(blk,'PortHandles').Inport(1),'autorouting','on');
end

% y e dy del Planar Joint
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_y'], 'Position',[580 905 680 925]);
set_param([modelName '/ToWS_y'], 'VariableName','y_log', 'SaveFormat','Array','SampleTime','-1');
add_line(modelName, get_param([modelName '/PS2SL_y'], 'PortHandles').Outport(1), get_param([modelName '/ToWS_y'], 'PortHandles').Inport(1),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_dy'],'Position',[580 935 680 955]);
set_param([modelName '/ToWS_dy'],'VariableName','dy_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, get_param([modelName '/PS2SL_dy'],'PortHandles').Outport(1), get_param([modelName '/ToWS_dy'],'PortHandles').Inport(1),'autorouting','on');

fprintf('[Logging] VRVL_log | VaVd_log | intev_log | 6 estados | y_log | dy_log OK\n');

%% =========================================================================
%  12. AUTO-LAYOUT + GUARDAR + SIMULAR
% =========================================================================
Simulink.BlockDiagram.arrangeSystem(modelName);
save_system(modelName,[modelName '.slx']);
fprintf('[Sim] Iniciando simulacion de %g s...\n', t_sim);
try
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimscapeLogType','all','SimscapeLogName','simlog');
    out   = sim(simIn);
    fprintf('[Sim] OK  t_final=%.3f s\n', out.tout(end));
catch e
    fprintf('[Sim] FALLO: %s\n', e.message);
    return
end

%% =========================================================================
%  13. DIAGNOSTICO + GRAFICAS
% =========================================================================
try
    % ── Extraer desde ToWS (robusto, sin dependencia de simlog paths) ──
    t_sm     = out.tout;
    theta_sm = out.get('th_log');
    dtheta_sm= out.get('dth_log');
    x_sm     = out.get('x_log');
    dx_sm    = out.get('dx_log');
    alpha_sm = out.get('al_log');
    dalpha_sm= out.get('dal_log');
    t_U      = out.tout;
    intev_sm = out.get('intev_log');

    % ── Reconstruir Vd_ode (faltaba en loop anterior) ──────────────────
    Vd_ode_local = zeros(length(t_ode),1);
    for ii = 1:length(t_ode)
        tref_al = alpha_ref_deg*pi/180 * (t_ode(ii)>=t_avance_a && t_ode(ii)<t_stop_a);
        Xe2 = [X_ode(ii,5)-tref_al; X_ode(ii,6)];
        Vd_ode_local(ii) = max(min(-K2*Xe2, V_sat_d), -V_sat_d);
    end

    % ── Referencias vectorizadas ────────────────────────────────────────
    theta_rider_t = th_lean * double(t_ode >= t_lean & t_ode < t_back);
    v_ref_t       = min(Kv * X_ode(:,8), v_max);
    alpha_ref_t   = alpha_ref_deg*pi/180 * double(t_ode >= t_avance_a & t_ode < t_stop_a);

    fprintf('\n[Diagnostico]\n');
    fprintf('  theta_final  = %.3f deg  %s\n', rad2deg(theta_sm(end)), iif(abs(rad2deg(theta_sm(end)))<2,'OK','REVISAR'));
    fprintf('  alpha_final  = %.3f deg  %s\n', rad2deg(alpha_sm(end)), iif(abs(rad2deg(alpha_sm(end)))<5,'OK','REVISAR'));
    fprintf('  dx_final     = %.3f m/s\n', dx_sm(end));
    fprintf('  x_final      = %.3f m\n',  x_sm(end));

    % ── Colores ─────────────────────────────────────────────────────────
    BG    = [0.10 0.10 0.12];
    AX    = [0.16 0.16 0.19];
    GR    = [0.28 0.28 0.32];
    C_ode = [1.00 0.40 0.40];   % rojo claro  — ODE
    C_sm  = [0.35 0.75 1.00];   % azul claro  — Simscape
    C_ref = [0.65 0.65 0.65];   % gris        — referencia rider
    C_vrf = [1.00 0.75 0.20];   % amarillo    — v_ref
    C_arf = [0.90 0.55 0.10];   % naranja     — alpha_ref
    C_ev  = [0.30 0.90 0.60];   % verde menta — eventos lean
    C_gi  = [0.80 0.40 1.00];   % violeta     — eventos giro
    TXT   = [0.92 0.92 0.92];

    set(0,'DefaultFigureColor',    BG);
    set(0,'DefaultAxesColor',      AX);
    set(0,'DefaultAxesXColor',     TXT);
    set(0,'DefaultAxesYColor',     TXT);
    set(0,'DefaultAxesGridColor',  GR);
    set(0,'DefaultTextColor',      TXT);

    figure('Name','Segway v5 — Lean-to-Speed',...
           'Color',BG,'Position',[40 30 1100 920]);

    % ── 1. Angulo de inclinacion theta ──────────────────────────────────
    ax1 = subplot(4,1,1);
    set(ax1,'Color',AX,'XColor',TXT,'YColor',TXT,'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',9);
    hold on; grid on;
    plot(t_ode, rad2deg(X_ode(:,1)),   '--','Color',C_ode,'LineWidth',1.8,'DisplayName','\theta  ODE');
    plot(t_sm,  rad2deg(theta_sm),          'Color',C_sm, 'LineWidth',2.2,'DisplayName','\theta  Simscape');
    plot(t_ode, rad2deg(theta_rider_t), ':','Color',C_ref,'LineWidth',1.5,'DisplayName','\theta_{rider}');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,    'Color',C_ev,'LineWidth',1,'LineStyle','--','Label','Inclina',   'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    xline(t_back,    'Color',C_ev,'LineWidth',1,'LineStyle','--','Label','Incorpora', 'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    xline(t_avance_a,'Color',C_gi,'LineWidth',1,'LineStyle','--','Label','Gira',      'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    xline(t_stop_a,  'Color',C_gi,'LineWidth',1,'LineStyle','--','Label','Recto',     'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    ylabel('\theta  [deg]','Color',TXT);
    title('\theta — \acute{A}ngulo de inclinaci\acute{o}n (pitch)','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',8);
    set(lg,'Color',[0.18 0.18 0.22],'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 2. Velocidad lineal dx ──────────────────────────────────────────
    ax2 = subplot(4,1,2);
    set(ax2,'Color',AX,'XColor',TXT,'YColor',TXT,'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',9);
    hold on; grid on;
    plot(t_ode, X_ode(:,4), '--','Color',C_ode,'LineWidth',1.8,'DisplayName','dx  ODE');
    plot(t_sm,  dx_sm,           'Color',C_sm, 'LineWidth',2.2,'DisplayName','dx  Simscape');
    plot(t_ode, v_ref_t,    ':',  'Color',C_vrf,'LineWidth',2.0,'DisplayName','v_{ref} = K_v\cdot\theta_{rider}');
    yline(v_max, 'Color',C_vrf,'LineWidth',0.8,'LineStyle','--',...
          'Label',sprintf('v_{max} = %.1f m/s',v_max),'HandleVisibility','off');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'Color',C_ev,'LineWidth',1,'LineStyle','--','HandleVisibility','off');
    xline(t_back,'Color',C_ev,'LineWidth',1,'LineStyle','--','HandleVisibility','off');
    ylabel('dx  [m/s]','Color',TXT);
    title('Velocidad lineal de avance','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',8);
    set(lg,'Color',[0.18 0.18 0.22],'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 3. Angulo de giro alpha ─────────────────────────────────────────
    ax3 = subplot(4,1,3);
    set(ax3,'Color',AX,'XColor',TXT,'YColor',TXT,'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',9);
    hold on; grid on;
    plot(t_ode, rad2deg(X_ode(:,5)), '--','Color',C_ode,'LineWidth',1.8,'DisplayName','\alpha  ODE');
    plot(t_sm,  rad2deg(alpha_sm),        'Color',C_sm, 'LineWidth',2.2,'DisplayName','\alpha  Simscape');
    plot(t_ode, rad2deg(alpha_ref_t), ':','Color',C_arf,'LineWidth',2.0,'DisplayName','\alpha_{ref}');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_avance_a,'Color',C_gi,'LineWidth',1,'LineStyle','--','Label','Gira', 'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    xline(t_stop_a,  'Color',C_gi,'LineWidth',1,'LineStyle','--','Label','Recto','LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    ylabel('\alpha  [deg]','Color',TXT);
    title('\alpha — \acute{A}ngulo de giro (yaw)','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northeast','FontSize',8);
    set(lg,'Color',[0.18 0.18 0.22],'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    % ── 4. Posicion x ───────────────────────────────────────────────────
    ax4 = subplot(4,1,4);
    set(ax4,'Color',AX,'XColor',TXT,'YColor',TXT,'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',9);
    hold on; grid on;
    plot(t_ode, X_ode(:,3), '--','Color',C_ode,'LineWidth',1.8,'DisplayName','x  ODE');
    plot(t_sm,  x_sm,            'Color',C_sm, 'LineWidth',2.2,'DisplayName','x  Simscape');
    yline(0,'Color',TXT,'LineWidth',0.5,'HandleVisibility','off');
    xline(t_lean,'Color',C_ev,'LineWidth',1,'LineStyle','--','Label','Inclina', 'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    xline(t_back,'Color',C_ev,'LineWidth',1,'LineStyle','--','Label','Incorpora','LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom','HandleVisibility','off');
    xlabel('t  [s]','Color',TXT);
    ylabel('x  [m]','Color',TXT);
    title('Posici\acute{o}n longitudinal','Color',TXT,'FontWeight','bold');
    lg=legend('Location','northwest','FontSize',8);
    set(lg,'Color',[0.18 0.18 0.22],'TextColor',TXT,'EdgeColor',GR);
    xlim([0 t_sim]);

    linkaxes([ax1 ax2 ax3 ax4],'x');

    sgtitle(sprintf('Segway v5  |  Lean-to-Speed  |  K_v = %.1f  |  \\theta_{lean} = %.0f%s  |  \\alpha_{ref} = %.0f%s',...
        Kv, rad2deg(th_lean), char(176), alpha_ref_deg, char(176)),...
        'FontSize',12,'FontWeight','bold','Color',TXT);

    % ── FIGURA 2: Trayectoria X vs Y en el plano ────────────────────────
    % ODE: integrar dx*cos(al) y dx*sin(al) (y no es estado del ODE)
    xw_ode = cumtrapz(t_ode, X_ode(:,4) .* cos(X_ode(:,5)));
    yw_ode = cumtrapz(t_ode, X_ode(:,4) .* sin(X_ode(:,5)));

    % Simscape: Px e Py del Planar Joint — posicion REAL en el mundo
    y_sm   = out.get('y_log');
    xw_sm  = x_sm;    % Px del Planar Joint = x en el mundo
    yw_sm  = y_sm;    % Py del Planar Joint = y en el mundo

    % Instantes clave en t_sm (para marcar en la trayectoria)
    [~, i_lean]  = min(abs(t_sm - t_lean));
    [~, i_back]  = min(abs(t_sm - t_back));
    [~, i_giro]  = min(abs(t_sm - t_avance_a));
    [~, i_recto] = min(abs(t_sm - t_stop_a));

    figure('Name','Segway v5 — Trayectoria X-Y',...
           'Color',BG,'Position',[160 80 700 640]);

    ax_traj = axes('Color',AX,'XColor',TXT,'YColor',TXT,...
                   'GridColor',GR,'GridAlpha',0.4,'Box','on','FontSize',10);
    hold on; grid on; axis equal;

    % ODE
    plot(xw_ode, yw_ode, '--','Color',C_ode,'LineWidth',1.8,'DisplayName','ODE');

    % Simscape — linea con gradiente temporal usando colormap
    % Dibujar en segmentos coloreados segun tiempo
    n_sm = length(t_sm);
    cmap = cool(n_sm);
    for k = 1:n_sm-1
        plot([xw_sm(k) xw_sm(k+1)], [yw_sm(k) yw_sm(k+1)],...
             'Color',cmap(k,:),'LineWidth',2.5,'HandleVisibility','off');
    end
    % Entrada ficticia para la leyenda del gradiente
    patch(NaN,NaN,[0.35 0.75 1.00],'DisplayName','Simscape (cool = tiempo)');

    % Marcadores de eventos
    plot(xw_sm(1),      yw_sm(1),      'o','Color',TXT,  'MarkerSize',8,'MarkerFaceColor',TXT,  'DisplayName','Inicio');
    plot(xw_sm(i_lean), yw_sm(i_lean), '^','Color',C_ev, 'MarkerSize',9,'MarkerFaceColor',C_ev, 'DisplayName','Inclina');
    plot(xw_sm(i_giro), yw_sm(i_giro), 's','Color',C_gi, 'MarkerSize',9,'MarkerFaceColor',C_gi, 'DisplayName','Gira');
    plot(xw_sm(i_recto),yw_sm(i_recto),'s','Color',C_arf,'MarkerSize',9,'MarkerFaceColor',C_arf,'DisplayName','Recto');
    plot(xw_sm(i_back), yw_sm(i_back), 'v','Color',C_ev, 'MarkerSize',9,'MarkerFaceColor',C_ev, 'DisplayName','Incorpora');
    plot(xw_sm(end),    yw_sm(end),    'p','Color',C_vrf,'MarkerSize',11,'MarkerFaceColor',C_vrf,'DisplayName','Final');

    % Barra de color = tiempo
    cb = colorbar(ax_traj); colormap(ax_traj, cool);
    caxis([0 t_sim]);
    cb.Label.String = 't  [s]';
    cb.Color = TXT;
    cb.Label.Color = TXT;

    xlabel('x_{mundo}  [m]','Color',TXT,'FontSize',10);
    ylabel('y_{mundo}  [m]','Color',TXT,'FontSize',10);
    title('Trayectoria en el plano  (x_w  vs  y_w)','Color',TXT,'FontSize',12,'FontWeight','bold');

    lg2 = legend('Location','best','FontSize',9);
    set(lg2,'Color',[0.18 0.18 0.22],'TextColor',TXT,'EdgeColor',GR);

    sgtitle(sprintf('Segway v5  |  \\alpha_{ref} = %.0f%s  |  K_v = %.1f',...
        alpha_ref_deg, char(176), Kv),...
        'FontSize',11,'FontWeight','bold','Color',TXT);

    % ── GUARDAR FIGURAS ──────────────────────────────────────────────────
    fig_dir = fullfile(fileparts(mfilename('fullpath')), 'debug');
    exportgraphics(figure(1), fullfile(fig_dir, 'debug_estados.png'),  'Resolution', 150);
    exportgraphics(figure(2), fullfile(fig_dir, 'debug_trayectoria.png'), 'Resolution', 150);
    fprintf('[Figuras] debug_estados.png y debug_trayectoria.png guardadas\n');

catch e
    fprintf('[Graficas] FALLO: %s\n', e.message);
end

fprintf('\n=========================================================\n');
fprintf(' LISTO v5\n');
fprintf(' Ajustar: Kv | th_lean | t_lean | t_back | tau_rider | v_max\n');
fprintf('=========================================================\n');

%% =========================================================================
%  FUNCIONES LOCALES
% =========================================================================
function s = iif(c,a,b); if c, s=a; else, s=b; end; end

function pj_set_param(blk, nm, vals)
% Intenta setear 'nm' en 'blk' con cada valor de 'vals' hasta que uno funcione
    for v = vals
        try
            set_param(blk, nm, v{1});
            fprintf('  SET  %-40s = %s\n', nm, v{1});
            return;
        catch
        end
    end
    fprintf('  FAIL %-40s (ningún valor funcionó: %s)\n', nm, strjoin(vals,', '));
end

function ref = get_ref(t, val, t_on, t_off)
    if t>=t_on && t<t_off, ref=val; else, ref=0; end
end

function dX = ode_lts_6(t, X, K1a, K2, V_sat_a, V_sat_d, V_sat_f, ...
    M, m, r, d, l, g, Icy, Icz, Iw, Iwz, alm, ...
    Kv, th_lean, t_lean, t_back, tau_rider, v_max, int_max, ...
    alpha_ref_rad, t_avance_a, t_stop_a)

    % X = [theta,dtheta,x,dx,alpha,dalpha,int_ev,rider_filtrado]
    th_target = th_lean * (t>=t_lean && t<t_back);
    dX8 = (th_target - X(8)) / tau_rider;   % filtro 1er orden

    th_r   = X(8);
    vr     = min(Kv*th_r, v_max);
    int_ev = max(min(X(7), int_max), -int_max);  % anti-windup

    tref_al = get_ref(t, alpha_ref_rad, t_avance_a, t_stop_a);

    Xe1 = [X(1)-th_r; X(2); X(4); int_ev];
    Va  = max(min(-K1a*Xe1, V_sat_a), -V_sat_a);

    Xe2 = [X(5)-tref_al; X(6)];
    Vd  = max(min(-K2*Xe2,  V_sat_d), -V_sat_d);

    VR = max(min(Va+Vd, V_sat_f), -V_sat_f);
    VL = max(min(Va-Vd, V_sat_f), -V_sat_f);
    tau_R = alm*VR; tau_L = alm*VL;

    theta=X(1); dtheta=X(2);
    M11  = Icy + M*l^2;
    M12  = M*l*cos(theta);
    M22  = M + 2*m + 2*Iw/r^2;
    M33  = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz;
    detM = M11*M22 - M12^2;

    F1 = M*g*l*sin(theta) - (tau_R+tau_L);
    F2 = M*l*dtheta^2*sin(theta) + (tau_R+tau_L)/r;
    F3 = d*(tau_R-tau_L)/(2*r);

    ddtheta = ( M22*F1 - M12*F2) / detM;
    ddx     = (-M12*F1 + M11*F2) / detM;
    ddalpha = F3 / M33;

    % anti-windup condicional en derivada
    ev = X(4) - vr;
    if (X(7)>= int_max && ev>0) || (X(7)<=-int_max && ev<0)
        d_int = 0;
    else
        d_int = ev;
    end

    dX = [dtheta; ddtheta; X(4); ddx; X(6); ddalpha; d_int; dX8];
end