%% build_v4.m
%  CONSTRUCCION DEL MODELO SIMSCAPE — Segway Testbench v4
%  ─ Ejecutar UNA vez para generar Segway_Testbench_v4.slx
%  ─ Despues reorganizar bloques manualmente y guardar con File>Save
%  ─ Para simular, usar run_v4.m (no reconstruye el modelo)
%  ─ NOTA: Si Segway_Testbench_v4.slx ya existe, este script lo sobreescribe.
%          Cuando tengas la disposicion final que quieres conservar,
%          NO vuelvas a correr build_v4.m sobre ese archivo.
% =========================================================================

%% GUARDIA — protege la disposicion manual del modelo
if exist('Segway_Testbench_v4.slx','file')
    resp = input([...
        '\n[build_v4] Segway_Testbench_v4.slx YA EXISTE.\n' ...
        '           Si continuas, se borrara toda la disposicion\n' ...
        '           de bloques que organizaste manualmente.\n' ...
        '           Escribe  SI  para confirmar, cualquier otra cosa cancela: '], 's');
    if ~strcmpi(strtrim(resp), 'SI')
        fprintf('[build_v4] Cancelado — modelo existente conservado.\n');
        return
    end
    fprintf('[build_v4] Confirmado — reconstruyendo modelo...\n');
end

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
theta0_deg = 5;      % perturbacion balanceo [deg]
alpha0_deg = 10;     % perturbacion giro     [deg]  <- valida K2
theta0     = theta0_deg * pi/180;
alpha0     = alpha0_deg * pi/180;

% -- Simulacion --
t_sim   = 15;    % duracion [s]
V_sat_a = 24;    % saturacion Va (avance)  [V]
V_sat_d = 24;    % saturacion Vd (giro)    [V]
V_sat_f = 24;    % saturacion final VR,VL  [V]

% -- RUTINA AVANCE: theta_ref --
theta_ref_deg = 0;
t_avance      = 999;
t_stop        = 1000;

% -- RUTINA GIRO: alpha_ref --
alpha_ref_deg = 0;
t_avance_a    = 999;
t_stop_a      = 1000;

fprintf('=========================================================\n');
fprintf(' BUILD Segway v4 | 6 Estados | K1+K2 desacoplados\n');
fprintf(' theta0=%.1f  alpha0=%.1f  t_sim=%gs\n', theta0_deg, alpha0_deg, t_sim);
fprintf('=========================================================\n');

% Advertencia: alpha0 pequeno -> Vd imperceptible en animacion 3D
if abs(alpha0_deg) < 5
    fprintf('[ADVERTENCIA] alpha0=%.1f deg es pequeno — Vd inicial sera %.2f V.\n', ...
        alpha0_deg, abs(alpha0_deg)*pi/180 * 28.28);
    fprintf('              El giro puede ser imperceptible visualmente.\n');
    fprintf('              Usa alpha0_deg >= 5 para validar K2 visualmente.\n');
end

%% =========================================================================
%  2. LINEALIZACION Y LQR
%  K1: avance  — [theta, dtheta, x, dx]   entrada Va
%  K2: giro    — [alpha, dalpha]           entrada Vd
%  Sin back-EMF: tau = alm*V
%  M11, M12, M22, M33, det0 ya vienen de params.m
% =========================================================================

% ── K1: subsistema avance ─────────────────────────────────────────────
A1 = zeros(4);
A1(1,2) = 1;
A1(2,1) = M22*M*g*l / det0;
A1(3,4) = 1;
A1(4,1) = -M12*M*g*l / det0;

b21_a = (M22*(-2*alm) - M12*(2*alm/r)) / det0;
b41_a = ( M12*(2*alm) + M11*(2*alm/r)) / det0;
B1    = [0; b21_a; 0; b41_a];

rango1 = rank(ctrb(A1, B1));
fprintf('[K1] Controlabilidad: %d/4', rango1);
if rango1==4, fprintf(' OK\n'); else, fprintf(' REVISAR\n'); end

Q1 = diag([2000, 100, 50, 50]);
R1 = 1;
[K1, ~, eigs1] = lqr(A1, B1, Q1, R1);
fprintf('[K1] K1 = [%.3f  %.3f  %.3f  %.3f]\n', K1);
fprintf('[K1] Polos: '); fprintf('%.3f  ', real(eigs1)); fprintf('\n');

% ── K2: subsistema giro ───────────────────────────────────────────────
A2 = [0 1; 0 0];
B2 = [0; d*alm/(r*M33)];

rango2 = rank(ctrb(A2, B2));
fprintf('[K2] Controlabilidad: %d/2', rango2);
if rango2==2, fprintf(' OK\n'); else, fprintf(' REVISAR\n'); end

Q2 = diag([800, 50]);
R2 = 1;
[K2, ~, eigs2] = lqr(A2, B2, Q2, R2);
fprintf('[K2] K2 = [%.3f  %.3f]\n', K2);
fprintf('[K2] Polos: '); fprintf('%.3f  ', real(eigs2)); fprintf('\n');

%% =========================================================================
%  4. CONSTRUIR MODELO SIMSCAPE
% =========================================================================
modelName = 'Segway_Testbench_v4';
if bdIsLoaded(modelName),            close_system(modelName, 0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName);
open_system(modelName);
set_param(modelName,'Solver','ode23t','StopTime',num2str(t_sim),...
    'RelTol','1e-4','AbsTol','1e-6');

add_block('nesl_utility/Solver Configuration',        [modelName '/Solver_Config'],'Position',[30 30 180 60]);
add_block('sm_lib/Utilities/Mechanism Configuration', [modelName '/Mech_Config'],  'Position',[30 90 220 120]);
set_param([modelName '/Mech_Config'],'GravityVector','[0; 0; -9.81]');
add_block('sm_lib/Frames and Transforms/World Frame', [modelName '/World'],         'Position',[30 160 130 190]);
add_line(modelName,'World/RConn1','Solver_Config/RConn1');
add_line(modelName,'World/RConn1','Mech_Config/RConn1');

% Cadena cinematica
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[220 160 340 190]);
set_param([modelName '/RT_AxleHeight'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Z','TranslationStandardOffset',num2str(r));

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_PrismaticAlign'],'Position',[400 160 520 190]);
set_param([modelName '/RT_PrismaticAlign'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','90','TranslationMethod','None');

add_block('sm_lib/Joints/Prismatic Joint',[modelName '/Joint_x'],'Position',[580 145 700 205]);
set_param([modelName '/Joint_x'],'MotionActuationMode','ComputedMotion','TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Realign'],'Position',[740 145 860 185]);
set_param([modelName '/RT_Realign'],'RotationMethod','StandardAxis','RotationStandardAxis','+Y','RotationAngle','-90','TranslationMethod','None');

% Joint_alpha: ahora InputTorque + Sensing + CI alpha0
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_alpha'],'Position',[920 145 1040 205]);
set_param([modelName '/Joint_alpha'],...
    'TorqueActuationMode','InputTorque',...
    'MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0',...
    'SensePosition','on','SenseVelocity','on',...
    'PositionTargetSpecify','on',...
    'PositionTargetValue',num2str(alpha0_deg),...
    'PositionTargetValueUnits','deg',...
    'PositionTargetPriority','High');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[960 60 1080 90]);
set_param([modelName '/RT_ThetaAxis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90','TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[1140 45 1260 105]);
set_param([modelName '/Joint_theta'],...
    'PositionTargetSpecify','on','PositionTargetValue',num2str(theta0_deg),...
    'PositionTargetValueUnits','deg','PositionTargetPriority','High',...
    'TorqueActuationMode','NoTorque','MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[1320 45 1440 105]);
set_param([modelName '/RT_BodyCM'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(l));

add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1500 45 1620 105]);
set_param([modelName '/Body_Solid'],'BrickDimensions',mat2str([body_D,body_H,body_W]),'InertiaType','Custom','Mass',num2str(M),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Icx,Icy,Icz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.2 0.5 0.8]');

add_line(modelName,'World/RConn1',            'RT_AxleHeight/LConn1');
add_line(modelName,'RT_AxleHeight/RConn1',    'RT_PrismaticAlign/LConn1');
add_line(modelName,'RT_PrismaticAlign/RConn1','Joint_x/LConn1');
add_line(modelName,'Joint_x/RConn1',          'RT_Realign/LConn1');
add_line(modelName,'RT_Realign/RConn1',        'Joint_alpha/LConn1');
add_line(modelName,'Joint_alpha/RConn1',       'RT_ThetaAxis/LConn1');
add_line(modelName,'RT_ThetaAxis/RConn1',      'Joint_theta/LConn1');
add_line(modelName,'Joint_theta/RConn1',       'RT_BodyCM/LConn1');
add_line(modelName,'RT_BodyCM/RConn1',         'Body_Solid/RConn1');

% Rueda derecha
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[960 160 1080 190]);
set_param([modelName '/RT_WheelR'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','+Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Axis'],'Position',[1140 160 1260 190]);
set_param([modelName '/RT_WheelR_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[1320 145 1440 205]);
set_param([modelName '/Joint_phi_R'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[1500 145 1620 205]);
set_param([modelName '/Wheel_R'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelR/LConn1');
add_line(modelName,'RT_WheelR/RConn1',      'RT_WheelR_Axis/LConn1');
add_line(modelName,'RT_WheelR_Axis/RConn1', 'Joint_phi_R/LConn1');
add_line(modelName,'Joint_phi_R/RConn1',    'Wheel_R/RConn1');

% Rueda izquierda
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL'],'Position',[960 250 1080 280]);
set_param([modelName '/RT_WheelL'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL_Axis'],'Position',[1140 250 1260 280]);
set_param([modelName '/RT_WheelL_Axis'],'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_L'],'Position',[1320 235 1440 295]);
set_param([modelName '/Joint_phi_L'],'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_L'],'Position',[1500 235 1620 295]);
set_param([modelName '/Wheel_L'],'CylinderRadius',num2str(r),'CylinderLength','0.08','InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]','MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_alpha/RConn1',    'RT_WheelL/LConn1');
add_line(modelName,'RT_WheelL/RConn1',      'RT_WheelL_Axis/LConn1');
add_line(modelName,'RT_WheelL_Axis/RConn1', 'Joint_phi_L/LConn1');
add_line(modelName,'Joint_phi_L/RConn1',    'Wheel_L/RConn1');

% Suelo visual
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Ground'],'Position',[30 280 150 310]);
set_param([modelName '/RT_Ground'],'RotationMethod','None','TranslationMethod','StandardAxis','TranslationStandardAxis','-Z','TranslationStandardOffset','0.025');
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Ground'],'Position',[200 280 320 310]);
set_param([modelName '/Ground'],'BrickDimensions','[10 10 0.05]','InertiaType','Custom','Mass','1e6','CenterOfMass','[0 0 0]','MomentsOfInertia','[1e6 1e6 1e6]','ProductsOfInertia','[0 0 0]','GraphicDiffuseColor','[0.6 0.6 0.6]');
add_line(modelName,'World/RConn1','RT_Ground/LConn1');
add_line(modelName,'RT_Ground/RConn1','Ground/RConn1');

fprintf('[Build] Estructura fisica OK\n');

%% =========================================================================
%  5. UPDATE → verificar port handles (leccion aprendida v3)
% =========================================================================
set_param(modelName,'SimulationCommand','update');
pH_Jt  = get_param([modelName '/Joint_theta'], 'PortHandles');
pH_Jx  = get_param([modelName '/Joint_x'],     'PortHandles');
pH_Ja  = get_param([modelName '/Joint_alpha'],  'PortHandles');
pH_JpR = get_param([modelName '/Joint_phi_R'],  'PortHandles');

fprintf('[Ports] Joint_theta=%d | Joint_x=%d | Joint_alpha=%d | Joint_phi_R=%d\n',...
    numel(pH_Jt.RConn), numel(pH_Jx.RConn), numel(pH_Ja.RConn), numel(pH_JpR.RConn));

if numel(pH_Ja.RConn) < 3
    error('[ERROR] Joint_alpha RConn=%d — SensePosition/SenseVelocity no activos', numel(pH_Ja.RConn));
end
fprintf('[Ports] Joint_alpha RConn(2)=alpha  RConn(3)=dalpha  OK\n');

%% =========================================================================
%  6. REFERENCIAS — theta_ref y alpha_ref como Steps
% =========================================================================
% theta_ref
tref_str = num2str(theta_ref_deg*pi/180,'%.6f');
add_block('simulink/Sources/Step',[modelName '/Step_th_on'], 'Position',[30 360 100 390]);
set_param([modelName '/Step_th_on'],'Time',num2str(t_avance),'Before','0','After',tref_str);
add_block('simulink/Sources/Step',[modelName '/Step_th_off'],'Position',[30 405 100 435]);
set_param([modelName '/Step_th_off'],'Time',num2str(t_stop),'Before','0','After',['-' tref_str]);
add_block('simulink/Math Operations/Sum',[modelName '/Sum_tref'],'Position',[140 368 180 427]);
set_param([modelName '/Sum_tref'],'Inputs','++');
pH_Stref = get_param([modelName '/Sum_tref'],'PortHandles');
add_line(modelName,get_param([modelName '/Step_th_on'], 'PortHandles').Outport(1), pH_Stref.Inport(1),'autorouting','on');
add_line(modelName,get_param([modelName '/Step_th_off'],'PortHandles').Outport(1), pH_Stref.Inport(2),'autorouting','on');

% alpha_ref
aref_str = num2str(alpha_ref_deg*pi/180,'%.6f');
add_block('simulink/Sources/Step',[modelName '/Step_al_on'], 'Position',[30 450 100 480]);
set_param([modelName '/Step_al_on'],'Time',num2str(t_avance_a),'Before','0','After',aref_str);
add_block('simulink/Sources/Step',[modelName '/Step_al_off'],'Position',[30 495 100 525]);
set_param([modelName '/Step_al_off'],'Time',num2str(t_stop_a),'Before','0','After',['-' aref_str]);
add_block('simulink/Math Operations/Sum',[modelName '/Sum_aref'],'Position',[140 458 180 517]);
set_param([modelName '/Sum_aref'],'Inputs','++');
pH_Saref = get_param([modelName '/Sum_aref'],'PortHandles');
add_line(modelName,get_param([modelName '/Step_al_on'], 'PortHandles').Outport(1), pH_Saref.Inport(1),'autorouting','on');
add_line(modelName,get_param([modelName '/Step_al_off'],'PortHandles').Outport(1), pH_Saref.Inport(2),'autorouting','on');

fprintf('[Refs] theta_ref=%.1f @ t=%.0f  |  alpha_ref=%.1f @ t=%.0f\n',...
    theta_ref_deg, t_avance, alpha_ref_deg, t_avance_a);

%% =========================================================================
%  7+8+9. CONTROLADOR LQR — MATLAB Function compacto
%  Entradas (8): theta, dtheta, x, dx, alpha, dalpha, theta_ref, alpha_ref
%  Salidas  (4): VR, VL, Va, Vd
%  K1/K2 hardcodeados con los valores calculados arriba
% =========================================================================
% 6 conversores PS→SL (estados sensados)
ps_names = {'PS2SL_th','PS2SL_dth','PS2SL_x','PS2SL_dx','PS2SL_al','PS2SL_dal'};
ps_src   = {pH_Jt.RConn(2), pH_Jt.RConn(3), pH_Jx.RConn(2), pH_Jx.RConn(3), pH_Ja.RConn(2), pH_Ja.RConn(3)};
ps_ypos  = [540 580 620 660 700 740];
for i = 1:6
    add_block('nesl_utility/PS-Simulink Converter',[modelName '/' ps_names{i}],'Position',[220 ps_ypos(i)-15 330 ps_ypos(i)+15]);
    add_line(modelName, ps_src{i}, get_param([modelName '/' ps_names{i}],'PortHandles').LConn(1),'autorouting','on');
end

% Bloque MATLAB Function: LQR_Controller
add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/LQR_Controller'],'Position',[500 580 700 760]);

% Escribir el script con K1/K2 hardcodeados
func_str = sprintf([...
'function [VR, VL, Va, Vd] = LQR_Controller(theta, dtheta, x, dx, alpha, dalpha, theta_ref, alpha_ref)\n'...
'%% LQR K1+K2 desacoplados — ganancias hardcodeadas\n'...
'%%  K1: avance [theta,dtheta,x,dx]  K2: giro [alpha,dalpha]\n'...
'%%  Salidas: VR, VL (a motores)  Va, Vd (para F_x y tau_alpha)\n'...
'K1 = %s;\n'...
'K2 = %s;\n'...
'V_sat_a = %.4f;\n'...
'V_sat_d = %.4f;\n'...
'V_sat_f = %.4f;\n'...
'\n'...
'Xe1 = [theta - theta_ref; dtheta; x; dx];\n'...
'Va  = max(min(-K1 * Xe1, V_sat_a), -V_sat_a);\n'...
'\n'...
'Xe2 = [alpha - alpha_ref; dalpha];\n'...
'Vd  = max(min(-K2 * Xe2, V_sat_d), -V_sat_d);\n'...
'\n'...
'VR = max(min(Va + Vd, V_sat_f), -V_sat_f);\n'...
'VL = max(min(Va - Vd, V_sat_f), -V_sat_f);\n'...
'end\n'], mat2str(K1,6), mat2str(K2,6), V_sat_a, V_sat_d, V_sat_f);

rt = sfroot();
ctrl_chart = rt.find('-isa','Stateflow.EMChart','Path',[modelName '/LQR_Controller']);
ctrl_chart.Script = func_str;

% Conectar PS2SL → LQR_Controller (entradas 1-6)
pH_LQR = get_param([modelName '/LQR_Controller'],'PortHandles');
for i = 1:6
    add_line(modelName, get_param([modelName '/' ps_names{i}],'PortHandles').Outport(1), pH_LQR.Inport(i),'autorouting','on');
end
% Entradas 7-8: referencias
add_line(modelName, pH_Stref.Outport(1), pH_LQR.Inport(7),'autorouting','on');
add_line(modelName, pH_Saref.Outport(1), pH_LQR.Inport(8),'autorouting','on');

fprintf('[LQR_Controller] MATLAB Function OK — K1/K2 hardcodeados\n');
fprintf('[LQR_Controller] Salidas: port1=VR  port2=VL  port3=Va  port4=Vd\n');

%% =========================================================================
%  10. TORQUES A JOINTS
%  LQR_Controller port1=VR  port2=VL  port3=Va  port4=Vd
% =========================================================================
% tau_R = alm*VR → Joint_phi_R
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauR'],'Position',[750 790 830 820]);
set_param([modelName '/Gain_tauR'],'Gain',num2str(alm));
pH_GtR = get_param([modelName '/Gain_tauR'],'PortHandles');
add_line(modelName, pH_LQR.Outport(1), pH_GtR.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauR'],'Position',[850 790 930 820]);
add_line(modelName, pH_GtR.Outport(1), get_param([modelName '/SL2PS_tauR'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauR/RConn1','Joint_phi_R/LConn2');

% tau_L = alm*VL → Joint_phi_L
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauL'],'Position',[750 830 830 860]);
set_param([modelName '/Gain_tauL'],'Gain',num2str(alm));
pH_GtL = get_param([modelName '/Gain_tauL'],'PortHandles');
add_line(modelName, pH_LQR.Outport(2), pH_GtL.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauL'],'Position',[850 830 930 860]);
add_line(modelName, pH_GtL.Outport(1), get_param([modelName '/SL2PS_tauL'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauL/RConn1','Joint_phi_L/LConn2');

% F_x = alm*2*Va/r → Joint_x  (Va = port3)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_Fx'],'Position',[750 870 830 900]);
set_param([modelName '/Gain_Fx'],'Gain',num2str(2*alm/r));
pH_GFx = get_param([modelName '/Gain_Fx'],'PortHandles');
add_line(modelName, pH_LQR.Outport(3), pH_GFx.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fx'],'Position',[850 870 930 900]);
add_line(modelName, pH_GFx.Outport(1), get_param([modelName '/SL2PS_Fx'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_Fx/RConn1','Joint_x/LConn2');

% tau_alpha = d*alm*Vd/r → Joint_alpha  (Vd = port4)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tau_al'],'Position',[750 910 830 940]);
set_param([modelName '/Gain_tau_al'],'Gain',num2str(d*alm/r));
pH_Gtal = get_param([modelName '/Gain_tau_al'],'PortHandles');
add_line(modelName, pH_LQR.Outport(4), pH_Gtal.Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tau_al'],'Position',[850 910 930 940]);
add_line(modelName, pH_Gtal.Outport(1), get_param([modelName '/SL2PS_tau_al'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tau_al/RConn1','Joint_alpha/LConn2');

fprintf('[Torques] tau_R | tau_L | F_x | tau_alpha → joints OK\n');

%% =========================================================================
%  11. LOGGING
%  LQR_Controller port1=VR  port2=VL  port3=Va  port4=Vd
% =========================================================================
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VRVL'],'Position',[750 950 780 990]);
set_param([modelName '/Mux_VRVL'],'Inputs','2');
pH_MVRVL = get_param([modelName '/Mux_VRVL'],'PortHandles');
add_line(modelName, pH_LQR.Outport(1), pH_MVRVL.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(2), pH_MVRVL.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VRVL'],'Position',[800 958 900 978]);
set_param([modelName '/ToWS_VRVL'],'VariableName','VRVL_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVRVL.Outport(1), get_param([modelName '/ToWS_VRVL'],'PortHandles').Inport(1),'autorouting','on');

add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VaVd'],'Position',[750 1000 780 1040]);
set_param([modelName '/Mux_VaVd'],'Inputs','2');
pH_MVaVd = get_param([modelName '/Mux_VaVd'],'PortHandles');
add_line(modelName, pH_LQR.Outport(3), pH_MVaVd.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(4), pH_MVaVd.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VaVd'],'Position',[800 1008 900 1028]);
set_param([modelName '/ToWS_VaVd'],'VariableName','VaVd_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVaVd.Outport(1), get_param([modelName '/ToWS_VaVd'],'PortHandles').Inport(1),'autorouting','on');

fprintf('[Logging] VRVL_log | VaVd_log OK\n');

%% =========================================================================
%  GUARDAR — SIN SIMULAR
% =========================================================================
save_system(modelName,[modelName '.slx']);

fprintf('\n=========================================================\n');
fprintf(' BUILD COMPLETO — Segway_Testbench_v4.slx guardado\n');
fprintf('\n PROXIMOS PASOS:\n');
fprintf('  1. Reorganiza los bloques en el modelo abierto\n');
fprintf('  2. Guarda con File > Save (Ctrl+S)\n');
fprintf('  3. Para simular, ejecuta run_v4.m\n');
fprintf('  IMPORTANTE: No vuelvas a correr build_v4.m o perderas\n');
fprintf('              la disposicion que organizaste manualmente.\n');
fprintf('=========================================================\n');
