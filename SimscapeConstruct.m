%% SimscapeConstruct.m  (v4.1 — Planar Joint + restricción lateral virtual)
%  Construye el modelo Simscape Multibody del Segway y corre la simulación.
%  Deja 'out' en el workspace para que LQR_Con_SegVelocidad lo use.
%
%  CAMBIO ARQUITECTURAL v4:
%    Cadena v3: World → Joint_alpha(origen) → Joint_fwd(heading) → Body
%    Cadena v4: World → Planar Joint (Px,Py,Rz) → Joint_theta → Body
%
%    Problema de v3: Joint_alpha estaba en el origen del mundo. Al avanzar
%    x metros, la inercia yaw crecía como M*x² (Steiner) → ~100x más lento.
%    Solución v4: un Planar Joint combina traslación XY + rotación Z en el
%    mismo punto → el yaw siempre gira alrededor de la posición actual.
%
%    La fuerza de avance se descompone en mundo: Fx = F*cos(α), Fy = F*sin(α).
%    La velocidad de avance se proyecta: dx_fwd = vx*cos(α) + vy*sin(α).
%
%  v4.1: Añade restricción lateral virtual (k_lat=10000 N/(m/s)).
%    El Planar Joint permite movimiento lateral libre. Sin restricción,
%    la inercia del robot lo hace derivar lateralmente en las curvas.
%    ForceDecomp aplica F_lat = -k_lat * v_lat donde v_lat es la
%    componente de velocidad perpendicular al heading.
%
%  Variables requeridas en workspace (definidas por LQR_Con_SegVelocidad):
%    modelName, M, m, r, d, l, g, Icy, Icz, Icx, Iw, Iwz, alm
%    body_W, body_D, body_H
%    theta0_deg, alpha0_deg, t_sim
%    tau_rider, th_lean, t_lean, t_back
%    tau_alpha, alpha_ref_deg, t_avance_a, t_stop_a
%    Kv, v_max, int_max
%    K1a, K2, V_sat_a, V_sat_d, V_sat_f
% =========================================================================

%% =========================================================================
%  4. CONSTRUIR MODELO SIMSCAPE
% =========================================================================
if bdIsLoaded(modelName),            close_system(modelName,0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end
new_system(modelName); open_system(modelName);
set_param(modelName,'Solver','ode23t','StopTime',num2str(t_sim),...
    'RelTol','1e-4','AbsTol','1e-6');

% ── Utilidades ────────────────────────────────────────────────────────────
add_block('nesl_utility/Solver Configuration',        [modelName '/Solver_Config'],'Position',[42 105 192 135]);
add_block('sm_lib/Utilities/Mechanism Configuration', [modelName '/Mech_Config'],  'Position',[227 130 417 160]);
set_param([modelName '/Mech_Config'],'GravityVector','[0; 0; -9.81]');
add_block('sm_lib/Frames and Transforms/World Frame', [modelName '/World'],         'Position',[92 40 192 70]);
add_line(modelName,'World/RConn1','Solver_Config/RConn1');
add_line(modelName,'World/RConn1','Mech_Config/RConn1');

% ── RT_AxleHeight: traslada el frame al eje de rueda ─────────────────────
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_AxleHeight'],'Position',[277 130 377 160]);
set_param([modelName '/RT_AxleHeight'],...
    'RotationMethod','None',...
    'TranslationMethod','StandardAxis','TranslationStandardAxis','+Z','TranslationStandardOffset',num2str(r));

% ── Planar Joint: Px (world X) + Py (world Y) + Rz (yaw) ────────────────
%    Rz gira alrededor de la posición actual (Px, Py) → inercia yaw = M33 ✓
%    Port mapping (con todo sensing + actuation activo):
%      LConn: 1=base_frame  2=Fx(Px)  3=Fy(Py)  4=Tz(Rz)
%      RConn: 1=follower    2=px      3=vx      4=py     5=vy     6=alpha  7=dalpha
add_block('sm_lib/Joints/Planar Joint',[modelName '/Joint_planar'],'Position',[407 125 567 185]);
set_param([modelName '/Joint_planar'],...
    'PxTorqueActuationMode','InputTorque','PxMotionActuationMode','ComputedMotion',...
    'PxSensePosition','on','PxSenseVelocity','on','PxDampingCoefficient','0',...
    'PyTorqueActuationMode','InputTorque','PyMotionActuationMode','ComputedMotion',...
    'PySensePosition','on','PySenseVelocity','on','PyDampingCoefficient','0',...
    'RzTorqueActuationMode','InputTorque','RzMotionActuationMode','ComputedMotion',...
    'RzSensePosition','on','RzSenseVelocity','on','RzDampingCoefficient','0');

% ── RT_ThetaAxis + Joint_theta ───────────────────────────────────────────
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_ThetaAxis'],'Position',[617 240 737 270]);
set_param([modelName '/RT_ThetaAxis'],...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90',...
    'TranslationMethod','None');

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_theta'],'Position',[767 225 887 285]);
set_param([modelName '/Joint_theta'],...
    'PositionTargetSpecify','on','PositionTargetValue',num2str(theta0_deg),...
    'PositionTargetValueUnits','deg','PositionTargetPriority','Low',...
    'TorqueActuationMode','NoTorque','MotionActuationMode','ComputedMotion',...
    'DampingCoefficient','0','SensePosition','on','SenseVelocity','on');

add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_BodyCM'],'Position',[917 205 1037 265]);
set_param([modelName '/RT_BodyCM'],...
    'RotationMethod','None','TranslationMethod','StandardAxis',...
    'TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(l));

add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Body_Solid'],'Position',[1070 225 1190 285]);
set_param([modelName '/Body_Solid'],...
    'BrickDimensions',mat2str([body_D,body_H,body_W]),...
    'InertiaType','Custom','Mass',num2str(M),'CenterOfMass','[0 0 0]',...
    'MomentsOfInertia',mat2str([Icx,Icy,Icz]),'ProductsOfInertia','[0 0 0]',...
    'GraphicDiffuseColor','[0.2 0.5 0.8]');

% ── Conexiones cadena principal ───────────────────────────────────────────
add_line(modelName,'World/RConn1',            'RT_AxleHeight/LConn1');
add_line(modelName,'RT_AxleHeight/RConn1',    'Joint_planar/LConn1');
add_line(modelName,'Joint_planar/RConn1',     'RT_ThetaAxis/LConn1');
add_line(modelName,'RT_ThetaAxis/RConn1',     'Joint_theta/LConn1');
add_line(modelName,'Joint_theta/RConn1',      'RT_BodyCM/LConn1');
add_line(modelName,'RT_BodyCM/RConn1',        'Body_Solid/RConn1');

% ── Rueda derecha (bifurca desde Planar Joint follower) ──────────────────
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR'],'Position',[617 120 737 150]);
set_param([modelName '/RT_WheelR'],...
    'RotationMethod','None','TranslationMethod','StandardAxis',...
    'TranslationStandardAxis','+Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelR_Axis'],'Position',[767 120 887 150]);
set_param([modelName '/RT_WheelR_Axis'],...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90',...
    'TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_R'],'Position',[917 105 1037 165]);
set_param([modelName '/Joint_phi_R'],...
    'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion',...
    'SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_R'],'Position',[1070 105 1190 165]);
set_param([modelName '/Wheel_R'],...
    'CylinderRadius',num2str(r),'CylinderLength','0.08',...
    'InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]',...
    'MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]',...
    'GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_planar/RConn1',    'RT_WheelR/LConn1');
add_line(modelName,'RT_WheelR/RConn1',       'RT_WheelR_Axis/LConn1');
add_line(modelName,'RT_WheelR_Axis/RConn1',  'Joint_phi_R/LConn1');
add_line(modelName,'Joint_phi_R/RConn1',     'Wheel_R/RConn1');

% ── Rueda izquierda ───────────────────────────────────────────────────────
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL'],'Position',[617 310 737 340]);
set_param([modelName '/RT_WheelL'],...
    'RotationMethod','None','TranslationMethod','StandardAxis',...
    'TranslationStandardAxis','-Y','TranslationStandardOffset',num2str(d/2));
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_WheelL_Axis'],'Position',[767 310 887 340]);
set_param([modelName '/RT_WheelL_Axis'],...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90',...
    'TranslationMethod','None');
add_block('sm_lib/Joints/Revolute Joint',[modelName '/Joint_phi_L'],'Position',[917 295 1037 355]);
set_param([modelName '/Joint_phi_L'],...
    'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion',...
    'SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Wheel_L'],'Position',[1070 295 1190 355]);
set_param([modelName '/Wheel_L'],...
    'CylinderRadius',num2str(r),'CylinderLength','0.08',...
    'InertiaType','Custom','Mass',num2str(m),'CenterOfMass','[0 0 0]',...
    'MomentsOfInertia',mat2str([Iwz,Iw,Iwz]),'ProductsOfInertia','[0 0 0]',...
    'GraphicDiffuseColor','[0.1 0.1 0.1]');
add_line(modelName,'Joint_planar/RConn1',    'RT_WheelL/LConn1');
add_line(modelName,'RT_WheelL/RConn1',       'RT_WheelL_Axis/LConn1');
add_line(modelName,'RT_WheelL_Axis/RConn1',  'Joint_phi_L/LConn1');
add_line(modelName,'Joint_phi_L/RConn1',     'Wheel_L/RConn1');

% ── Suelo ─────────────────────────────────────────────────────────────────
add_block('sm_lib/Frames and Transforms/Rigid Transform',[modelName '/RT_Ground'],'Position',[277 190 397 220]);
set_param([modelName '/RT_Ground'],...
    'RotationMethod','None','TranslationMethod','StandardAxis',...
    'TranslationStandardAxis','-Z','TranslationStandardOffset','0.025');
add_block('sm_lib/Body Elements/Brick Solid',[modelName '/Ground'],'Position',[427 190 547 220]);
set_param([modelName '/Ground'],...
    'BrickDimensions','[30 8 0.05]','InertiaType','Custom','Mass','1e6',...
    'CenterOfMass','[0 0 0]','MomentsOfInertia','[1e6 1e6 1e6]','ProductsOfInertia','[0 0 0]',...
    'GraphicDiffuseColor','[0.90 0.90 0.90]');
add_line(modelName,'World/RConn1','RT_Ground/LConn1');
add_line(modelName,'RT_Ground/RConn1','Ground/RConn1');

% ── Cuadrícula visual 40×40 cm (STL generado, masa despreciable) ────────
grid_stl = fullfile(pwd, 'grid_floor.stl');
fid = fopen(grid_stl, 'w');
fprintf(fid, 'solid grid\n');
hw = 0.005; gz = 0.001;   % semi-ancho línea [m], altura sobre suelo [m]
wr = @(x1,y1,x2,y2) fprintf(fid, [...
    'facet normal 0 0 1\n outer loop\n' ...
    '  vertex %.4f %.4f %.4f\n  vertex %.4f %.4f %.4f\n  vertex %.4f %.4f %.4f\n' ...
    ' endloop\nendfacet\n' ...
    'facet normal 0 0 1\n outer loop\n' ...
    '  vertex %.4f %.4f %.4f\n  vertex %.4f %.4f %.4f\n  vertex %.4f %.4f %.4f\n' ...
    ' endloop\nendfacet\n'], ...
    x1,y1,gz, x2,y1,gz, x2,y2,gz,  x1,y1,gz, x2,y2,gz, x1,y2,gz);
for y = -4:0.4:4,   wr(-15, y-hw, 15, y+hw); end
for x = -15:0.4:15, wr(x-hw, -4, x+hw, 4);  end
fprintf(fid, 'endsolid grid\n');
fclose(fid);

add_block('sm_lib/Body Elements/File Solid',[modelName '/Grid_Lines'],'Position',[427 230 547 260]);
set_param([modelName '/Grid_Lines'],...
    'ExtGeomFileName', grid_stl,...
    'UnitType','Custom','ExtGeomFileUnits','m',...
    'InertiaType','Custom','Mass','0.001',...
    'CenterOfMass','[0 0 0]',...
    'MomentsOfInertia','[1e-6 1e-6 1e-6]','ProductsOfInertia','[0 0 0]',...
    'GraphicDiffuseColor','[0.5 0.5 0.5]');
add_line(modelName,'World/RConn1','Grid_Lines/RConn1');

fprintf('[Build] Estructura fisica OK (v4.1 — Planar Joint + cuadrícula visual)\n');

%% =========================================================================
%  5. UPDATE → port handles
% =========================================================================
set_param(modelName,'SimulationCommand','update');
pH_PJ  = get_param([modelName '/Joint_planar'],'PortHandles');
pH_Jt  = get_param([modelName '/Joint_theta'], 'PortHandles');
pH_JpR = get_param([modelName '/Joint_phi_R'], 'PortHandles');

fprintf('[Ports] planar: LConn=%d RConn=%d | theta: RConn=%d | phi_R: RConn=%d\n',...
    numel(pH_PJ.LConn), numel(pH_PJ.RConn), numel(pH_Jt.RConn), numel(pH_JpR.RConn));
if numel(pH_PJ.RConn) < 7
    error('[ERROR] Planar Joint RConn=%d (esperado 7)', numel(pH_PJ.RConn));
end

%% =========================================================================
%  6. THETA_RIDER + ALPHA_REF — filtros 1er orden
% =========================================================================
add_block('simulink/Sources/Step',[modelName '/Step_rider_on'],'Position',[200 500 270 530]);
set_param([modelName '/Step_rider_on'],'Time',num2str(t_lean),'Before','0','After',num2str(th_lean,'%.6f'));
add_block('simulink/Sources/Step',[modelName '/Step_rider_off'],'Position',[200 550 270 580]);
set_param([modelName '/Step_rider_off'],'Time',num2str(t_back),'Before','0','After',num2str(-th_lean,'%.6f'));
add_block('simulink/Math Operations/Sum',[modelName '/Sum_rider'],'Position',[310 508 350 572]);
set_param([modelName '/Sum_rider'],'Inputs','++');
pH_Srider = get_param([modelName '/Sum_rider'],'PortHandles');
add_line(modelName, get_param([modelName '/Step_rider_on'], 'PortHandles').Outport(1), pH_Srider.Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/Step_rider_off'],'PortHandles').Outport(1), pH_Srider.Inport(2),'autorouting','on');
add_block('simulink/Continuous/Transfer Fcn',[modelName '/Filtro_rider'],'Position',[380 508 480 542]);
set_param([modelName '/Filtro_rider'],'Numerator','[1]','Denominator',sprintf('[%.4f 1]',tau_rider));
pH_Frider = get_param([modelName '/Filtro_rider'],'PortHandles');
add_line(modelName, pH_Srider.Outport(1), pH_Frider.Inport(1),'autorouting','on');

% alpha_ref — con filtro 1er orden (tau_alpha)
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
add_block('simulink/Continuous/Transfer Fcn',[modelName '/Filtro_alpha'],'Position',[380 628 480 662]);
set_param([modelName '/Filtro_alpha'],'Numerator','[1]','Denominator',sprintf('[%.4f 1]',tau_alpha));
pH_Falpha = get_param([modelName '/Filtro_alpha'],'PortHandles');
add_line(modelName, pH_Saref.Outport(1), pH_Falpha.Inport(1),'autorouting','on');

fprintf('[Rider] filtro rider tau=%.2fs | filtro alpha tau=%.2fs OK\n', tau_rider, tau_alpha);

%% =========================================================================
%  7. PS2SL — sensing desde Planar Joint + Joint_theta
%     Planar Joint RConn: 1=frame 2=px 3=vx 4=py 5=vy 6=alpha 7=dalpha
% =========================================================================
% ── Señales físicas → Simulink ────────────────────────────────────────────
ps_defs = {
    'PS2SL_th',  pH_Jt.RConn(2),  [580 390 690 420];   % theta
    'PS2SL_dth', pH_Jt.RConn(3),  [580 430 690 460];   % dtheta
    'PS2SL_px',  pH_PJ.RConn(2),  [580 470 690 500];   % px (world X)
    'PS2SL_vx',  pH_PJ.RConn(3),  [580 510 690 540];   % vx (world X vel)
    'PS2SL_py',  pH_PJ.RConn(4),  [580 550 690 580];   % py (world Y)
    'PS2SL_vy',  pH_PJ.RConn(5),  [580 590 690 620];   % vy (world Y vel)
    'PS2SL_al',  pH_PJ.RConn(6),  [580 630 690 660];   % alpha (yaw)
    'PS2SL_dal', pH_PJ.RConn(7),  [580 670 690 700];   % dalpha (yaw vel)
};
for i = 1:size(ps_defs,1)
    add_block('nesl_utility/PS-Simulink Converter',[modelName '/' ps_defs{i,1}],'Position',ps_defs{i,3});
    add_line(modelName, ps_defs{i,2}, get_param([modelName '/' ps_defs{i,1}],'PortHandles').LConn(1),'autorouting','on');
end

% ── Proyección de velocidad: dx_fwd = vx·cos(α) + vy·sin(α) ─────────────
%    Descompone la velocidad mundo al heading del robot
add_block('simulink/Math Operations/Trigonometric Function',[modelName '/Cos_al'],'Position',[720 630 770 660]);
set_param([modelName '/Cos_al'],'Operator','cos');
add_block('simulink/Math Operations/Trigonometric Function',[modelName '/Sin_al'],'Position',[720 670 770 700]);
set_param([modelName '/Sin_al'],'Operator','sin');
% Alimentar cos/sin con alpha
pH_al_out = get_param([modelName '/PS2SL_al'],'PortHandles');
add_line(modelName, pH_al_out.Outport(1), get_param([modelName '/Cos_al'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, pH_al_out.Outport(1), get_param([modelName '/Sin_al'],'PortHandles').Inport(1),'autorouting','on');

% vx * cos(α)
add_block('simulink/Math Operations/Product',[modelName '/Prod_vx_cos'],'Position',[800 505 840 540]);
set_param([modelName '/Prod_vx_cos'],'Inputs','**');
add_line(modelName, get_param([modelName '/PS2SL_vx'],'PortHandles').Outport(1), get_param([modelName '/Prod_vx_cos'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/Cos_al'],'PortHandles').Outport(1),   get_param([modelName '/Prod_vx_cos'],'PortHandles').Inport(2),'autorouting','on');

% vy * sin(α)
add_block('simulink/Math Operations/Product',[modelName '/Prod_vy_sin'],'Position',[800 555 840 590]);
set_param([modelName '/Prod_vy_sin'],'Inputs','**');
add_line(modelName, get_param([modelName '/PS2SL_vy'],'PortHandles').Outport(1), get_param([modelName '/Prod_vy_sin'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/Sin_al'],'PortHandles').Outport(1),   get_param([modelName '/Prod_vy_sin'],'PortHandles').Inport(2),'autorouting','on');

% dx_fwd = vx·cos + vy·sin
add_block('simulink/Math Operations/Sum',[modelName '/Sum_dx_fwd'],'Position',[870 510 910 585]);
set_param([modelName '/Sum_dx_fwd'],'Inputs','++');
add_line(modelName, get_param([modelName '/Prod_vx_cos'],'PortHandles').Outport(1), get_param([modelName '/Sum_dx_fwd'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/Prod_vy_sin'],'PortHandles').Outport(1), get_param([modelName '/Sum_dx_fwd'],'PortHandles').Inport(2),'autorouting','on');

pH_dx_fwd = get_param([modelName '/Sum_dx_fwd'],'PortHandles');

% ── x_fwd = ∫ dx_fwd dt  (distancia recorrida a lo largo del heading) ────
add_block('simulink/Continuous/Integrator',[modelName '/Int_x_fwd'],'Position',[940 525 990 560]);
set_param([modelName '/Int_x_fwd'],'InitialCondition','0');
add_line(modelName, pH_dx_fwd.Outport(1), get_param([modelName '/Int_x_fwd'],'PortHandles').Inport(1),'autorouting','on');

fprintf('[Sensing] 8 PS2SL + proyección dx_fwd + integrador x_fwd OK\n');

%% =========================================================================
%  8. INTEGRADOR DE VELOCIDAD — ∫(dx_fwd - v_ref)dt con anti-windup
% =========================================================================
add_block('simulink/Math Operations/Gain',[modelName '/Gain_Kv'],'Position',[510 750 570 780]);
set_param([modelName '/Gain_Kv'],'Gain',num2str(Kv));
pH_GKv = get_param([modelName '/Gain_Kv'],'PortHandles');
add_line(modelName, pH_Frider.Outport(1), pH_GKv.Inport(1),'autorouting','on');

add_block('simulink/Discontinuities/Saturation',[modelName '/Sat_vref'],'Position',[590 750 650 780]);
set_param([modelName '/Sat_vref'],'UpperLimit',num2str(v_max),'LowerLimit',num2str(-v_max));
pH_Svref = get_param([modelName '/Sat_vref'],'PortHandles');
add_line(modelName, pH_GKv.Outport(1), pH_Svref.Inport(1),'autorouting','on');

add_block('simulink/Math Operations/Sum',[modelName '/Sum_ev'],'Position',[680 748 720 812]);
set_param([modelName '/Sum_ev'],'Inputs','+-');
pH_Sev = get_param([modelName '/Sum_ev'],'PortHandles');
% dx_fwd → Sum_ev input 1
add_line(modelName, pH_dx_fwd.Outport(1), pH_Sev.Inport(1),'autorouting','on');
add_line(modelName, pH_Svref.Outport(1),  pH_Sev.Inport(2),'autorouting','on');

add_block('simulink/Continuous/Integrator',[modelName '/Int_ev'],'Position',[750 755 800 790]);
set_param([modelName '/Int_ev'],...
    'LimitOutput','on',...
    'UpperSaturationLimit', num2str( int_max),...
    'LowerSaturationLimit', num2str(-int_max),...
    'InitialCondition','0');
pH_Iev = get_param([modelName '/Int_ev'],'PortHandles');
add_line(modelName, pH_Sev.Outport(1), pH_Iev.Inport(1),'autorouting','on');

fprintf('[Int_ev] Integrador dx_fwd-v_ref con anti-windup +-%.1f OK\n', int_max);

%% =========================================================================
%  9. LQR_CONTROLLER — MATLAB Function
% =========================================================================
add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/LQR_Controller'],'Position',[1020 400 1180 780]);

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

% Conectar 9 entradas al LQR:
%   1=theta  2=dtheta  3=px(~)  4=dx_fwd  5=alpha  6=dalpha
%   7=theta_rider  8=alpha_ref  9=int_ev
add_line(modelName, get_param([modelName '/PS2SL_th'], 'PortHandles').Outport(1), pH_LQR.Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/PS2SL_dth'],'PortHandles').Outport(1), pH_LQR.Inport(2),'autorouting','on');
add_line(modelName, get_param([modelName '/PS2SL_px'], 'PortHandles').Outport(1), pH_LQR.Inport(3),'autorouting','on');  % ~ (no usado)
add_line(modelName, pH_dx_fwd.Outport(1),                                         pH_LQR.Inport(4),'autorouting','on');  % dx_fwd proyectado
add_line(modelName, get_param([modelName '/PS2SL_al'], 'PortHandles').Outport(1), pH_LQR.Inport(5),'autorouting','on');
add_line(modelName, get_param([modelName '/PS2SL_dal'],'PortHandles').Outport(1), pH_LQR.Inport(6),'autorouting','on');
add_line(modelName, pH_Frider.Outport(1),                                         pH_LQR.Inport(7),'autorouting','on');
add_line(modelName, pH_Falpha.Outport(1),                                         pH_LQR.Inport(8),'autorouting','on');
add_line(modelName, pH_Iev.Outport(1),                                            pH_LQR.Inport(9),'autorouting','on');

fprintf('[LQR_Controller] 9 entradas conectadas OK\n');

%% =========================================================================
%  10. FUERZAS Y TORQUES
% =========================================================================

% τ_R → Joint_phi_R
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauR'],'Position',[1220 420 1270 450]);
set_param([modelName '/Gain_tauR'],'Gain',num2str(alm));
add_line(modelName, pH_LQR.Outport(1), get_param([modelName '/Gain_tauR'],'PortHandles').Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauR'],'Position',[1290 420 1370 450]);
add_line(modelName, get_param([modelName '/Gain_tauR'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_tauR'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauR/RConn1','Joint_phi_R/LConn2');

% τ_L → Joint_phi_L
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tauL'],'Position',[1220 470 1270 500]);
set_param([modelName '/Gain_tauL'],'Gain',num2str(alm));
add_line(modelName, pH_LQR.Outport(2), get_param([modelName '/Gain_tauL'],'PortHandles').Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tauL'],'Position',[1290 470 1370 500]);
add_line(modelName, get_param([modelName '/Gain_tauL'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_tauL'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName,'SL2PS_tauL/RConn1','Joint_phi_L/LConn2');

% F_fwd = (2*alm/r) * Va
add_block('simulink/Math Operations/Gain',[modelName '/Gain_F'],'Position',[1220 520 1270 550]);
set_param([modelName '/Gain_F'],'Gain',num2str(2*alm/r));
add_line(modelName, pH_LQR.Outport(3), get_param([modelName '/Gain_F'],'PortHandles').Inport(1),'autorouting','on');

% ForceDecomp: descompone F_fwd en Fx,Fy + fuerza lateral de restricción
%   v_lat = -vx*sin(α) + vy*cos(α)  (velocidad perpendicular al heading)
%   F_lat = -k_lat * v_lat           (fricción virtual de neumático)
%   Fx = F_fwd*cos(α) - F_lat*sin(α)
%   Fy = F_fwd*sin(α) + F_lat*cos(α)
add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/ForceDecomp'],'Position',[1310 510 1430 600]);
fd_str = sprintf([...
'function [Fx, Fy] = ForceDecomp(F_fwd, vx, vy, alpha)\n'...
'%%#codegen\n'...
'ca = cos(alpha); sa = sin(alpha);\n'...
'v_lat = -vx*sa + vy*ca;\n'...
'k_lat = %.1f;\n'...
'F_lat = -k_lat * v_lat;\n'...
'Fx = F_fwd*ca + F_lat*(-sa);\n'...
'Fy = F_fwd*sa + F_lat*ca;\n'...
'end\n'], 10000.0);
rt2 = sfroot();
fd_chart = rt2.find('-isa','Stateflow.EMChart','Path',[modelName '/ForceDecomp']);
fd_chart.Script = fd_str;

pH_FD = get_param([modelName '/ForceDecomp'],'PortHandles');
% Inputs: 1=F_fwd, 2=vx, 3=vy, 4=alpha
add_line(modelName, get_param([modelName '/Gain_F'],'PortHandles').Outport(1),    pH_FD.Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/PS2SL_vx'],'PortHandles').Outport(1),  pH_FD.Inport(2),'autorouting','on');
add_line(modelName, get_param([modelName '/PS2SL_vy'],'PortHandles').Outport(1),  pH_FD.Inport(3),'autorouting','on');
add_line(modelName, get_param([modelName '/PS2SL_al'],'PortHandles').Outport(1),  pH_FD.Inport(4),'autorouting','on');

% Fx → Planar Joint Px (LConn2)
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fx'],'Position',[1460 515 1540 540]);
add_line(modelName, pH_FD.Outport(1), get_param([modelName '/SL2PS_Fx'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/SL2PS_Fx'],'PortHandles').RConn(1), pH_PJ.LConn(2),'autorouting','on');

% Fy → Planar Joint Py (LConn3)
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_Fy'],'Position',[1460 565 1540 590]);
add_line(modelName, pH_FD.Outport(2), get_param([modelName '/SL2PS_Fy'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/SL2PS_Fy'],'PortHandles').RConn(1), pH_PJ.LConn(3),'autorouting','on');

% T_yaw = (d*alm/r) * Vd → Planar Joint Rz (LConn4)
add_block('simulink/Math Operations/Gain',[modelName '/Gain_tau_al'],'Position',[1220 610 1270 640]);
set_param([modelName '/Gain_tau_al'],'Gain',num2str(d*alm/r));
add_line(modelName, pH_LQR.Outport(4), get_param([modelName '/Gain_tau_al'],'PortHandles').Inport(1),'autorouting','on');
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL2PS_tau_al'],'Position',[1290 613 1370 637]);
add_line(modelName, get_param([modelName '/Gain_tau_al'],'PortHandles').Outport(1), get_param([modelName '/SL2PS_tau_al'],'PortHandles').Inport(1),'autorouting','on');
add_line(modelName, get_param([modelName '/SL2PS_tau_al'],'PortHandles').RConn(1), pH_PJ.LConn(4),'autorouting','on');

fprintf('[Forces] Fx,Fy→Planar(Px,Py) | Tz→Planar(Rz) | tauR,tauL→phi_R,phi_L OK\n');

%% =========================================================================
%  11. LOGGING
% =========================================================================
% VR/VL
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VRVL'],'Position',[1220 660 1250 700]);
set_param([modelName '/Mux_VRVL'],'Inputs','2');
pH_MVRVL = get_param([modelName '/Mux_VRVL'],'PortHandles');
add_line(modelName, pH_LQR.Outport(1), pH_MVRVL.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(2), pH_MVRVL.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VRVL'],'Position',[1270 665 1360 685]);
set_param([modelName '/ToWS_VRVL'],'VariableName','VRVL_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVRVL.Outport(1), get_param([modelName '/ToWS_VRVL'],'PortHandles').Inport(1),'autorouting','on');

% Va/Vd
add_block('simulink/Signal Routing/Mux',[modelName '/Mux_VaVd'],'Position',[1220 710 1250 750]);
set_param([modelName '/Mux_VaVd'],'Inputs','2');
pH_MVaVd = get_param([modelName '/Mux_VaVd'],'PortHandles');
add_line(modelName, pH_LQR.Outport(3), pH_MVaVd.Inport(1),'autorouting','on');
add_line(modelName, pH_LQR.Outport(4), pH_MVaVd.Inport(2),'autorouting','on');
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_VaVd'],'Position',[1270 715 1360 735]);
set_param([modelName '/ToWS_VaVd'],'VariableName','VaVd_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_MVaVd.Outport(1), get_param([modelName '/ToWS_VaVd'],'PortHandles').Inport(1),'autorouting','on');

% int_ev
add_block('simulink/Sinks/To Workspace',[modelName '/ToWS_intev'],'Position',[830 800 920 820]);
set_param([modelName '/ToWS_intev'],'VariableName','intev_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_Iev.Outport(1), get_param([modelName '/ToWS_intev'],'PortHandles').Inport(1),'autorouting','on');

% 6 estados del controlador: th, dth, x_fwd, dx_fwd, al, dal
log_defs = {
    'ToWS_th',  'th_log',  'PS2SL_th';
    'ToWS_dth', 'dth_log', 'PS2SL_dth';
    'ToWS_al',  'al_log',  'PS2SL_al';
    'ToWS_dal', 'dal_log', 'PS2SL_dal';
};
log_pos_y = [850 870 890 910];
for i = 1:size(log_defs,1)
    blk = [modelName '/' log_defs{i,1}];
    add_block('simulink/Sinks/To Workspace', blk, 'Position', [1270 log_pos_y(i) 1360 log_pos_y(i)+15]);
    set_param(blk,'VariableName',log_defs{i,2},'SaveFormat','Array','SampleTime','-1');
    add_line(modelName, get_param([modelName '/' log_defs{i,3}],'PortHandles').Outport(1), ...
             get_param(blk,'PortHandles').Inport(1),'autorouting','on');
end

% x_fwd (posición a lo largo del heading — integrada)
blk = [modelName '/ToWS_x'];
add_block('simulink/Sinks/To Workspace', blk, 'Position', [1270 930 1360 945]);
set_param(blk,'VariableName','x_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, get_param([modelName '/Int_x_fwd'],'PortHandles').Outport(1), ...
         get_param(blk,'PortHandles').Inport(1),'autorouting','on');

% dx_fwd (velocidad de avance proyectada)
blk = [modelName '/ToWS_dx'];
add_block('simulink/Sinks/To Workspace', blk, 'Position', [1270 950 1360 965]);
set_param(blk,'VariableName','dx_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, pH_dx_fwd.Outport(1), get_param(blk,'PortHandles').Inport(1),'autorouting','on');

% px, py — posición world directa (para trayectoria sin cumtrapz)
blk = [modelName '/ToWS_px'];
add_block('simulink/Sinks/To Workspace', blk, 'Position', [1270 970 1360 985]);
set_param(blk,'VariableName','px_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, get_param([modelName '/PS2SL_px'],'PortHandles').Outport(1), ...
         get_param(blk,'PortHandles').Inport(1),'autorouting','on');

blk = [modelName '/ToWS_py'];
add_block('simulink/Sinks/To Workspace', blk, 'Position', [1270 990 1360 1005]);
set_param(blk,'VariableName','py_log','SaveFormat','Array','SampleTime','-1');
add_line(modelName, get_param([modelName '/PS2SL_py'],'PortHandles').Outport(1), ...
         get_param(blk,'PortHandles').Inport(1),'autorouting','on');

fprintf('[Logging] VRVL | VaVd | intev | th,dth,al,dal | x,dx (fwd) | px,py (world) OK\n');

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
