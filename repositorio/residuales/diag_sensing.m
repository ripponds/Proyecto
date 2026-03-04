%% diag_sensing.m
%  Diagnóstico definitivo de puertos de sensing en Simscape Multibody 2025b
%  Objetivo: saber si SensePosition crea RConn adicionales o Outports

clear; clc; bdclose all;
mdl = 'diag_sense_test';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);
set_param(mdl,'Solver','ode23t','StopTime','0.1');

% Estructura mínima funcional
add_block('nesl_utility/Solver Configuration', [mdl '/SC'], 'Position',[30 30 150 60]);
add_block('sm_lib/Utilities/Mechanism Configuration',[mdl '/MC'],'Position',[30 90 200 120]);
add_block('sm_lib/Frames and Transforms/World Frame',[mdl '/W'], 'Position',[30 160 130 190]);
add_block('sm_lib/Joints/Revolute Joint',             [mdl '/RJ'],'Position',[200 145 320 205]);
add_block('sm_lib/Body Elements/Brick Solid',         [mdl '/BS'],'Position',[380 145 500 205]);
set_param([mdl '/BS'],'Mass','1','MomentsOfInertia','[0.1 0.1 0.1]');

add_line(mdl,'W/RConn1','SC/RConn1');
add_line(mdl,'W/RConn1','MC/RConn1');
add_line(mdl,'W/RConn1','RJ/LConn1');
add_line(mdl,'RJ/RConn1','BS/RConn1');

% ── Estado ANTES de habilitar sensing ──
set_param(mdl,'SimulationCommand','update');
pH0 = get_param([mdl '/RJ'],'PortHandles');
fprintf('=== ANTES de SensePosition=on ===\n');
fprintf('  LConn=%d  RConn=%d  Inport=%d  Outport=%d\n',...
    numel(pH0.LConn),numel(pH0.RConn),numel(pH0.Inport),numel(pH0.Outport));

% ── Habilitar sensing ──
set_param([mdl '/RJ'],'SensePosition','on','SenseVelocity','on');
set_param(mdl,'SimulationCommand','update');
pH1 = get_param([mdl '/RJ'],'PortHandles');
fprintf('\n=== DESPUÉS de SensePosition=on + SenseVelocity=on ===\n');
fprintf('  LConn=%d  RConn=%d  Inport=%d  Outport=%d\n',...
    numel(pH1.LConn),numel(pH1.RConn),numel(pH1.Inport),numel(pH1.Outport));
fprintf('  RConn handles: %s\n', mat2str(pH1.RConn));
fprintf('  LConn handles: %s\n', mat2str(pH1.LConn));

% ── Intentar conectar PS2SL si hay RConn extra ──
if numel(pH1.RConn) > 1
    fprintf('\n  >> RConn extra encontrado — intentando conectar PS2SL...\n');
    add_block('nesl_utility/PS-Simulink Converter',[mdl '/PS2SL'],'Position',[560 155 660 185]);
    pH_ps = get_param([mdl '/PS2SL'],'PortHandles');
    fprintf('  PS2SL: LConn=%d RConn=%d Inport=%d Outport=%d\n',...
        numel(pH_ps.LConn),numel(pH_ps.RConn),numel(pH_ps.Inport),numel(pH_ps.Outport));
    try
        add_line(mdl, pH1.RConn(2), pH_ps.LConn(1), 'autorouting','on');
        fprintf('  Conexión RJ.RConn(2) -> PS2SL.LConn(1): OK\n');
    catch e
        fprintf('  Conexión RConn(2)->LConn(1) FALLO: %s\n', e.message);
        try
            add_line(mdl, pH1.RConn(2), pH_ps.RConn(1), 'autorouting','on');
            fprintf('  Conexión RJ.RConn(2) -> PS2SL.RConn(1): OK\n');
        catch e2
            fprintf('  Conexión RConn(2)->RConn(1) FALLO: %s\n', e2.message);
        end
    end
    set_param(mdl,'SimulationCommand','update');
    pH_ps2 = get_param([mdl '/PS2SL'],'PortHandles');
    fprintf('  PS2SL tras update: Outport=%d\n', numel(pH_ps2.Outport));
end

% ── Diagnóstico Transform Sensor con update completo ──
add_block('sm_lib/Frames and Transforms/Transform Sensor',[mdl '/TS'],'Position',[200 280 320 340]);
set_param([mdl '/TS'],'SenseAngle','on','SenseOmegaZ','on');
set_param(mdl,'SimulationCommand','update');
pH_ts = get_param([mdl '/TS'],'PortHandles');
fprintf('\n=== Transform Sensor (SenseAngle+SenseOmegaZ) ===\n');
fprintf('  LConn=%d  RConn=%d  Inport=%d  Outport=%d\n',...
    numel(pH_ts.LConn),numel(pH_ts.RConn),numel(pH_ts.Inport),numel(pH_ts.Outport));

fprintf('\n=== FIN DIAGNÓSTICO ===\n');
close_system(mdl,0);
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
