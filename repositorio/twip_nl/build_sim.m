% =========================================================================
%  build_sim.m — Construye TWIP_NL.slx
%  PREREQUISITO: correr twip_params.m primero
%
%  Arquitectura:
%    Constant(p_phys) ──────────────────────────────► Plant
%    Constant(p_bache) ──► Bache ──► Sum_pitch ──────► Plant
%    Constant(d1_ext)  ──► Sum_pitch
%    Constant(d2_ext)  ─────────────────────────────► Plant
%    Constant(K_lqr)   ──────────────────────────────► Controller
%    X0 ──► Integrator ──► Plant (feedback)
%                     └──► Controller ──► Plant
% =========================================================================
clc;
assert(exist('p_phys','var')==1, 'Corre twip_params.m primero');

%% Cargar estado guardado por el usuario (si existe)
pos_file = fullfile(pwd, 'block_positions.mat');
bp           = struct();
deleted_cons = {};
if exist(pos_file, 'file')
    tmp = load(pos_file);
    if isfield(tmp,'block_pos'),        bp           = tmp.block_pos;        end
    if isfield(tmp,'deleted_connections'), deleted_cons = tmp.deleted_connections; end
    fprintf('Usando estado guardado de block_positions.mat\n');
end

% Helper: devuelve posición guardada si existe, si no usa el default
pos = @(name, default) get_saved_pos(bp, name, default);

% Helper: true si la conexión fue eliminada por el usuario
is_deleted = @(s,sp,d,dp) check_deleted(deleted_cons, s, sp, d, dp);

mdl = 'TWIP_NL';

if bdIsLoaded(mdl)
    if ~strcmp(get_param(mdl,'SimulationStatus'),'stopped')
        set_param(mdl,'SimulationCommand','stop');
    end
    close_system(mdl, 0);
end
slx = fullfile(pwd, [mdl '.slx']);
if exist(slx,'file'), delete(slx); end
new_system(mdl);

%% Solver
set_param(mdl,'StopTime','10','Solver','ode45', ...
              'RelTol','1e-4','AbsTol','1e-6','MaxStep','0.02');

%% ── BLOQUES ──────────────────────────────────────────────────────────────

% Integrador de estados
add_block('simulink/Continuous/Integrator',[mdl '/Integrator'], ...
    'Position',pos('Integrator',[420,200,460,240]), ...
    'InitialConditionSource','internal','InitialCondition','X0');

% MATLAB Function blocks
add_mfcn(mdl,'Plant',      pos('Plant',      [220,190,360,250]));
add_mfcn(mdl,'Controller', pos('Controller', [520,290,660,340]));
add_mfcn(mdl,'Bache',      pos('Bache',      [100,360,220,400]));

% Constant blocks (leen del workspace)
add_const(mdl,'Kc',      'K_lqr',   pos('Kc',      [520,360,660,390]));
add_const(mdl,'P_phys',  'p_phys',  pos('P_phys',  [60, 190,160,220]));
add_const(mdl,'P_bache', 'p_bache', pos('P_bache', [60, 360,160,390]));
add_const(mdl,'D1',      'd1_ext',  pos('D1',      [60, 250,140,280]));
add_const(mdl,'D2',      'd2_ext',  pos('D2',      [60, 300,140,330]));

% Sum pitch: bache + d1_ext
add_block('simulink/Math Operations/Sum',[mdl '/Sum_pitch'], ...
    'Position',pos('Sum_pitch',[170,248,195,272]),'Inputs','++');

% Mux perturbaciones d = [d_pitch; d2_ext]
add_block('simulink/Signal Routing/Mux',[mdl '/Mux_d'], ...
    'Position',pos('Mux_d',[210,248,216,322]),'Inputs','2');

% Demux x[6] para scopes y bache
add_block('simulink/Signal Routing/Demux',[mdl '/Demux_x'], ...
    'Position',pos('Demux_x',[475,178,481,392]),'Outputs','6');

% Demux u[2] para scope
add_block('simulink/Signal Routing/Demux',[mdl '/Demux_u'], ...
    'Position',pos('Demux_u',[675,295,681,345]),'Outputs','2');

% Scopes
add_scope(mdl,'Scope_tilt',pos('Scope_tilt',[760,183,810,237]),2);
add_scope(mdl,'Scope_ctrl',pos('Scope_ctrl',[760,303,810,357]),2);

% Terminators para psi y psi_dot (no monitoreados)
add_block('simulink/Sinks/Terminator',[mdl '/Term_psi'],    'Position',pos('Term_psi',    [530,290,550,310]));
add_block('simulink/Sinks/Terminator',[mdl '/Term_psi_dot'],'Position',pos('Term_psi_dot',[530,320,550,340]));

% Botón "Exportar → Claude": subsistema vacío con máscara y OpenFcn
add_block('built-in/SubSystem',[mdl '/Exportar_Claude'], ...
    'Position',pos('Exportar_Claude',[760,420,880,460]));
set_param([mdl '/Exportar_Claude'], 'OpenFcn', ...
    'addpath(fileparts(get_param(bdroot,''FileName''))); run(''export_model_state'');');
mk = Simulink.Mask.create([mdl '/Exportar_Claude']);
mk.IconUnits    = 'normalized';
mk.Display      = sprintf('color(''blue'');\ntext(0.5,0.5,''>> Claude'');');

%% ── SCRIPTS MATLAB FUNCTION ──────────────────────────────────────────────
set_script(mdl,'Plant', [...
    "function xdot = Plant(x, u, d, p_phys)", ...
    "xdot = twip_plant_fcn(x, u, d, p_phys);", ...
    "end"]);

set_script(mdl,'Controller', [...
    "function u = Controller(x, K_lqr)", ...
    "u = twip_ctrl_lqr(x, K_lqr);", ...
    "end"]);

set_script(mdl,'Bache', [...
    "function d_pitch = Bache(x_pos, x_dot, p_bache)", ...
    "d_pitch = twip_bache_fcn(x_pos, x_dot, p_bache);", ...
    "end"]);

%% Aplicar orientación y mirror guardados a todos los bloques
apply_transform(mdl, bp);

%% ── CONEXIONES ───────────────────────────────────────────────────────────
% L solo crea la línea si el usuario no la eliminó explícitamente
L = @(s,d) safe_line(mdl, s, d, is_deleted);

% Lazo dinámico principal
L('Plant/1',      'Integrator/1');
L('Integrator/1', 'Plant/1');
L('Integrator/1', 'Demux_x/1');
L('Integrator/1', 'Controller/1');

% Parámetros físicos → Plant
L('P_phys/1',  'Plant/4');

% Control
L('Kc/1',         'Controller/2');
L('Controller/1', 'Plant/2');
L('Controller/1', 'Demux_u/1');

% Perturbaciones
L('Bache/1',     'Sum_pitch/1');
L('D1/1',        'Sum_pitch/2');
L('Sum_pitch/1', 'Mux_d/1');
L('D2/1',        'Mux_d/2');
L('Mux_d/1',     'Plant/3');

% Entradas del Bache
L('Demux_x/1', 'Bache/1');
L('Demux_x/4', 'Bache/2');
L('P_bache/1', 'Bache/3');

% Terminar psi y psi_dot sin usar
L('Demux_x/3', 'Term_psi/1');
L('Demux_x/6', 'Term_psi_dot/1');

% Scopes con nombres de señal
lh = L('Demux_x/2','Scope_tilt/1'); if lh~=0, set_param(lh,'Name','theta [rad]');     end
lh = L('Demux_x/5','Scope_tilt/2'); if lh~=0, set_param(lh,'Name','theta_dot [rad/s]'); end
lh = L('Demux_u/1','Scope_ctrl/1'); if lh~=0, set_param(lh,'Name','u1 [Nm]');         end
lh = L('Demux_u/2','Scope_ctrl/2'); if lh~=0, set_param(lh,'Name','u2 [Nm]');         end

%% Configurar scopes (API 2024b+)
cfg_scope(mdl,'Scope_tilt','Cabeceo');
cfg_scope(mdl,'Scope_ctrl','Torques de control');

%% Restaurar routing exacto de líneas base (si está guardado)
if exist(pos_file, 'file')
    tmp3 = load(pos_file);
    if isfield(tmp3, 'all_connections')
        ac = tmp3.all_connections;
        all_lines = find_system(mdl, 'SearchDepth',1,'FindAll','on','Type','line');
        for ci = 1:numel(ac)
            try
                pts = ac{ci}.points;
                if numel(pts) == 0
                    continue
                end
                for li = 1:numel(all_lines)
                    lh = all_lines(li);
                    sn = get_param(get_param(lh,'SrcBlockHandle'),'Name');
                    dn = get_param(get_param(lh,'DstBlockHandle'),'Name');
                    sp = get_param(get_param(lh,'SrcPortHandle'),'PortNumber');
                    dp = get_param(get_param(lh,'DstPortHandle'),'PortNumber');
                    if strcmp(sn,ac{ci}.src) && strcmp(dn,ac{ci}.dst) && sp==ac{ci}.sp && dp==ac{ci}.dp
                        set_param(lh,'Points', pts);
                        break;
                    end
                end
            catch
            end
        end
    end
end

%% Recrear bloques de usuario guardados en block_positions.mat
if exist(pos_file, 'file')
    tmp2 = load(pos_file);
    if isfield(tmp2, 'user_extra')
        ue = tmp2.user_extra;
        fields = fieldnames(ue);
        for fi = 1:numel(fields)
            e     = ue.(fields{fi});
            bpath = [mdl '/' e.name];
            % Elegir librería: usar ReferenceBlock si existe, si no mapear por tipo
            if ~isempty(e.ref)
                libsrc = e.ref;
            else
                libsrc = type2lib(e.type);
            end
            if isempty(libsrc), continue; end
            add_block(libsrc, bpath, 'Position', e.position);
            % Aplicar parámetros extra
            pnames = fieldnames(e.params);
            for pi = 1:numel(pnames)
                try
                    set_param(bpath, pnames{pi}, e.params.(pnames{pi}));
                catch
                end
            end
        end
        % Recrear conexiones de usuario con coordenadas exactas
        if isfield(tmp2, 'user_connections')
            uc = tmp2.user_connections;
            for ci = 1:numel(uc)
                try
                    lh = add_line(mdl, ...
                        sprintf('%s/%d', uc{ci}.src, uc{ci}.sp), ...
                        sprintf('%s/%d', uc{ci}.dst, uc{ci}.dp));
                    if isfield(uc{ci},'points') && ~isempty(uc{ci}.points)
                        set_param(lh, 'Points', uc{ci}.points);
                    end
                    if isfield(uc{ci},'signame') && ~isempty(uc{ci}.signame)
                        set_param(lh, 'Name', uc{ci}.signame);
                    end
                catch
                end
            end
        end
        fprintf('Bloques de usuario recreados: %d\n', numel(fields));
    end
end

%% Guardar
save_system(mdl);
open_system(mdl);
fprintf('\n%s listo — presiona RUN.\n', mdl);
fprintf('Para nuevo experimento: edita twip_params.m y corre de nuevo (no rebuild).\n\n');

%% ═══════════════ HELPERS LOCALES ═════════════════════════════════════════

function add_mfcn(mdl, name, pos)
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [mdl '/' name], 'Position', pos);
end

function add_const(mdl, name, val, pos)
    add_block('simulink/Sources/Constant', [mdl '/' name], ...
              'Position', pos, 'Value', val);
end

function add_scope(mdl, name, pos, nports)
    add_block('simulink/Sinks/Scope', [mdl '/' name], ...
              'Position', pos, 'NumInputPorts', num2str(nports));
end

function set_script(mdl, block, lines)
    rt    = sfroot();
    m     = rt.find('-isa','Simulink.BlockDiagram','Name', mdl);
    chart = m.find('-isa','Stateflow.EMChart','Path', [mdl '/' block]);
    chart.Script = strjoin(lines, newline);
end

function lh = safe_line(mdl, src, dst, is_deleted)
% Crea línea solo si el usuario no la eliminó; devuelve 0 si omitida
    lh = 0;
    parts_s = strsplit(src, '/');  parts_d = strsplit(dst, '/');
    sname = parts_s{1};  sp = str2double(parts_s{2});
    dname = parts_d{1};  dp = str2double(parts_d{2});
    if is_deleted(sname, sp, dname, dp)
        return
    end
    try
        lh = add_line(mdl, src, dst, 'autorouting','on');
    catch
    end
end

function result = check_deleted(deleted_cons, s, sp, d, dp)
% Devuelve true si la conexión está en la lista de eliminadas
    result = false;
    for i = 1:numel(deleted_cons)
        dc = deleted_cons{i};
        if strcmp(dc.src,s) && dc.sp==sp && strcmp(dc.dst,d) && dc.dp==dp
            result = true; return;
        end
    end
end

function apply_transform(mdl, bp)
% Aplica orientación y mirror guardados a todos los bloques
    fields = fieldnames(bp);
    for i = 1:numel(fields)
        fname = fields{i};
        entry = bp.(fname);
        % Reconstruir nombre original (makeValidName reemplaza espacios por _)
        bpath = [mdl '/' strrep(fname,'_',' ')];
        % Intentar con el nombre tal cual si el anterior no existe
        if ~bdIsLoaded(mdl), return; end
        try, get_param(bpath,'Position'); catch
            bpath = [mdl '/' fname];
            try, get_param(bpath,'Position'); catch, continue; end
        end
        try
            if isfield(entry,'orientation') && ~isempty(entry.orientation)
                set_param(bpath,'Orientation', entry.orientation);
            end
        catch
        end
        try
            if isfield(entry,'mirror') && ~isempty(entry.mirror)
                set_param(bpath,'BlockMirror', entry.mirror);
            end
        catch
        end
    end
end

function lib = type2lib(btype)
    switch btype
        case 'Step',        lib = 'simulink/Sources/Step';
        case 'Scope',       lib = 'simulink/Sinks/Scope';
        case 'Constant',    lib = 'simulink/Sources/Constant';
        case 'Gain',        lib = 'simulink/Math Operations/Gain';
        case 'Sum',         lib = 'simulink/Math Operations/Sum';
        case 'Mux',         lib = 'simulink/Signal Routing/Mux';
        case 'Demux',       lib = 'simulink/Signal Routing/Demux';
        case 'Integrator',  lib = 'simulink/Continuous/Integrator';
        case 'Terminator',  lib = 'simulink/Sinks/Terminator';
        case 'SubSystem',   lib = 'built-in/SubSystem';
        otherwise,          lib = '';
    end
end

function p = get_saved_pos(bp, name, default)
    fname = matlab.lang.makeValidName(name);
    if isfield(bp, fname)
        entry = bp.(fname);
        % block_pos ahora es struct con campo .position
        if isstruct(entry) && isfield(entry, 'position')
            p = entry.position;
        else
            p = entry;   % compatibilidad con formato anterior
        end
    else
        p = default;
    end
end

function cfg_scope(mdl, name, title_str)
    try
        sc = get_param([mdl '/' name], 'ScopeConfiguration');
        sc.Title = title_str;
        sc.ShowLegend = true;
        sc.OpenAtSimulationStart = true;
    catch
        try
            set_param([mdl '/' name], 'ShowLegends', 'on');
        catch
        end
    end
end