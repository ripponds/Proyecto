% =========================================================================
%  export_model_state.m — Guarda estado del modelo para build_sim.m
%  Doble-click en bloque ">> Claude" para ejecutar.
%
%  Guarda en block_positions.mat:
%    1. Posición de TODOS los bloques  (block_pos)
%    2. Bloques nuevos que el usuario agregó  (user_extra)
%    3. Conexiones de/hacia esos bloques nuevos  (user_connections)
% =========================================================================

mdl      = 'TWIP_NL';
proj_dir = fileparts(which([mdl '.slx']));
if isempty(proj_dir), proj_dir = pwd; end

original_blocks = {'Integrator','Plant','Controller','Bache','Kc','P_phys', ...
                   'P_bache','D1','D2','Sum_pitch','Mux_d','Demux_x','Demux_u', ...
                   'Scope_tilt','Scope_ctrl','Term_psi','Term_psi_dot','Exportar_Claude'};

blocks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');

%% 1. Posiciones de todos los bloques
block_pos = struct();
for i = 1:numel(blocks)
    bname = get_param(blocks{i}, 'Name');
    fname = matlab.lang.makeValidName(bname);
    entry.position    = get_param(blocks{i}, 'Position');
    entry.orientation = ''; try, entry.orientation = get_param(blocks{i},'Orientation');  catch, end
    entry.mirror      = ''; try, entry.mirror      = get_param(blocks{i},'BlockMirror');  catch, end
    block_pos.(fname) = entry;
end

%% 2. Bloques nuevos del usuario
user_extra       = struct();
user_block_names = {};
for i = 1:numel(blocks)
    bname = get_param(blocks{i}, 'Name');
    if ismember(bname, original_blocks), continue; end

    fname = matlab.lang.makeValidName(bname);
    btype = get_param(blocks{i}, 'BlockType');
    bref  = ''; try, bref = get_param(blocks{i},'ReferenceBlock'); catch, end

    e.name        = bname;
    e.type        = btype;
    e.ref         = bref;
    e.position    = get_param(blocks{i}, 'Position');
    e.orientation = ''; try, e.orientation = get_param(blocks{i},'Orientation');  catch, end
    e.mirror      = ''; try, e.mirror      = get_param(blocks{i},'BlockMirror');  catch, end
    e.params      = struct();
    try
        switch btype
            case 'Constant', e.params.Value        = get_param(blocks{i},'Value');
            case 'Scope',    e.params.NumInputPorts = get_param(blocks{i},'NumInputPorts');
            case 'Step',     e.params.Time          = get_param(blocks{i},'Time');
                             e.params.Before        = get_param(blocks{i},'Before');
                             e.params.After         = get_param(blocks{i},'After');
            case 'Gain',     e.params.Gain          = get_param(blocks{i},'Gain');
            case 'Mux',      e.params.Inputs        = get_param(blocks{i},'Inputs');
            case 'Demux',    e.params.Outputs       = get_param(blocks{i},'Outputs');
        end
    catch
    end

    user_extra.(fname)      = e;
    user_block_names{end+1} = bname; %#ok<AGROW>
end

%% 3. Conexiones que involucran bloques nuevos
user_connections = {};
lines = find_system(mdl, 'SearchDepth',1,'FindAll','on','Type','line');
for i = 1:numel(lines)
    lh = lines(i);
    try
        sname = get_param(get_param(lh,'SrcBlockHandle'),'Name');
        dname = get_param(get_param(lh,'DstBlockHandle'),'Name');
        if ~ismember(sname, user_block_names) && ~ismember(dname, user_block_names)
            continue
        end
        sp      = get_param(get_param(lh,'SrcPortHandle'),'PortNumber');
        dp      = get_param(get_param(lh,'DstPortHandle'),'PortNumber');
        pts     = []; try, pts = get_param(lh,'Points'); catch, end
        signame = ''; try, signame = get_param(lh,'Name'); catch, end
        user_connections{end+1} = struct('src',sname,'sp',sp,'dst',dname,'dp',dp,...
                                         'points',pts,'signame',signame); %#ok<AGROW>
    catch
    end
end

%% 4. Conexiones originales que el usuario eliminó
original_connections = {
    'Plant',      1, 'Integrator',   1;
    'Integrator', 1, 'Plant',        1;
    'Integrator', 1, 'Demux_x',      1;
    'Integrator', 1, 'Controller',   1;
    'P_phys',     1, 'Plant',        4;
    'Kc',         1, 'Controller',   2;
    'Controller', 1, 'Plant',        2;
    'Controller', 1, 'Demux_u',      1;
    'Bache',      1, 'Sum_pitch',    1;
    'D1',         1, 'Sum_pitch',    2;
    'Sum_pitch',  1, 'Mux_d',        1;
    'D2',         1, 'Mux_d',        2;
    'Mux_d',      1, 'Plant',        3;
    'Demux_x',    1, 'Bache',        1;
    'Demux_x',    4, 'Bache',        2;
    'P_bache',    1, 'Bache',        3;
    'Demux_x',    3, 'Term_psi',     1;
    'Demux_x',    6, 'Term_psi_dot', 1;
    'Demux_x',    2, 'Scope_tilt',   1;
    'Demux_x',    5, 'Scope_tilt',   2;
    'Demux_u',    1, 'Scope_ctrl',   1;
    'Demux_u',    2, 'Scope_ctrl',   2;
};

% Snapshot plano de todas las conexiones actuales para comparar
all_src = {};  all_sp = [];  all_dst = {};  all_dp = [];
for i = 1:numel(lines)
    lh = lines(i);
    try
        sname  = get_param(get_param(lh,'SrcBlockHandle'),'Name');
        sp_num = get_param(get_param(lh,'SrcPortHandle'), 'PortNumber');
        % DstPortHandle puede ser array si la línea está ramificada
        dst_ph = get_param(lh,'DstPortHandle');
        for di = 1:numel(dst_ph)
            if dst_ph(di) == -1, continue; end
            all_src{end+1} = sname;              %#ok<AGROW>
            all_sp(end+1)  = sp_num;             %#ok<AGROW>
            all_dst{end+1} = get_param(get_param(dst_ph(di),'Parent'),'Name'); %#ok<AGROW>
            all_dp(end+1)  = get_param(dst_ph(di),'PortNumber');               %#ok<AGROW>
        end
    catch
    end
end

deleted_connections = {};
for r = 1:size(original_connections,1)
    os = original_connections{r,1};  osp = original_connections{r,2};
    od = original_connections{r,3};  odp = original_connections{r,4};
    found = false;
    for c = 1:numel(all_src)
        if strcmp(all_src{c},os) && all_sp(c)==osp && strcmp(all_dst{c},od) && all_dp(c)==odp
            found = true; break;
        end
    end
    if ~found
        deleted_connections{end+1} = struct('src',os,'sp',osp,'dst',od,'dp',odp); %#ok<AGROW>
    end
end

%% Guardar
mat_out = fullfile(proj_dir, 'block_positions.mat');
save(mat_out, 'block_pos', 'user_extra', 'user_connections', 'deleted_connections');

nu = numel(fieldnames(user_extra));
nc = numel(user_connections);
nd = numel(deleted_connections);
fprintf('Guardado → block_positions.mat\n');
fprintf('  Bloques nuevos: %d  |  Conexiones nuevas: %d  |  Eliminadas: %d\n', nu, nc, nd);