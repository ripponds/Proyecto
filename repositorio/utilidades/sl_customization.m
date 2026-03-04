function sl_customization(cm)
% Agrega botón "Exportar → Claude" a la barra Tools del modelo TWIP_NL
    cm.addCustomMenuFcn('Simulink:ToolsMenu', @getExportItem);
end

function schema = getExportItem(~)
    schema = sl_action_schema;
    schema.label  = 'Exportar estado → Claude';
    schema.callback = @exportCallback;
end

function exportCallback(callbackInfo)
    mdl    = get_param(callbackInfo.model, 'Name');
    slxfn  = get_param(mdl, 'FileName');
    if isempty(slxfn)
        % Modelo sin guardar: buscar el script en el path de MATLAB
        script = which('export_model_state.m');
    else
        script = fullfile(fileparts(slxfn), 'export_model_state.m');
    end
    if exist(script, 'file')
        run(script);
    else
        errordlg('No se encontró export_model_state.m junto al .slx ni en el path.', ...
                 'Exportar Claude');
    end
end