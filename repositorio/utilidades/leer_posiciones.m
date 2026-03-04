%% leer_posiciones.m
%  Muestra en consola las posiciones de todos los bloques del modelo abierto.
%  Corre esto despues de reorganizar bloques manualmente en Simulink.
%  Copia la salida y pégala en Simscape_LQR_v41.m para que el rebuild
%  respete tu disposicion.
% =========================================================================

modelName = 'Segway_Testbench_v41';

if ~bdIsLoaded(modelName)
    error('El modelo %s no esta abierto. Abrelo primero.', modelName);
end

bloques = find_system(modelName, 'SearchDepth', 1, 'Type', 'block');

fprintf('\n=========================================================\n');
fprintf(' POSICIONES DE BLOQUES — %s\n', modelName);
fprintf('=========================================================\n');
fprintf('%-35s  %-22s  %s\n', 'BLOQUE', 'POSITION', 'ICONSHAPE');
fprintf('%s\n', repmat('-',1,80));

for i = 1:numel(bloques)
    nombre = strrep(bloques{i}, [modelName '/'], '');
    pos    = get_param(bloques{i}, 'Position');
    try
        shape = get_param(bloques{i}, 'IconShape');
    catch
        shape = '-';
    end
    fprintf('%-35s  [%-18s  %s\n', nombre, ...
        sprintf('%d %d %d %d]', pos(1), pos(2), pos(3), pos(4)), shape);
end

fprintf('=========================================================\n');
fprintf(' Total: %d bloques\n', numel(bloques));
fprintf('=========================================================\n');
