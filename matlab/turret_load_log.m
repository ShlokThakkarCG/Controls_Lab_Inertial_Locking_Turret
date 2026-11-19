function L = turret_load_log(csvPath)
%TURRET_LOAD_LOG  Load a turret log (CSV + optional _raw.txt) into a struct.
%   L = turret_load_log("logs/20261120_024928_lead_test_v3.csv")
%   L.t       time [s] from the first sample
%   L.<col>   every CSV column as a column vector (t_ms, mode, locked, err_tot, ...)
%   L.names   column names
%   L.events  table of firmware events (t [s], kind, text) if the _raw.txt exists
%   L.name    file stem
    csvPath = string(csvPath);
    fid = fopen(csvPath, 'r');
    if fid < 0, error("turret_load_log:open", "Cannot open %s", csvPath); end
    hdr = string(fgetl(fid));
    fclose(fid);
    names = split(hdr, ",")';
    M = readmatrix(csvPath, 'NumHeaderLines', 1);
    L = struct();
    L.names = names;
    [~, L.name] = fileparts(csvPath);
    for k = 1:numel(names)
        L.(matlab.lang.makeValidName(names(k))) = M(:, k);
    end
    L.t = (L.t_ms - L.t_ms(1)) / 1000;

    % events from the raw log (lines like "   12.345 << E,123456,lock,mode=3,...")
    L.events = table('Size', [0 3], 'VariableTypes', {'double', 'string', 'string'}, ...
                     'VariableNames', {'t', 'kind', 'text'});
    rawPath = replace(csvPath, ".csv", "_raw.txt");
    if isfile(rawPath)
        raw = readlines(rawPath);
        ev = raw(contains(raw, "<< E,"));
        for k = 1:numel(ev)
            body = extractAfter(ev(k), "<< E,");
            parts = split(body, ",");
            if numel(parts) >= 2
                tm = (str2double(parts(1)) - L.t_ms(1)) / 1000;
                L.events = [L.events; {tm, parts(2), strjoin(parts(3:end), ",")}]; %#ok<AGROW>
            end
        end
    end
end
