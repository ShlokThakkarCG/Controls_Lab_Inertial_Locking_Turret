classdef TurretLink < handle
    % TurretLink  Serial connection to the turret firmware (turret_stab / turret_probe).
    %   The firmware prints line-based telemetry:
    %     H,<columns>   header for D lines      D,<values>   one data sample
    %     S,<k=v,...>   1 Hz health             E,<t>,<event,...>   events      # text
    %   poll() drains whatever has arrived and returns it parsed.
    %
    %   link = TurretLink("COM7"); link.connect();
    %   [D, S, E, msgs] = link.poll();   % D: numeric rows, columns in link.Cols
    %   link.send("l");                   % any firmware command
    %   link.startRecording("mytest");   link.stopRecording();

    properties
        Port string
        Baud double = 921600
        Header string = string.empty     % column names from the H line
        Cols struct = struct()           % Cols.<name> = column index
        LastStatus struct = struct()     % parsed fields of the latest S line
        IsRecording logical = false
        RecordPath string = ""
    end

    properties (Access = private)
        S                                 % serialport object
        Buffer char = ''
        CsvFid double = -1
        RawFid double = -1
        HeaderWritten logical = false
        T0 uint64
    end

    methods
        function obj = TurretLink(port)
            if nargin < 1 || strlength(string(port)) == 0
                port = TurretLink.findPort();
            end
            obj.Port = string(port);
        end

        function connect(obj)
            obj.S = serialport(obj.Port, obj.Baud, "Timeout", 1);
            % The ESP32's native USB port resets the chip when RTS is high while DTR
            % is low. Release RTS first so that state never occurs.
            try, setRTS(obj.S, false); setDTR(obj.S, false); catch, end
            configureTerminator(obj.S, "LF");
            flush(obj.S);
            obj.Buffer = '';
            obj.T0 = tic;
            obj.send("stream on");        % makes the firmware re-print its header
            obj.send("info");
        end

        function disconnect(obj)
            obj.stopRecording();
            if ~isempty(obj.S)
                delete(obj.S);
                obj.S = [];
            end
        end

        function tf = isConnected(obj)
            tf = ~isempty(obj.S) && isvalid(obj.S);
        end

        function send(obj, cmd)
            if ~obj.isConnected(), return; end
            writeline(obj.S, char(cmd));
            obj.writeRaw(">> " + string(cmd));
        end

        function [D, S, E, msgs] = poll(obj)
            D = zeros(0, max(1, numel(obj.Header)));
            S = string.empty; E = string.empty; msgs = string.empty;
            if ~obj.isConnected(), return; end
            n = obj.S.NumBytesAvailable;
            if n > 0
                obj.Buffer = [obj.Buffer, read(obj.S, n, "char")];
            end
            nl = find(obj.Buffer == newline, 1, 'last');
            if isempty(nl), return; end
            chunk = obj.Buffer(1:nl);
            obj.Buffer = obj.Buffer(nl+1:end);
            lines = strtrim(splitlines(string(chunk)));
            lines = lines(strlength(lines) > 0);
            [D, S, E, msgs] = obj.parseLines(lines);
        end

        function [D, S, E, msgs] = parseLines(obj, lines)
            % Parse an array of telemetry lines (also used for offline replay).
            isD = startsWith(lines, "D,");
            isH = startsWith(lines, "H,");
            isS = startsWith(lines, "S,");
            isE = startsWith(lines, "E,");
            for h = find(isH)'
                obj.setHeader(extractAfter(lines(h), 2));
            end
            S = lines(isS); E = lines(isE);
            msgs = lines(~(isD | isH | isS | isE));
            if ~isempty(S), obj.LastStatus = TurretLink.parseStatus(S(end)); end
            for k = 1:numel(lines), if ~isD(k), obj.writeRaw("<< " + lines(k)); end, end
            dl = extractAfter(lines(isD), 2);
            nc = numel(obj.Header);
            D = zeros(0, max(1, nc));
            if isempty(dl) || nc == 0, return; end
            vals = sscanf(char(strjoin(dl, ",") + ","), '%f,');
            if numel(vals) == nc * numel(dl)
                D = reshape(vals, nc, [])';
            else                                           % a damaged line: parse one by one
                rows = cell(numel(dl), 1);
                for k = 1:numel(dl)
                    v = sscanf(char(dl(k) + ","), '%f,');
                    if numel(v) == nc, rows{k} = v'; end
                end
                D = vertcat(rows{:});
                dl = dl(~cellfun(@isempty, rows));
            end
            if obj.IsRecording && obj.HeaderWritten
                fprintf(obj.CsvFid, '%s\n', dl);
                for k = 1:numel(dl), obj.writeRaw("<< D," + dl(k)); end
            end
        end

        function setHeader(obj, hdr)
            hdr = string(hdr);
            names = split(hdr, ",")';
            if ~isequal(names, obj.Header)
                obj.Header = names;
                obj.Cols = struct();
                for k = 1:numel(names)
                    obj.Cols.(matlab.lang.makeValidName(names(k))) = k;
                end
            end
            if obj.IsRecording && ~obj.HeaderWritten
                fprintf(obj.CsvFid, '%s\n', hdr);
                obj.HeaderWritten = true;
            end
        end

        function path = startRecording(obj, tag)
            % Same file layout as tools/turret_console.py, so all analysis tools work.
            if nargin < 2, tag = "matlab"; end
            obj.stopRecording();
            logDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'logs');
            if ~isfolder(logDir), mkdir(logDir); end
            stamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            base = fullfile(logDir, stamp + "_" + tag);
            obj.CsvFid = fopen(base + ".csv", 'w');
            obj.RawFid = fopen(base + "_raw.txt", 'w');
            obj.RecordPath = base + ".csv";
            obj.IsRecording = true;
            obj.HeaderWritten = false;
            if ~isempty(obj.Header)
                obj.setHeader(strjoin(obj.Header, ","));
            end
            obj.send("stream on");                  % re-print the header into the new log
            path = obj.RecordPath;
        end

        function stopRecording(obj)
            if obj.CsvFid > 0, fclose(obj.CsvFid); end
            if obj.RawFid > 0, fclose(obj.RawFid); end
            obj.CsvFid = -1; obj.RawFid = -1;
            obj.IsRecording = false;
        end

        function delete(obj)
            obj.disconnect();
        end
    end

    methods (Access = private)
        function writeRaw(obj, txt)
            if obj.IsRecording && obj.RawFid > 0
                fprintf(obj.RawFid, '%10.3f %s\n', toc(obj.T0), txt);
            end
        end
    end

    methods (Static)
        function port = findPort()
            % Prefer the ESP32's native USB port (USB VID 303A shows as "USB Serial Device").
            ports = serialportlist("available");
            if isempty(ports)
                error("TurretLink:noPort", "No serial ports found. Is the turret plugged in?");
            end
            port = ports(end);
        end

        function st = parseStatus(line)
            st = struct();
            parts = split(extractAfter(line, 2), ",");
            st.t_ms = str2double(parts(1));
            for k = 2:numel(parts)
                kv = split(parts(k), "=");
                if numel(kv) == 2
                    st.(matlab.lang.makeValidName(kv(1))) = kv(2);
                end
            end
        end
    end
end
