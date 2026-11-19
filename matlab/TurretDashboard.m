classdef TurretDashboard < handle
    % TurretDashboard  Live control & visualisation of the ENR 312 stabilized turret.
    %
    %   TurretDashboard            % open the app (pick the port inside)
    %   TurretDashboard("COM7")    % open and connect straight away
    %
    %   * 3D mock-up of the turret moving with the real hardware
    %     (green ray = locked direction, red dashed = barrel direction from its IMU)
    %   * live plots: base disturbance vs pointing error, error components, joints
    %   * live score: RMS pointing error, base motion, % reduction (last 10 s locked)
    %   * controls: LOCK, mode, lead, feedback gains, center, torque
    %   * Record: writes logs/<stamp>_<tag>.csv + _raw.txt (same format as the Python tools)
    %   * Replay: play any saved log through the 3D model and plots (no hardware needed)

    properties (Access = private)
        Fig; Grid
        % controls
        PortDrop; ConnectBtn; StatusLamp; StatusLbl
        LockBtn; ModeDrop; LeadSlider; LeadLbl; KpField; KiField
        RecordBtn; TagField; RecordLbl; ReplayBtn; ReplaySpeed
        HealthLbl; ScoreLbl; ModeLbl
        % axes
        Ax3D; AxTop; AxMid; AxBot
        hBase; hErr; hErrAz; hErrEl; hCmdAz; hMeasAz; hCmdEl; hMeasEl; hLockShade
        Model
        % data
        Link; State; Timer
        Replay struct = struct('active', false)
        UiTic uint64 = tic
    end

    methods
        function app = TurretDashboard(port)
            app.buildUi();
            app.State = TurretState(struct());
            app.Timer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.05, ...
                              'BusyMode', 'drop', 'TimerFcn', @(~, ~) app.tick(), ...
                              'ErrorFcn', @(~, e) disp(e.Data.message));
            start(app.Timer);
            if nargin >= 1 && strlength(string(port)) > 0
                app.PortDrop.Items = unique([string(port), app.PortDrop.Items]);
                app.PortDrop.Value = string(port);
                app.onConnect();
            end
        end

        function delete(app)
            app.onClose();
        end

        function loadReplay(app, csvPath)
            % Play a saved log through the 3D model and plots (scriptable demo).
            csvPath = char(csvPath);
            fid = fopen(csvPath); hdr = fgetl(fid); fclose(fid);
            M = readmatrix(csvPath, 'NumHeaderLines', 1);
            tmp = TurretLink("none");
            tmp.setHeader(hdr);
            if ~isfield(tmp.Cols, 'bq_w')
                uialert(app.Fig, 'This log has no IMU quaternion columns.', 'Replay');
                return;
            end
            app.State = TurretState(tmp.Cols);
            [~, f, e] = fileparts(csvPath);
            app.Replay = struct('active', true, 'M', M, 'cols', tmp.Cols, 'i', 1, ...
                                't0', tic, 'tlog0', M(1, tmp.Cols.t_ms) / 1000, 'name', string([f e]));
            app.StatusLamp.Color = [0.3 0.55 0.95];
            app.StatusLbl.Text = "replay: " + f;
        end

        function snapshot(app, pngPath)
            exportapp(app.Fig, pngPath);
        end
    end

    % =========================== UI ===========================
    methods (Access = private)
        function buildUi(app)
            app.Fig = uifigure('Name', 'ENR 312 — Inertially Stabilized Turret', ...
                               'Position', [60 60 1500 880], 'Color', [0.97 0.97 0.98], ...
                               'CloseRequestFcn', @(~, ~) app.onClose());
            try, app.Fig.Theme = 'light'; catch, end
            app.Grid = uigridlayout(app.Fig, [3 3], 'ColumnWidth', {270, '1.15x', '1x'}, ...
                                    'RowHeight', {'1x', '1x', '1x'}, 'Padding', [10 10 10 10], ...
                                    'ColumnSpacing', 10, 'RowSpacing', 8);

            % ---------- left control panel ----------
            p = uipanel(app.Grid, 'Title', 'Turret control', 'FontWeight', 'bold');
            p.Layout.Row = [1 3]; p.Layout.Column = 1;
            g = uigridlayout(p, [19 2], 'RowHeight', repmat({'fit'}, 1, 19), ...
                             'ColumnWidth', {'1x', '1x'}, 'RowSpacing', 6);

            uilabel(g, 'Text', 'Serial port', 'FontWeight', 'bold');
            uibutton(g, 'Text', 'Refresh', 'ButtonPushedFcn', @(~, ~) app.refreshPorts());
            app.PortDrop = uidropdown(g, 'Items', {'(none)'});
            app.PortDrop.Layout.Column = [1 2];
            app.ConnectBtn = uibutton(g, 'Text', 'Connect', 'ButtonPushedFcn', @(~, ~) app.onConnect());
            app.ConnectBtn.Layout.Column = [1 2];
            app.StatusLamp = uilamp(g, 'Color', [0.6 0.6 0.6]);
            app.StatusLbl = uilabel(g, 'Text', 'disconnected');

            app.LockBtn = uibutton(g, 'Text', 'LOCK', 'FontSize', 20, 'FontWeight', 'bold', ...
                                   'BackgroundColor', [0.85 0.87 0.9], 'ButtonPushedFcn', @(~, ~) app.cmd("l"));
            app.LockBtn.Layout.Column = [1 2];
            app.ModeLbl = uilabel(g, 'Text', 'state: —', 'FontWeight', 'bold');
            app.ModeLbl.Layout.Column = [1 2];

            uilabel(g, 'Text', 'Control mode');
            app.ModeDrop = uidropdown(g, 'Items', {'0 HOLD', '1 FF', '2 FF+LEAD', '3 FULL', '4 FB only'}, ...
                                      'Value', '3 FULL', 'ValueChangedFcn', @(s, ~) app.cmd("m " + extractBefore(string(s.Value), " ")));
            app.LeadLbl = uilabel(g, 'Text', 'Lead: 110 ms');
            app.LeadLbl.Layout.Column = [1 2];
            app.LeadSlider = uislider(g, 'Limits', [0 200], 'Value', 110, 'MajorTicks', 0:50:200, ...
                                      'ValueChangedFcn', @(s, ~) app.onLead(s.Value));
            app.LeadSlider.Layout.Column = [1 2];
            uilabel(g, 'Text', 'Kp (FB)');
            app.KpField = uieditfield(g, 'numeric', 'Value', 0.30, 'ValueChangedFcn', @(s, ~) app.cmd(sprintf("kp %.3f", s.Value)));
            uilabel(g, 'Text', 'Ki (FB) [1/s]');
            app.KiField = uieditfield(g, 'numeric', 'Value', 2.0, 'ValueChangedFcn', @(s, ~) app.cmd(sprintf("ki %.3f", s.Value)));
            uibutton(g, 'Text', 'Center (0,0)', 'ButtonPushedFcn', @(~, ~) app.cmd("c"));
            uibutton(g, 'Text', 'Torque OFF', 'ButtonPushedFcn', @(~, ~) app.cmd("off"));

            uilabel(g, 'Text', 'Log tag');
            app.TagField = uieditfield(g, 'text', 'Value', 'dashboard');
            app.RecordBtn = uibutton(g, 'state', 'Text', 'Record', 'ValueChangedFcn', @(s, ~) app.onRecord(s.Value));
            app.RecordBtn.Layout.Column = [1 2];
            app.RecordLbl = uilabel(g, 'Text', 'not recording', 'FontSize', 10, 'WordWrap', 'on');
            app.RecordLbl.Layout.Column = [1 2];
            app.ReplayBtn = uibutton(g, 'Text', 'Replay a log…', 'ButtonPushedFcn', @(~, ~) app.onReplay());
            app.ReplaySpeed = uidropdown(g, 'Items', {'0.5x', '1x', '2x'}, 'Value', '1x');

            h = uilabel(g, 'Text', 'Live score (last 10 s locked)', 'FontWeight', 'bold'); h.Layout.Column = [1 2];
            app.ScoreLbl = uilabel(g, 'Text', '—', 'FontName', 'Consolas', 'FontSize', 13, 'WordWrap', 'on');
            app.ScoreLbl.Layout.Column = [1 2];
            h = uilabel(g, 'Text', 'Health', 'FontWeight', 'bold'); h.Layout.Column = [1 2];
            app.HealthLbl = uilabel(g, 'Text', '—', 'FontName', 'Consolas', 'FontSize', 11, 'WordWrap', 'on');
            app.HealthLbl.Layout.Column = [1 2];

            % ---------- 3D model ----------
            app.Ax3D = uiaxes(app.Grid);
            app.Ax3D.Layout.Row = [1 3]; app.Ax3D.Layout.Column = 2;
            title(app.Ax3D, 'Live turret (green = locked direction, red = barrel IMU)');
            app.Model = TurretModel3D(app.Ax3D);

            % ---------- plots ----------
            app.AxTop = TurretDashboard.makeAxes(app.Grid, 1, 'Base disturbance vs barrel pointing error (green band = locked)', 'angle [deg]');
            app.hLockShade = area(app.AxTop, nan, nan, 'FaceColor', [0.2 0.7 0.3], 'FaceAlpha', 0.08, ...
                                  'EdgeColor', 'none', 'HandleVisibility', 'off');
            app.hBase = plot(app.AxTop, nan, nan, 'Color', [0.55 0.55 0.55], 'LineWidth', 1.4);
            app.hErr  = plot(app.AxTop, nan, nan, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.8);
            legend(app.AxTop, {'base rotation since lock', 'pointing error'}, 'Location', 'northwest');
            app.AxMid = TurretDashboard.makeAxes(app.Grid, 2, 'Pointing error components (barrel frame)', 'error [deg]');
            app.hErrAz = plot(app.AxMid, nan, nan, 'Color', [0.0 0.45 0.74], 'LineWidth', 1.3);
            app.hErrEl = plot(app.AxMid, nan, nan, 'Color', [0.93 0.55 0.1], 'LineWidth', 1.3);
            legend(app.AxMid, {'azimuth', 'elevation'}, 'Location', 'northwest');
            app.AxBot = TurretDashboard.makeAxes(app.Grid, 3, 'Joints: command vs measured', 'joint [deg]');
            app.hCmdAz  = plot(app.AxBot, nan, nan, 'Color', [0.0 0.45 0.74], 'LineWidth', 1.4);
            app.hMeasAz = plot(app.AxBot, nan, nan, '--', 'Color', [0.0 0.45 0.74]);
            app.hCmdEl  = plot(app.AxBot, nan, nan, 'Color', [0.93 0.55 0.1], 'LineWidth', 1.4);
            app.hMeasEl = plot(app.AxBot, nan, nan, '--', 'Color', [0.93 0.55 0.1]);
            legend(app.AxBot, {'az cmd', 'az meas', 'el cmd', 'el meas'}, 'Location', 'northwest', 'NumColumns', 2);
            xlabel(app.AxBot, 'time [s]');
            app.refreshPorts();
        end

        function refreshPorts(app)
            ports = serialportlist("available");
            if isempty(ports), ports = "(none)"; end
            app.PortDrop.Items = ports;
            app.PortDrop.Value = ports(end);
        end

        % =========================== actions ===========================
        function onConnect(app)
            if ~isempty(app.Link) && app.Link.isConnected()
                app.Link.disconnect();
                app.Link = [];
                app.ConnectBtn.Text = 'Connect';
                app.StatusLamp.Color = [0.6 0.6 0.6];
                app.StatusLbl.Text = 'disconnected';
                return;
            end
            app.stopReplay();
            try
                app.Link = TurretLink(app.PortDrop.Value);
                app.Link.connect();
                app.State = TurretState(struct());
                app.ConnectBtn.Text = 'Disconnect';
                app.StatusLamp.Color = [0.95 0.75 0.1];
                app.StatusLbl.Text = "connected " + app.PortDrop.Value;
            catch err
                uialert(app.Fig, err.message, 'Could not connect');
                app.Link = [];
            end
        end

        function cmd(app, c)
            if ~isempty(app.Link) && app.Link.isConnected()
                app.Link.send(c);
            end
        end

        function onLead(app, v)
            v = round(v / 5) * 5;
            app.LeadSlider.Value = v;
            app.LeadLbl.Text = sprintf('Lead: %d ms', v);
            app.cmd(sprintf("lead %d", v));
        end

        function onRecord(app, on)
            if isempty(app.Link) || ~app.Link.isConnected()
                app.RecordBtn.Value = false;
                return;
            end
            if on
                path = app.Link.startRecording(matlab.lang.makeValidName(app.TagField.Value));
                [~, f, e] = fileparts(path);
                app.RecordLbl.Text = "recording → logs/" + f + e;
                app.RecordBtn.BackgroundColor = [0.95 0.45 0.4];
            else
                app.Link.stopRecording();
                app.RecordLbl.Text = "saved " + app.Link.RecordPath;
                app.RecordBtn.BackgroundColor = [0.96 0.96 0.96];
            end
        end

        function onReplay(app)
            logDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'logs');
            [f, d] = uigetfile(fullfile(logDir, '*.csv'), 'Choose a turret_stab log');
            figure(app.Fig);
            if isequal(f, 0), return; end
            app.loadReplay(fullfile(d, f));
        end

        function stopReplay(app)
            app.Replay = struct('active', false);
        end

        % =========================== main loop ===========================
        function tick(app)
            if ~isvalid(app.Fig), return; end
            try
                if app.Replay.active
                    app.tickReplay();
                elseif ~isempty(app.Link) && app.Link.isConnected()
                    [D, ~, ~, ~] = app.Link.poll();
                    app.State.setCols(app.Link.Cols);
                    app.State.push(D);
                    app.updateHealth(app.Link.LastStatus);
                end
                app.redraw();
            catch err
                fprintf(2, 'dashboard tick: %s\n', err.message);
            end
        end

        function tickReplay(app)
            r = app.Replay;
            sp = str2double(extractBefore(string(app.ReplaySpeed.Value), "x"));
            tNow = r.tlog0 + toc(r.t0) * sp;
            tl = r.M(:, r.cols.t_ms) / 1000;
            j = find(tl <= tNow, 1, 'last');
            if isempty(j) || j < r.i, return; end
            app.State.push(r.M(r.i:j, :));
            app.Replay.i = j + 1;
            if j >= size(r.M, 1)
                app.StatusLbl.Text = "replay finished: " + r.name;
                app.stopReplay();
            end
        end

        function redraw(app)
            st = app.State;
            if isempty(st.t), return; end
            [Rd, az, el, tgt, brl] = st.displayPose();
            app.Model.update(Rd, az, el, tgt, brl);
            x = st.t - st.t(end);
            set(app.hBase, 'XData', x, 'YData', st.baseRot);
            set(app.hErr,  'XData', x, 'YData', st.errTot);
            ymax = max([10; st.baseRot(:); st.errTot(:)], [], 'omitnan') * 1.1;
            set(app.hLockShade, 'XData', x, 'YData', ymax * (st.lockedBuf == 1));
            set(app.hErrAz, 'XData', x, 'YData', st.errAz);
            set(app.hErrEl, 'XData', x, 'YData', st.errEl);
            set(app.hCmdAz, 'XData', x, 'YData', st.cmdAz);
            set(app.hMeasAz, 'XData', x, 'YData', st.measAz);
            set(app.hCmdEl, 'XData', x, 'YData', st.cmdEl);
            set(app.hMeasEl, 'XData', x, 'YData', st.measEl);
            for a = [app.AxTop app.AxMid app.AxBot], xlim(a, [-st.WindowS 0]); end
            ylim(app.AxTop, [0 ymax]);

            if toc(app.UiTic) > 0.2                    % text widgets: 5 Hz is plenty
                app.UiTic = tic;
                names = {'HOLD', 'FF', 'FF+LEAD', 'FULL', 'FB only'};
                mname = '?';
                if ~isnan(st.Mode) && st.Mode >= 0 && st.Mode <= 4, mname = names{st.Mode + 1}; end
                if st.Locked
                    app.LockBtn.Text = 'LOCKED';
                    app.LockBtn.BackgroundColor = [0.35 0.78 0.45];
                    if ~app.Replay.active, app.StatusLamp.Color = [0.2 0.8 0.3]; end
                else
                    app.LockBtn.Text = 'LOCK';
                    app.LockBtn.BackgroundColor = [0.85 0.87 0.9];
                    if ~app.Replay.active && ~isempty(app.Link), app.StatusLamp.Color = [0.95 0.75 0.1]; end
                end
                if ~isnan(st.Mode) && st.Mode >= 0 && st.Mode <= 4
                    items = app.ModeDrop.Items;
                    if ~strcmp(app.ModeDrop.Value, items{st.Mode + 1}), app.ModeDrop.Value = items{st.Mode + 1}; end
                end
                if ~isnan(st.Lead) && abs(app.LeadSlider.Value - st.Lead) > 1
                    app.LeadSlider.Value = min(200, st.Lead);
                    app.LeadLbl.Text = sprintf('Lead: %g ms', st.Lead);
                end
                app.ModeLbl.Text = sprintf('state: %s | mode %s | lead %g ms', ...
                    string(TurretDashboard.tern(st.Locked, "LOCKED", "free")), mname, st.Lead);
                m = st.metrics(10);
                if isnan(m.errRms)
                    app.ScoreLbl.Text = 'lock the turret and move the base';
                else
                    app.ScoreLbl.Text = sprintf(['pointing error  %5.2f° RMS\n' ...
                                                 'max error       %5.2f°\n' ...
                                                 'base motion     %5.1f° RMS\n' ...
                                                 'base rate       %5.0f°/s RMS\n' ...
                                                 'reduction       %5.0f %%'], ...
                                                m.errRms, m.errMax, m.baseRms, m.rateRms, m.reduction);
                end
            end
            drawnow limitrate;
        end

        function updateHealth(app, s)
            if isempty(fieldnames(s)), return; end
            g = @(f) TurretDashboard.field(s, f);
            app.HealthLbl.Text = sprintf(['IMU base/barrel  %s / %s Hz\n' ...
                                          'control         %s Hz\n' ...
                                          'servo reads     %s\n' ...
                                          'servo volts     %s\n' ...
                                          'loop max        %s us'], ...
                g('base_hz'), g('brl_hz'), g('ctrl_hz'), g('servo_ok'), g('volt_min'), g('loop_max_us'));
        end

        function onClose(app)
            try
                if ~isempty(app.Timer) && isvalid(app.Timer), stop(app.Timer); delete(app.Timer); end
            catch
            end
            try
                if ~isempty(app.Link), app.Link.disconnect(); end
            catch
            end
            if ~isempty(app.Fig) && isvalid(app.Fig), delete(app.Fig); end
        end
    end

    methods (Static, Access = private)
        function ax = makeAxes(parent, row, ttl, ylab)
            ax = uiaxes(parent);
            ax.Layout.Row = row; ax.Layout.Column = 3;
            hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
            title(ax, ttl); ylabel(ax, ylab);
        end

        function v = field(s, f)
            if isfield(s, f), v = char(s.(f)); else, v = '—'; end
        end

        function out = tern(c, a, b)
            if c, out = a; else, out = b; end
        end
    end
end
