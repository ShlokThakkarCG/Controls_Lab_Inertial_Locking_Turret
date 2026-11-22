classdef TurretDashboard < handle
    % TurretDashboard  Live control & visualisation of the ENR 312 stabilized turret.
    %
    %   TurretDashboard            % open the app (pick the port inside)
    %   TurretDashboard("COM7")    % open and connect straight away
    %
    %   * Aim: jog the turret with the arrow pad or the keyboard arrow keys
    %     (free: moves the joints; locked: nudges the locked target)
    %   * Point lock: type the target distance, LOCK -> the turret holds that POINT in
    %     space while the base is rotated AND moved (firmware turret_stab v2).
    %     Distance 0 = classic direction lock (target infinitely far away).
    %   * 3D mock-up moving with the hardware: green = line to the locked point / direction,
    %     red dashed = barrel direction from its IMU, pink star = locked point,
    %     purple = path of the turret pivot (estimated from the base IMU accelerometer)
    %   * live plots: base disturbance vs pointing error, error components, joints, translation
    %   * Record: writes logs/<stamp>_<tag>.csv + _raw.txt (same format as the Python tools)
    %   * Replay: play any saved log through the 3D model and plots (no hardware needed)

    properties (Constant, Access = private)
        AzLim = [-75 110]
        ElLim = [-18 55]
    end

    properties (Access = private)
        Fig; Grid
        % controls
        PortDrop; ConnectBtn; StatusLamp; StatusLbl
        LockBtn; ModeDrop; LeadSlider; LeadLbl; KpField; KiField
        StepDrop; AimLbl; RangeField; PivX; PivY; PivZ; TransBox
        RotBtn; TransBtn; ModeHint; TransWidgets; VerdictLbl; SecLockLbl
        RecordBtn; TagField; RecordLbl; ReplayBtn; ReplaySpeed
        HealthLbl; ScoreLbl; ModeLbl
        % axes
        Ax3D; AxTop; AxMid; AxBot; AxTr
        hBase; hErr; hErrAz; hErrEl; hCmdAz; hMeasAz; hCmdEl; hMeasEl; hLockShade
        hTrX; hTrY; hTrZ; hZupt; hEnvP; hEnvN; hRate
        Model
        % demo mode: "rotation" = direction lock only, "translation" = point-lock experiment
        DemoMode string = "rotation"
        % data
        Link; State; Timer
        Replay struct = struct('active', false)
        UiTic uint64 = tic
        % aim bookkeeping
        JogAz double = 0; JogEl double = 0       % joint targets while free
        AimAz double = 0; AimEl double = 0       % target offsets while locked
        LastJog uint64 = tic
        WasLocked logical = false
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

        function selectMode(app, mode)
            % "rotation" (direction lock, no translation) or "translation" (point-lock experiment)
            app.setDemoMode(mode);
        end

        function snapshot(app, pngPath)
            exportapp(app.Fig, pngPath);
        end

        function jog(app, dAz, dEl)
            % Arrow pad / keys. Free: move the joints. Locked: nudge the locked target.
            % dAz > 0 = left, dEl > 0 = up (turret conventions).
            if app.State.Locked
                app.AimAz = app.AimAz + dAz;
                app.AimEl = app.AimEl + dEl;
                app.cmd(sprintf("aim %.2f %.2f", app.AimAz, app.AimEl));
            else
                if toc(app.LastJog) > 0.6 && ~isempty(app.State.cmdAz)   % re-seed from where the turret is
                    app.JogAz = app.State.cmdAz(end);
                    app.JogEl = app.State.cmdEl(end);
                end
                app.JogAz = min(max(app.JogAz + dAz, app.AzLim(1)), app.AzLim(2));
                app.JogEl = min(max(app.JogEl + dEl, app.ElLim(1)), app.ElLim(2));
                app.cmd(sprintf("go %.2f %.2f", app.JogAz, app.JogEl));
            end
            app.LastJog = tic;
            app.updateAimLabel();
        end
    end

    % =========================== UI ===========================
    methods (Access = private)
        function buildUi(app)
            app.Fig = uifigure('Name', 'ENR 312 — Inertially Stabilized Turret (point lock)', ...
                               'Position', [40 40 1560 920], 'Color', [0.97 0.97 0.98], ...
                               'CloseRequestFcn', @(~, ~) app.onClose(), ...
                               'WindowKeyPressFcn', @(~, e) app.onKey(e));
            try, app.Fig.Theme = 'light'; catch, end
            app.Grid = uigridlayout(app.Fig, [4 3], 'ColumnWidth', {290, '1.15x', '1x'}, ...
                                    'RowHeight', {'1x', '1x', '1x', '1x'}, 'Padding', [10 10 10 10], ...
                                    'ColumnSpacing', 10, 'RowSpacing', 8);

            % ---------- left control panel (scrolls if the window is short) ----------
            p = uipanel(app.Grid, 'Title', 'Turret control', 'FontWeight', 'bold');
            p.Layout.Row = [1 4]; p.Layout.Column = 1;
            nRows = 40;
            g = uigridlayout(p, [nRows 2], 'RowHeight', repmat({'fit'}, 1, nRows), ...
                             'ColumnWidth', {'1x', '1x'}, 'RowSpacing', 5, 'Scrollable', 'on');

            uilabel(g, 'Text', 'Serial port', 'FontWeight', 'bold');
            uibutton(g, 'Text', 'Refresh', 'ButtonPushedFcn', @(~, ~) app.refreshPorts());
            app.PortDrop = uidropdown(g, 'Items', {'(none)'});
            app.PortDrop.Layout.Column = [1 2];
            app.ConnectBtn = uibutton(g, 'Text', 'Connect', 'ButtonPushedFcn', @(~, ~) app.onConnect());
            app.ConnectBtn.Layout.Column = [1 2];
            app.StatusLamp = uilamp(g, 'Color', [0.6 0.6 0.6]);
            app.StatusLbl = uilabel(g, 'Text', 'disconnected');

            % --- demo mode switch ---
            h = uilabel(g, 'Text', 'Demo mode', 'FontWeight', 'bold', 'FontColor', [0.1 0.35 0.6]);
            h.Layout.Column = [1 2];
            app.RotBtn = uibutton(g, 'Text', {'ROTATION', 'LOCK'}, 'FontWeight', 'bold', 'FontSize', 13, ...
                                  'Tooltip', 'hold a direction: base rotation is rejected, no translation', ...
                                  'ButtonPushedFcn', @(~, ~) app.setDemoMode("rotation"));
            app.TransBtn = uibutton(g, 'Text', {'TRANSLATION', 'TEST'}, 'FontWeight', 'bold', 'FontSize', 13, ...
                                    'Tooltip', 'hold a point at a distance using the IMU position estimate (experiment)', ...
                                    'ButtonPushedFcn', @(~, ~) app.setDemoMode("translation"));
            app.ModeHint = uilabel(g, 'Text', '', 'WordWrap', 'on', 'FontSize', 11);
            app.ModeHint.Layout.Column = [1 2];

            % --- aim ---
            h = uilabel(g, 'Text', '1. Aim  (arrow keys work too)', 'FontWeight', 'bold', 'FontColor', [0.1 0.35 0.6]);
            h.Layout.Column = [1 2];
            uilabel(g, 'Text', 'Step per press');
            app.StepDrop = uidropdown(g, 'Items', {'0.5°', '1°', '2°', '5°', '10°'}, 'Value', '2°');
            pad = uigridlayout(g, [3 3], 'RowHeight', {30, 30, 30}, 'ColumnWidth', {'1x', '1x', '1x'}, ...
                               'Padding', [30 0 30 0], 'RowSpacing', 3, 'ColumnSpacing', 3);
            pad.Layout.Column = [1 2];
            mk = @(txt, r, c, fn) TurretDashboard.padBtn(pad, txt, r, c, fn);
            mk('▲', 1, 2, @() app.jog(0, app.step()));
            mk('◄', 2, 1, @() app.jog(app.step(), 0));
            mk('●', 2, 2, @() app.onCenterAim());
            mk('►', 2, 3, @() app.jog(-app.step(), 0));
            mk('▼', 3, 2, @() app.jog(0, -app.step()));
            app.AimLbl = uilabel(g, 'Text', 'joints: az 0.0°  el 0.0°', 'FontName', 'Consolas');
            app.AimLbl.Layout.Column = [1 2];

            % --- lock (+ target distance etc. in translation mode, in one collapsible sub-grid) ---
            app.SecLockLbl = uilabel(g, 'Text', '2. Lock', 'FontWeight', 'bold', 'FontColor', [0.1 0.35 0.6]);
            app.SecLockLbl.Layout.Column = [1 2];
            tg = uigridlayout(g, [6 2], 'RowHeight', repmat({'fit'}, 1, 6), 'ColumnWidth', {'1x', '1x'}, ...
                              'Padding', [0 0 0 0], 'RowSpacing', 5);
            tg.Layout.Column = [1 2];
            uilabel(tg, 'Text', 'Distance from pivot [m]', 'WordWrap', 'on');
            app.RangeField = uieditfield(tg, 'numeric', 'Value', 0.5, 'Limits', [0 20], ...
                                         'ValueDisplayFormat', '%.2f', 'Tooltip', '0 = infinitely far (direction lock)', ...
                                         'ValueChangedFcn', @(s, ~) app.cmd(sprintf("range %.3f", s.Value)));
            h = uilabel(tg, 'Text', 'Pivot offset from base IMU [cm]  (x right, y fwd, z up)', 'WordWrap', 'on', 'FontSize', 11);
            h.Layout.Column = [1 2];
            pv = uigridlayout(tg, [1 3], 'RowHeight', {24}, 'ColumnWidth', {'1x', '1x', '1x'}, 'Padding', [0 0 0 0], 'ColumnSpacing', 4);
            pv.Layout.Column = [1 2];
            app.PivX = uieditfield(pv, 'numeric', 'Value', 0, 'ValueChangedFcn', @(~, ~) app.sendPivot());
            app.PivY = uieditfield(pv, 'numeric', 'Value', 21, 'ValueChangedFcn', @(~, ~) app.sendPivot());
            app.PivZ = uieditfield(pv, 'numeric', 'Value', 7, 'ValueChangedFcn', @(~, ~) app.sendPivot());
            app.TransBox = uicheckbox(tg, 'Text', 'Use translation', 'Value', true, ...
                                      'ValueChangedFcn', @(s, ~) app.cmd(TurretDashboard.tern(s.Value, "trans on", "trans off")));
            uibutton(tg, 'Text', 'Zero position', 'Tooltip', 'reset the translation estimate (keeps the lock)', ...
                     'ButtonPushedFcn', @(~, ~) app.cmd("treset"));
            app.VerdictLbl = uilabel(tg, 'WordWrap', 'on', 'FontSize', 11, 'FontColor', [0.6 0.15 0.1], 'Text', ...
                ['Why it fails: position = double integral of acceleration, so an accel error a_err ' ...
                 'grows as ½·a_err·t² (0.05 m/s² → 10 cm after 2 s). An accelerometer cannot tell ' ...
                 'constant velocity from rest: position is NOT observable from the IMU alone. ' ...
                 'Fix: an absolute position sensor (camera / ToF) fused with the IMU in a Kalman filter.']);
            app.VerdictLbl.Layout.Column = [1 2];
            app.TransWidgets = {g, tg};                % parent grid + the sub-grid to collapse
            app.LockBtn = uibutton(g, 'Text', 'LOCK', 'FontSize', 18, 'FontWeight', 'bold', ...
                                   'BackgroundColor', [0.85 0.87 0.9], 'ButtonPushedFcn', @(~, ~) app.onLock());
            app.LockBtn.Layout.Column = [1 2];
            app.ModeLbl = uilabel(g, 'Text', 'state: —', 'FontWeight', 'bold', 'WordWrap', 'on');
            app.ModeLbl.Layout.Column = [1 2];

            % --- controller ---
            h = uilabel(g, 'Text', 'Controller', 'FontWeight', 'bold', 'FontColor', [0.1 0.35 0.6]);
            h.Layout.Column = [1 2];
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
            uibutton(g, 'Text', 'Center (0,0)', 'ButtonPushedFcn', @(~, ~) app.onCenterAim());
            uibutton(g, 'Text', 'Torque OFF', 'ButtonPushedFcn', @(~, ~) app.cmd("off"));

            % --- logging ---
            uilabel(g, 'Text', 'Log tag');
            app.TagField = uieditfield(g, 'text', 'Value', 'pointlock');
            app.RecordBtn = uibutton(g, 'state', 'Text', 'Record', 'ValueChangedFcn', @(s, ~) app.onRecord(s.Value));
            app.RecordBtn.Layout.Column = [1 2];
            app.RecordLbl = uilabel(g, 'Text', 'not recording', 'FontSize', 10, 'WordWrap', 'on');
            app.RecordLbl.Layout.Column = [1 2];
            app.ReplayBtn = uibutton(g, 'Text', 'Replay a log…', 'ButtonPushedFcn', @(~, ~) app.onReplay());
            app.ReplaySpeed = uidropdown(g, 'Items', {'0.5x', '1x', '2x'}, 'Value', '1x');

            h = uilabel(g, 'Text', 'Live score (last 10 s locked)', 'FontWeight', 'bold'); h.Layout.Column = [1 2];
            app.ScoreLbl = uilabel(g, 'Text', '—', 'FontName', 'Consolas', 'FontSize', 12, 'WordWrap', 'on');
            app.ScoreLbl.Layout.Column = [1 2];
            h = uilabel(g, 'Text', 'Health', 'FontWeight', 'bold'); h.Layout.Column = [1 2];
            app.HealthLbl = uilabel(g, 'Text', '—', 'FontName', 'Consolas', 'FontSize', 11, 'WordWrap', 'on');
            app.HealthLbl.Layout.Column = [1 2];

            % ---------- 3D model ----------
            app.Ax3D = uiaxes(app.Grid);
            app.Ax3D.Layout.Row = [1 4]; app.Ax3D.Layout.Column = 2;
            title(app.Ax3D, {'Live turret', '\color[rgb]{0.1,0.6,0.3}green\color{black} = to locked point,  \color[rgb]{0.85,0.2,0.2}red\color{black} = barrel IMU,  \color[rgb]{0.55,0.25,0.8}purple\color{black} = pivot path'});
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
            % 4th plot: base rate (rotation mode) or translation estimate + drift envelope (translation mode)
            app.AxTr = TurretDashboard.makeAxes(app.Grid, 4, '', '');
            app.hZupt = area(app.AxTr, nan, nan, 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.10, ...
                             'EdgeColor', 'none', 'HandleVisibility', 'off', 'BaseValue', 0);
            app.hTrX = plot(app.AxTr, nan, nan, 'Color', [0.85 0.2 0.2], 'LineWidth', 1.4);
            app.hTrY = plot(app.AxTr, nan, nan, 'Color', [0.2 0.65 0.3], 'LineWidth', 1.4);
            app.hTrZ = plot(app.AxTr, nan, nan, 'Color', [0.2 0.4 0.85], 'LineWidth', 1.4);
            app.hEnvP = plot(app.AxTr, nan, nan, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.1);
            app.hEnvN = plot(app.AxTr, nan, nan, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.1, 'HandleVisibility', 'off');
            app.hRate = plot(app.AxTr, nan, nan, 'Color', [0.45 0.3 0.7], 'LineWidth', 1.4);
            xlabel(app.AxTr, 'time [s]');
            app.refreshPorts();
            app.setDemoMode("rotation");
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
                app.pushModeSettings();             % firmware follows the dashboard's demo mode
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

        function onLock(app)
            if ~app.State.Locked
                app.pushModeSettings();             % make sure the lock uses this mode's settings
            end
            app.cmd("l");
        end

        function setDemoMode(app, mode)
            % ROTATION LOCK: direction lock, no translation anywhere in the UI or firmware.
            % TRANSLATION TEST: point lock at a typed distance using the IMU position estimate.
            mode = string(mode);
            if mode ~= app.DemoMode && ~isempty(app.State) && app.State.Locked
                app.cmd("l");                       % unlock: a lock belongs to one mode
            end
            app.DemoMode = mode;
            isT = mode == "translation";
            on = [0.15 0.45 0.8]; off = [0.9 0.91 0.93];
            app.RotBtn.BackgroundColor = TurretDashboard.tern(isT, off, on);
            app.RotBtn.FontColor = TurretDashboard.tern(isT, [0.2 0.2 0.2], [1 1 1]);
            app.TransBtn.BackgroundColor = TurretDashboard.tern(isT, [0.8 0.4 0.1], off);
            app.TransBtn.FontColor = TurretDashboard.tern(isT, [1 1 1], [0.2 0.2 0.2]);
            g = app.TransWidgets{1}; tg = app.TransWidgets{2};
            tg.Visible = TurretDashboard.tern(isT, 'on', 'off');
            g.RowHeight{tg.Layout.Row(1)} = TurretDashboard.tern(isT, 'fit', 0);   % collapse: no gap in rotation mode
            if isT
                app.ModeHint.Text = ['Experiment: holds a POINT at the typed distance while the base is rotated ' ...
                                     'AND moved. Translation comes from double-integrating the base accelerometer.'];
                app.SecLockLbl.Text = '2. Target distance & lock';
                app.Fig.Name = 'ENR 312 — Stabilized Turret  |  TRANSLATION TEST (point lock, experiment)';
                title(app.AxTr, 'Pivot translation since lock (IMU estimate) vs drift envelope ½·a·t²');
                ylabel(app.AxTr, 'shift [cm]');
                legend(app.AxTr, [app.hTrX app.hTrY app.hTrZ app.hEnvP], ...
                       {'x right', 'y forward', 'z up', '±½·0.05·t²'}, 'Location', 'northwest', 'NumColumns', 4);
            else
                app.ModeHint.Text = ['Holds the barrel''s DIRECTION in space while the base is rotated by hand ' ...
                                     '(roll, pitch, yaw). Aim, LOCK, then move the base.'];
                app.SecLockLbl.Text = '2. Lock';
                app.Fig.Name = 'ENR 312 — Stabilized Turret  |  ROTATION LOCK';
                title(app.AxTr, 'Base angular rate (the disturbance the controller rejects)');
                ylabel(app.AxTr, 'rate [deg/s]');
                legend(app.AxTr, app.hRate, {'|base rate|'}, 'Location', 'northwest');
            end
            set([app.hTrX app.hTrY app.hTrZ app.hEnvP app.hEnvN app.hZupt], 'Visible', TurretDashboard.tern(isT, 'on', 'off'));
            app.hRate.Visible = TurretDashboard.tern(isT, 'off', 'on');
            app.pushModeSettings();
        end

        function pushModeSettings(app)
            if app.DemoMode == "translation"
                app.cmd(sprintf("range %.3f", app.RangeField.Value));
                app.sendPivot();
                app.cmd(TurretDashboard.tern(app.TransBox.Value, "trans on", "trans off"));
            else
                app.cmd("range 0");                 % 0 = direction lock (target infinitely far)
                app.cmd("trans off");
            end
        end

        function onCenterAim(app)
            if app.State.Locked
                app.AimAz = 0; app.AimEl = 0;
                app.cmd("aim 0 0");
            else
                app.JogAz = 0; app.JogEl = 0;
                app.cmd("c");
            end
            app.LastJog = tic;
            app.updateAimLabel();
        end

        function onKey(app, e)
            % Arrow keys aim (Shift = 5x step). Ignored while typing in a number box.
            if isa(app.Fig.CurrentObject, 'matlab.ui.control.NumericEditField') || ...
               isa(app.Fig.CurrentObject, 'matlab.ui.control.EditField'), return; end
            k = app.step();
            if any(strcmp(e.Modifier, 'shift')), k = 5 * k; end
            switch e.Key
                case 'leftarrow',  app.jog(k, 0);
                case 'rightarrow', app.jog(-k, 0);
                case 'uparrow',    app.jog(0, k);
                case 'downarrow',  app.jog(0, -k);
            end
        end

        function s = step(app)
            s = str2double(erase(string(app.StepDrop.Value), "°"));
        end

        function sendPivot(app)
            app.cmd(sprintf("pivot %.1f %.1f %.1f", app.PivX.Value, app.PivY.Value, app.PivZ.Value));
        end

        function updateAimLabel(app)
            if app.State.Locked
                app.AimLbl.Text = sprintf('target nudge: az %+.1f°  el %+.1f°', app.AimAz, app.AimEl);
            else
                app.AimLbl.Text = sprintf('joints: az %.1f°  el %.1f°', app.JogAz, app.JogEl);
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
            if st.Locked && ~app.WasLocked                 % new lock: nudges start from zero
                app.AimAz = 0; app.AimEl = 0;
                app.updateAimLabel();
            elseif ~st.Locked && app.WasLocked
                app.updateAimLabel();
            end
            app.WasLocked = st.Locked;
            [Rd, az, el, tgt, brl] = st.displayPose();
            isT = app.DemoMode == "translation";
            [piv, tp, RdLock] = st.pointPose();
            if ~isT, piv = [0; 0; 0]; tp = []; end          % rotation mode: no translation in the 3D view
            app.Model.update(Rd, az, el, tgt, brl, piv, tp, RdLock);
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
            if isT
                lk = st.lockedBuf == 1;                       % translation only means something while locked
                set(app.hTrX, 'XData', x, 'YData', TurretDashboard.maskNaN(st.pivX, lk));
                set(app.hTrY, 'XData', x, 'YData', TurretDashboard.maskNaN(st.pivY, lk));
                set(app.hTrZ, 'XData', x, 'YData', TurretDashboard.maskNaN(st.pivZ, lk));
                % drift envelope since the current lock: ½·a_err·t² with a_err = 0.05 m/s² -> 2.5·t² cm
                env = nan(size(x));
                if st.Locked && lk(end)
                    k0 = find(~lk, 1, 'last'); if isempty(k0), k0 = 0; end
                    tl = st.t(k0 + 1:end) - st.t(k0 + 1);
                    env(k0 + 1:end) = 2.5 * tl.^2;
                end
                tmax = max([2; abs([st.pivX(lk); st.pivY(lk); st.pivZ(lk)])], [], 'omitnan') * 1.15;
                env(env > tmax) = NaN;                        % the parabola leaves the plot: let it
                set(app.hEnvP, 'XData', x, 'YData', env);
                set(app.hEnvN, 'XData', x, 'YData', -env);
                set(app.hZupt, 'XData', x, 'YData', tmax * (st.zuptBuf == 1 & lk));
                ylim(app.AxTr, [-tmax tmax]);
            else
                set(app.hRate, 'XData', x, 'YData', st.rateBuf);
                ylim(app.AxTr, [0 max([20; st.rateBuf(:)], [], 'omitnan') * 1.1]);
            end
            for a = [app.AxTop app.AxMid app.AxBot app.AxTr], xlim(a, [-st.WindowS 0]); end
            ylim(app.AxTop, [0 ymax]);

            if toc(app.UiTic) > 0.2                    % text widgets: 5 Hz is plenty
                app.UiTic = tic;
                names = {'HOLD', 'FF', 'FF+LEAD', 'FULL', 'FB only'};
                mname = '?';
                if ~isnan(st.Mode) && st.Mode >= 0 && st.Mode <= 4, mname = names{st.Mode + 1}; end
                pointLock = isT && st.HasPoint && st.RangeM > 0;
                if st.Locked
                    if pointLock, app.LockBtn.Text = sprintf('LOCKED on point (%.2f m)', st.RangeM);
                    else, app.LockBtn.Text = 'LOCKED on direction'; end
                    app.LockBtn.BackgroundColor = [0.35 0.78 0.45];
                    if ~app.Replay.active, app.StatusLamp.Color = [0.2 0.8 0.3]; end
                else
                    if isT && app.RangeField.Value > 0, app.LockBtn.Text = sprintf('LOCK point @ %.2f m', app.RangeField.Value);
                    else, app.LockBtn.Text = 'LOCK direction'; end
                    app.LockBtn.BackgroundColor = [0.85 0.87 0.9];
                    if ~app.Replay.active && ~isempty(app.Link), app.StatusLamp.Color = [0.95 0.75 0.1]; end
                end
                if ~st.Locked && toc(app.LastJog) > 0.6 && ~isempty(st.cmdAz)
                    app.JogAz = st.cmdAz(end); app.JogEl = st.cmdEl(end);
                    app.updateAimLabel();
                end
                if ~isnan(st.Mode) && st.Mode >= 0 && st.Mode <= 4
                    items = app.ModeDrop.Items;
                    if ~strcmp(app.ModeDrop.Value, items{st.Mode + 1}), app.ModeDrop.Value = items{st.Mode + 1}; end
                end
                if ~isnan(st.Lead) && abs(app.LeadSlider.Value - st.Lead) > 1
                    app.LeadSlider.Value = min(200, st.Lead);
                    app.LeadLbl.Text = sprintf('Lead: %g ms', st.Lead);
                end
                kind = "free";
                if st.Locked, kind = TurretDashboard.tern(pointLock, "POINT lock", "DIRECTION lock"); end
                app.ModeLbl.Text = sprintf('state: %s | mode %s | lead %g ms', kind, mname, st.Lead);
                m = st.metrics(10);
                if isnan(m.errRms)
                    app.ScoreLbl.Text = 'aim, set the distance, LOCK, then move the base';
                else
                    txt = sprintf(['pointing error  %5.2f° RMS\n' ...
                                   'max error       %5.2f°\n' ...
                                   'base motion     %5.1f° RMS\n' ...
                                   'base rate       %5.0f°/s RMS\n' ...
                                   'reduction       %5.0f %%'], ...
                                  m.errRms, m.errMax, m.baseRms, m.rateRms, m.reduction);
                    if pointLock && st.Locked
                        txt = sprintf('%s\npivot moved     %5.1f cm', txt, 100 * norm(st.Piv));
                    end
                    app.ScoreLbl.Text = txt;
                end
            end
            drawnow limitrate;
        end

        function updateHealth(app, s)
            if isempty(fieldnames(s)), return; end
            g = @(f) TurretDashboard.field(s, f);
            app.HealthLbl.Text = sprintf(['IMU base/barrel  %s / %s Hz\n' ...
                                          'base accel      %s Hz\n' ...
                                          'control         %s Hz\n' ...
                                          'servo reads     %s\n' ...
                                          'servo volts     %s\n' ...
                                          'loop max        %s us'], ...
                g('base_hz'), g('brl_hz'), g('la_hz'), g('ctrl_hz'), g('servo_ok'), g('volt_min'), g('loop_max_us'));
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

        function b = padBtn(parent, txt, r, c, fn)
            b = uibutton(parent, 'Text', txt, 'FontSize', 14, 'ButtonPushedFcn', @(~, ~) fn());
            b.Layout.Row = r; b.Layout.Column = c;
        end

        function y = maskNaN(y, keep)
            y(~keep) = NaN;
        end

        function v = field(s, f)
            if isfield(s, f), v = char(s.(f)); else, v = '—'; end
        end

        function out = tern(c, a, b)
            if c, out = a; else, out = b; end
        end
    end
end
