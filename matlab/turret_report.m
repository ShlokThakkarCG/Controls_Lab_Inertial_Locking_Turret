function S = turret_report(csvPath, varargin)
%TURRET_REPORT  Report-quality figures and statistics from one turret_stab log.
%   S = turret_report("logs/20261120_022805_hand_test_v1b.csv")
%   S = turret_report(path, 'Save', true, 'Visible', 'off')
%
%   The run is split into segments of constant (locked, mode, lead). For each
%   locked segment it reports base motion, pointing error and the reduction.
%   Samples where a joint sat on its limit are excluded from the error stats
%   (the turret physically could not point there) and counted separately.
%
%   Figures (saved to analysis/matlab_<log>_*.png when 'Save' is true):
%     1 time history with mode shading     2 RMS error per segment (bar)
%     3 error distribution (CDF)           4 error vs base angular rate
%   S is a table with one row per locked segment.

    opt = inputParser;
    opt.addParameter('Save', true);
    opt.addParameter('Visible', 'on');
    opt.parse(varargin{:});
    o = opt.Results;

    L = turret_load_log(csvPath);
    modeNames = ["HOLD", "FF", "FF+LEAD", "FULL", "FB"];
    % Joint-limit samples: the firmware limits changed between runs, so detect
    % them from the log itself - a command parked at its extreme value.
    sat = atLimit(L.cmd_az, -70, 100) | atLimit(L.cmd_el, -15, 50);
    baseRot = baseRotation(L);
    rate = sqrt(L.bw_x.^2 + L.bw_y.^2 + L.bw_z.^2);

    key = [L.locked L.mode L.lead_ms];
    edges = [1; find(any(diff(key) ~= 0, 2)) + 1; numel(L.t) + 1];
    rows = {};
    seg = struct('t0', {}, 't1', {}, 'mode', {}, 'lead', {}, 'label', {}, 'idx', {});
    for k = 1:numel(edges) - 1
        a = edges(k); b = edges(k + 1) - 1;
        if L.locked(a) ~= 1 || b - a < 100, continue; end
        idx = (a + 20:b)';                      % skip 0.2 s after each switch
        ok = idx(~sat(idx));
        e = L.err_tot(ok);
        m = L.mode(a); ld = L.lead_ms(a);
        lbl = modeNames(m + 1);
        if any(m == [2 3]), lbl = lbl + sprintf(" %g ms", ld); end
        rows(end + 1, :) = {L.t(a), L.t(b) - L.t(a), lbl, m, ld, max(baseRot(idx)), ...
            rmsv(baseRot(idx)), rmsv(rate(idx)), rmsv(e), pct(e, 95), max(e), ...
            100 * (1 - rmsv(e) / max(rmsv(baseRot(ok)), 1e-6)), 100 * mean(sat(idx))}; %#ok<AGROW>
        seg(end + 1) = struct('t0', L.t(a), 't1', L.t(b), 'mode', m, 'lead', ld, 'label', lbl, 'idx', ok); %#ok<AGROW>
    end
    S = cell2table(rows, 'VariableNames', {'t0', 'dur', 'segment', 'mode', 'lead_ms', 'base_max', ...
        'base_rms', 'rate_rms', 'err_rms', 'err_p95', 'err_max', 'reduction_pct', 'limit_pct'});
    fprintf('\n%s\n', L.name);
    disp(S(:, {'segment', 'dur', 'base_max', 'rate_rms', 'err_rms', 'err_p95', 'err_max', 'reduction_pct', 'limit_pct'}));

    outDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'analysis');
    if o.Save && ~isfolder(outDir), mkdir(outDir); end
    pal = lines(7);
    segCol = @(m) pal(mod(m, 7) + 1, :);

    % ---------- 1: time history ----------
    f1 = figure('Visible', o.Visible, 'Position', [50 50 1200 820], 'Color', 'w');
    lightTheme(f1);
    tl = tiledlayout(f1, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('Stabilizer run %s', strrep(L.name, '_', '\_')), 'FontWeight', 'bold');
    ax1 = nexttile(tl); hold(ax1, 'on');
    plot(ax1, L.t, baseRot, 'Color', [0.55 0.55 0.55], 'LineWidth', 1.2);
    plot(ax1, L.t, L.err_tot, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.5);
    ylabel(ax1, 'angle [deg]'); legend(ax1, {'base rotation since lock', 'barrel pointing error'}, 'Location', 'northeast');
    ax2 = nexttile(tl); hold(ax2, 'on');
    plot(ax2, L.t, L.err_az, 'LineWidth', 1.1); plot(ax2, L.t, L.err_el, 'LineWidth', 1.1);
    ylabel(ax2, 'error [deg]'); legend(ax2, {'azimuth (barrel frame)', 'elevation'}, 'Location', 'northeast');
    ax3 = nexttile(tl); hold(ax3, 'on');
    plot(ax3, L.t, L.cmd_az, 'Color', pal(1, :), 'LineWidth', 1.3); plot(ax3, L.t, L.meas_az, '--', 'Color', pal(1, :));
    plot(ax3, L.t, L.cmd_el, 'Color', pal(2, :), 'LineWidth', 1.3); plot(ax3, L.t, L.meas_el, '--', 'Color', pal(2, :));
    ylabel(ax3, 'joint [deg]'); xlabel(ax3, 'time [s]');
    legend(ax3, {'az command', 'az measured', 'el command', 'el measured'}, 'Location', 'northeast', 'NumColumns', 2);
    for ax = [ax1 ax2 ax3]
        grid(ax, 'on'); box(ax, 'on');
        yl = ylim(ax);
        for s = seg
            patch(ax, [s.t0 s.t1 s.t1 s.t0], [yl(1) yl(1) yl(2) yl(2)], segCol(s.mode), ...
                  'FaceAlpha', 0.07, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
        ylim(ax, yl);
    end
    for s = seg
        text(ax1, (s.t0 + s.t1) / 2, max(ylim(ax1)) * 0.93, s.label, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    linkaxes([ax1 ax2 ax3], 'x');

    % ---------- 2: RMS per segment ----------
    f2 = figure('Visible', o.Visible, 'Position', [80 80 900 480], 'Color', 'w');
    lightTheme(f2);
    ax = axes(f2); hold(ax, 'on');
    if ~isempty(S)
        Y = [S.base_rms S.err_rms];
        b = bar(ax, categorical(S.segment, S.segment), Y, 'grouped');
        b(1).FaceColor = [0.65 0.65 0.65]; b(2).FaceColor = [0.85 0.2 0.2];
        for k = 1:height(S)
            text(ax, k + 0.14, S.err_rms(k), sprintf('%.2f°\n(-%.0f%%)', S.err_rms(k), S.reduction_pct(k)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
        end
        legend(ax, {'base motion (RMS)', 'barrel pointing error (RMS)'}, 'Location', 'northeast');
    end
    ylabel(ax, 'angle [deg]'); grid(ax, 'on'); box(ax, 'on');
    title(ax, 'How much of the base motion reaches the barrel, per control mode');

    % ---------- 3: error distribution ----------
    f3 = figure('Visible', o.Visible, 'Position', [110 110 760 480], 'Color', 'w');
    lightTheme(f3);
    ax = axes(f3); hold(ax, 'on');
    for s = seg
        e = sort(L.err_tot(s.idx));
        plot(ax, e, (1:numel(e)) / numel(e) * 100, 'LineWidth', 1.8, 'DisplayName', s.label);
    end
    xlabel(ax, 'pointing error [deg]'); ylabel(ax, '% of time below'); grid(ax, 'on'); box(ax, 'on');
    legend(ax, 'Location', 'southeast'); title(ax, 'Pointing error distribution'); xlim(ax, [0 max(10, xlim(ax) * [0; 1])]);

    % ---------- 4: error vs base rate ----------
    f4 = figure('Visible', o.Visible, 'Position', [140 140 760 480], 'Color', 'w');
    lightTheme(f4);
    ax = axes(f4); hold(ax, 'on');
    binEdges = 0:20:300;
    for s = seg
        r = rate(s.idx); e = L.err_tot(s.idx);
        [~, ~, bi] = histcounts(r, binEdges);
        mu = accumarray(bi(bi > 0), e(bi > 0), [numel(binEdges) - 1 1], @mean, NaN);
        cnt = accumarray(bi(bi > 0), 1, [numel(binEdges) - 1 1]);
        mu(cnt < 10) = NaN;
        plot(ax, binEdges(1:end - 1) + 10, mu, '-o', 'LineWidth', 1.6, 'DisplayName', s.label);
    end
    xlabel(ax, 'base angular rate [deg/s]'); ylabel(ax, 'mean pointing error [deg]');
    grid(ax, 'on'); box(ax, 'on'); legend(ax, 'Location', 'northwest');
    title(ax, 'Faster base motion → larger error (lag); flatter = better');

    if o.Save
        names = {'timeline', 'rms_per_mode', 'error_cdf', 'error_vs_rate'};
        figs = [f1 f2 f3 f4];
        for k = 1:4
            exportgraphics(figs(k), fullfile(outDir, sprintf('matlab_%s_%s.png', L.name, names{k})), 'Resolution', 130);
        end
        fprintf('figures saved to %s\n', outDir);
    end
end

function br = baseRotation(L)
    % Base rotation angle since lock, from the logged yaw/pitch/roll (Z-X-Y order).
    y = L.base_yaw; p = L.base_pitch; r = L.base_roll;
    tr = (cosd(y) .* cosd(r) - sind(y) .* sind(p) .* sind(r)) + cosd(y) .* cosd(p) + cosd(p) .* cosd(r);
    br = acosd(max(-1, min(1, (tr - 1) / 2)));
end

function v = rmsv(x)
    x = x(~isnan(x));
    if isempty(x), v = NaN; else, v = sqrt(mean(x.^2)); end
end

function v = pct(x, p)
    x = sort(x(~isnan(x)));
    if isempty(x), v = NaN; return; end
    v = x(max(1, min(numel(x), round(p / 100 * numel(x)))));
end

function sat = atLimit(cmd, loThresh, hiThresh)
    % true where the command sits at the run's extreme value, if that extreme is near a joint limit
    sat = false(size(cmd));
    lo = min(cmd); hi = max(cmd);
    if lo <= loThresh, sat = sat | cmd <= lo + 0.05; end
    if hi >= hiThresh, sat = sat | cmd >= hi - 0.05; end
end

function lightTheme(f)
    try, theme(f, 'light'); catch, end      % report figures stay white even if MATLAB uses a dark theme
end
