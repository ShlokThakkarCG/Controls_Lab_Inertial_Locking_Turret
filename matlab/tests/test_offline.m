% Headless checks for the MATLAB tools (run: matlab -batch "run('matlab/tests/test_offline.m')").
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fileparts(here));
logs = fullfile(root, 'logs');
out = fullfile(root, 'analysis');

% ---- 1. kinematics agree with the firmware conventions ----
assert(norm(TurretMath.turretRot(90, 0) * [0;1;0] - [-1;0;0]) < 1e-12, '+az must turn the barrel left');
assert(norm(TurretMath.turretRot(0, 90) * [0;1;0] - [0;0;1]) < 1e-12, '+el must tilt the barrel up');
q = [cosd(15) 0 0 sind(15)];                       % 30 deg about +Z
assert(abs(TurretMath.heading(TurretMath.quatToRot(q)) - 30) < 1e-9, 'heading');
fprintf('kinematics OK\n');

% ---- 2. parse raw telemetry lines exactly like the live link does ----
raw = readlines(fullfile(logs, '20261120_024928_lead_test_v3_raw.txt'));
lines = strtrim(extractAfter(raw(contains(raw, '<< ')), '<< '));
link = TurretLink("none");
[D, S, E] = link.parseLines(lines);
fprintf('parsed %d D rows x %d cols, %d S, %d E lines\n', size(D, 1), size(D, 2), numel(S), numel(E));
assert(size(D, 2) == numel(link.Header) && size(D, 1) > 4000, 'D parse');
assert(isfield(link.LastStatus, 'base_hz'), 'S parse');

% ---- 3. state engine: lock detection + display pose + metrics ----
st = TurretState(link.Cols);
for k = 1:200:size(D, 1)                              % feed in chunks like the timer would
    st.push(D(k:min(end, k + 199), :));
end
fprintf('locks seen: %d, buffer %.1f s\n', st.LockCount, st.t(end) - st.t(1));
assert(st.LockCount >= 1, 'lock detection');

% replay into a figure and render frames of the 3D model at a locked moment
c = link.Cols;
iLock = find(D(:, c.locked) == 1, 1) + 1500;           % 15 s after lock
st2 = TurretState(c);
st2.push(D(1:iLock, :));
[Rd, az, el, tgt, brl] = st2.displayPose();
fprintf('pose at t=%.1fs: az %.1f el %.1f, target %s, angle(barrel IMU, target) = %.2f deg\n', ...
    st2.t(end) - st2.t(1), az, el, mat2str(round(tgt', 3)), acosd(min(1, dot(tgt, brl))));
m = st2.metrics(10);
fprintf('metrics: err %.2f RMS, base %.1f RMS, reduction %.0f%%\n', m.errRms, m.baseRms, m.reduction);
assert(~isempty(tgt) && acosd(min(1, dot(tgt, brl))) < 10, 'barrel IMU ray should be near the target ray');

f = figure('Visible', 'off', 'Position', [0 0 900 800], 'Color', 'w');
try, theme(f, 'light'); catch, end
ax = axes(f);
mdl = TurretModel3D(ax);
mdl.update(Rd, az, el, tgt, brl);
title(ax, sprintf('3D model test frame (az %.1f°, el %.1f°)', az, el));
exportgraphics(f, fullfile(out, 'matlab_test_model3d.png'), 'Resolution', 110);
fprintf('3D frame saved\n');

% ---- 4. report figures for two key runs ----
S1 = turret_report(fullfile(logs, '20261120_022805_hand_test_v1b.csv'), 'Visible', 'off');
S2 = turret_report(fullfile(logs, '20261120_024928_lead_test_v3.csv'), 'Visible', 'off');
assert(height(S1) == 3 && height(S2) == 4, 'segments');
fprintf('ALL OFFLINE TESTS PASSED\n');
