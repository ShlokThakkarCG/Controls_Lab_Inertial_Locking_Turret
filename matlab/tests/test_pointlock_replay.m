% Point-lock dashboard check without moving the hardware:
% take a real turret_stab v2 log (point lock, base still), add a synthetic
% side-to-side slide of the pivot to its translation columns, replay it in the
% dashboard and screenshot the 3D view (target star, pivot path, shifted plate).
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'matlab'));
logs = fullfile(root, 'logs');
d = dir(fullfile(logs, '*_pointlock_still.csv'));
assert(~isempty(d), 'record a pointlock_still log first');
src = fullfile(logs, d(end).name);
fid = fopen(src); hdr = fgetl(fid); fclose(fid);
M = readmatrix(src, 'NumHeaderLines', 1);
link = TurretLink("none"); link.setHeader(hdr); c = link.Cols;
assert(isfield(c, 'piv_x') && isfield(c, 'tp_y'), 'log lacks point-lock columns');

t = (M(:, c.t_ms) - M(1, c.t_ms)) / 1000;
lk = M(:, c.locked) == 1;
tl = t - t(find(lk, 1));
M(lk, c.piv_x) = 8 * sin(2 * pi * 0.25 * tl(lk));          % +-8 cm slide to the right/left [cm]
M(lk, c.piv_y) = 3 * (1 - cos(2 * pi * 0.25 * tl(lk)));    % and a little forward
M(:, c.zupt) = 0;
syn = fullfile(tempdir, 'pointlock_synthetic.csv');
fid = fopen(syn, 'w'); fprintf(fid, '%s\n', hdr); fclose(fid);
writematrix(M, syn, 'WriteMode', 'append');

% state engine checks
st = TurretState(c);
st.push(M(1:find(lk, 1) + 300, :));
[piv, tp] = st.pointPose();
fprintf('range %.2f m, target point %s m, pivot shift %s cm\n', st.RangeM, mat2str(round(tp', 3)), mat2str(round(100 * piv', 1)));
assert(st.Locked && st.HasPoint && abs(norm(tp) - 0.5) < 0.02, 'point lock state');

app = TurretDashboard();
app.loadReplay(syn);
t_lock = t(find(lk, 1));
pause(t_lock + 5);                               % log starts at boot; run 5 s into the lock
app.snapshot(fullfile(root, 'analysis', 'matlab_dashboard_pointlock.png'));
delete(app);
fprintf('POINT LOCK REPLAY TEST DONE\n');
