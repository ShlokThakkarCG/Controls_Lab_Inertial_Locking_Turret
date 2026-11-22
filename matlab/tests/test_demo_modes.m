% Demo-mode switch check without hardware: replay the marker point-lock log in
% ROTATION LOCK mode and in TRANSLATION TEST mode and screenshot both.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'matlab'));
d = dir(fullfile(root, 'logs', '*_pointlock_marker.csv'));
assert(~isempty(d), 'needs a pointlock_marker log');
src = fullfile(d(end).folder, d(end).name);
M = readmatrix(src, 'NumHeaderLines', 1);
fid = fopen(src); hdr = fgetl(fid); fclose(fid);
link = TurretLink("none"); link.setHeader(hdr); c = link.Cols;
t = M(:, c.t_ms) / 1000;
lk = M(:, c.locked) == 1;
tLock = t(find(lk, 1)) - t(1);

app = TurretDashboard();
modes = ["rotation", "translation"];
for k = 1:2
    app.selectMode(modes(k));
    app.loadReplay(src);
    pause(tLock + 20);                                  % 20 s into the lock (slides happen 10-20 s)
    app.snapshot(fullfile(root, 'analysis', "matlab_dashboard_mode_" + modes(k) + ".png"));
end
delete(app);
fprintf('DEMO MODES TEST DONE\n');
