% Build the real dashboard, replay a log at 2x for a few seconds, screenshot it.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'matlab'));
app = TurretDashboard();
app.loadReplay(fullfile(root, 'logs', '20261120_024928_lead_test_v3.csv'));
pause(22);                                   % lock happens ~10 s in; let it run into the locked part
app.snapshot(fullfile(root, 'analysis', 'matlab_dashboard_replay.png'));
delete(app);
fprintf('DASHBOARD REPLAY TEST DONE\n');
