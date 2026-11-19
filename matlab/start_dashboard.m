%START_DASHBOARD  Open the ENR 312 turret dashboard (run from anywhere).
%   start_dashboard          % pick the port in the app
%   start_dashboard("COM7")  % connect straight away
function app = start_dashboard(port)
    addpath(fileparts(mfilename('fullpath')));
    if nargin < 1, port = ""; end
    app = TurretDashboard(port);
end
