% Live serial check against the real turret (no UI). Turret must be on COM7 (or edit).
addpath(fileparts(fileparts(mfilename('fullpath'))));
link = TurretLink("COM7");
link.connect();
st = TurretState(struct());
t0 = tic; nD = 0; nS = 0; msgs = string.empty; sent = false;
while toc(t0) < 4
    [D, S, ~, m] = link.poll();
    st.setCols(link.Cols);
    st.push(D);
    nD = nD + size(D, 1); nS = nS + numel(S); msgs = [msgs; m]; %#ok<AGROW>
    if ~sent && toc(t0) > 1.5, link.send("info"); sent = true; end
    pause(0.05);
end
link.disconnect();
fprintf('D rows: %d in 4 s (%.0f Hz), S lines: %d, header cols: %d\n', nD, nD / 4, nS, numel(link.Header));
fprintf('status: base %s Hz, barrel %s Hz, servo %s, volts %s\n', link.LastStatus.base_hz, ...
        link.LastStatus.brl_hz, link.LastStatus.servo_ok, link.LastStatus.volt_min);
disp(msgs(contains(msgs, "# mode")));
[~, az, el] = st.displayPose();
fprintf('pose: az %.1f el %.1f\n', az, el);
assert(nD > 300, 'expected ~100 Hz of data');
fprintf('LIVE LINK TEST PASSED\n');
