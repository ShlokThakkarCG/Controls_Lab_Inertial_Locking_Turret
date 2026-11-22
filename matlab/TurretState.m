classdef TurretState < handle
    % TurretState  Turns turret_stab data rows into display quantities.
    %   Keeps rolling buffers for the live plots, the lock reference, and the
    %   pose for the 3D model. UI-free so it can be tested headless.
    %
    %   st = TurretState(cols);       % cols = TurretLink.Cols (column indices)
    %   st.push(D);                    % D = numeric rows from TurretLink.poll
    %   [Rd, az, el, tgt, brl] = st.displayPose();
    %   m = st.metrics(10);            % stats over the last 10 s of locked data
    %   [piv, tp, RdLock] = st.pointPose();   % point lock: pivot shift + target point [m], display frame
    %
    % Display frame = base IMU world frame turned about Z so that the base faced
    % +Y at the reference moment (the lock, or the first sample).

    properties
        WindowS double = 15            % length of the rolling plot buffers [s]
        % rolling buffers (column vectors)
        t = zeros(0, 1); baseRot = zeros(0, 1); errTot = zeros(0, 1)
        errAz = zeros(0, 1); errEl = zeros(0, 1)
        cmdAz = zeros(0, 1); measAz = zeros(0, 1); cmdEl = zeros(0, 1); measEl = zeros(0, 1)
        lockedBuf = zeros(0, 1); rateBuf = zeros(0, 1)
        pivX = zeros(0, 1); pivY = zeros(0, 1); pivZ = zeros(0, 1); zuptBuf = zeros(0, 1)   % [cm]
        % latest state
        Locked logical = false
        Mode double = NaN
        Lead double = NaN
        Rbase double = eye(3)
        Rbrl double = eye(3)
        Az double = 0
        El double = 0
        % point lock (firmware v2)
        RangeM double = 0              % target distance [m] (0 = direction lock)
        Piv double = [0; 0; 0]         % pivot displacement since lock [m], base-IMU world
        TP double = [0; 0; 0]          % target point relative to the lock pivot [m], base-IMU world
        HasPoint logical = false       % the log carries point-lock columns
        RdLock double = eye(3)         % display attitude of the base at the lock
        % lock / view reference
        Rref double = eye(3)
        Yaw0 double = 0
        TargetW double = []            % locked direction in base-IMU world
        Roff double = eye(3)           % barrel-IMU world -> base-IMU world (from the lock)
        HaveRef logical = false
        LockCount double = 0
    end

    properties (Access = private)
        C struct
    end

    methods
        function obj = TurretState(cols)
            obj.C = cols;
        end

        function setCols(obj, cols)
            obj.C = cols;
        end

        function push(obj, D)
            if isempty(D) || ~isfield(obj.C, 'bq_w'), return; end
            c = obj.C;
            has = @(f) isfield(c, f);
            n = size(D, 1);
            tt = D(:, c.t_ms) / 1000;
            br = nan(n, 1);
            for i = 1:n
                obj.Rbase = TurretMath.quatToRot(D(i, [c.bq_w c.bq_x c.bq_y c.bq_z]));
                obj.Rbrl  = TurretMath.quatToRot(D(i, [c.rq_w c.rq_x c.rq_y c.rq_z]));
                obj.Az = D(i, c.meas_az);
                obj.El = D(i, c.meas_el);
                lk = has('locked') && D(i, c.locked) == 1;
                if ~obj.HaveRef || (lk && ~obj.Locked)
                    obj.setReference(lk);
                end
                if ~lk && obj.Locked
                    obj.TargetW = [];
                end
                obj.Locked = lk;
                br(i) = TurretMath.rotAngle(obj.Rref' * obj.Rbase);
            end
            obj.HasPoint = has('piv_x') && has('tp_x') && has('range_m');
            if obj.HasPoint
                obj.RangeM = D(end, c.range_m);
                obj.Piv = D(end, [c.piv_x c.piv_y c.piv_z])' / 100;
                obj.TP = D(end, [c.tp_x c.tp_y c.tp_z])';
            end
            if has('mode'), obj.Mode = D(end, c.mode); end
            if has('lead_ms'), obj.Lead = D(end, c.lead_ms); end
            get = @(f) TurretState.col(D, c, f);
            obj.t       = [obj.t; tt];
            obj.baseRot = [obj.baseRot; br];
            obj.errTot  = [obj.errTot; get('err_tot')];
            obj.errAz   = [obj.errAz; get('err_az')];
            obj.errEl   = [obj.errEl; get('err_el')];
            obj.cmdAz   = [obj.cmdAz; get('cmd_az')];
            obj.measAz  = [obj.measAz; get('meas_az')];
            obj.cmdEl   = [obj.cmdEl; get('cmd_el')];
            obj.measEl  = [obj.measEl; get('meas_el')];
            obj.lockedBuf = [obj.lockedBuf; get('locked')];
            w = sqrt(get('bw_x').^2 + get('bw_y').^2 + get('bw_z').^2);
            obj.rateBuf = [obj.rateBuf; w];
            obj.pivX = [obj.pivX; get('piv_x')];
            obj.pivY = [obj.pivY; get('piv_y')];
            obj.pivZ = [obj.pivZ; get('piv_z')];
            obj.zuptBuf = [obj.zuptBuf; get('zupt')];
            keep = obj.t >= obj.t(end) - obj.WindowS;
            if ~all(keep)
                f = {'t', 'baseRot', 'errTot', 'errAz', 'errEl', 'cmdAz', 'measAz', ...
                     'cmdEl', 'measEl', 'lockedBuf', 'rateBuf', 'pivX', 'pivY', 'pivZ', 'zuptBuf'};
                for k = 1:numel(f), obj.(f{k}) = obj.(f{k})(keep); end
            end
        end

        function [Rd, az, el, tgt, brl] = displayPose(obj)
            Rv = TurretMath.rotz(-obj.Yaw0);
            Rd = Rv * obj.Rbase;
            az = obj.Az; el = obj.El;
            if obj.Locked && ~isempty(obj.TargetW)
                tgt = Rv * obj.TargetW;
                brl = Rv * obj.Roff * obj.Rbrl * [0; 1; 0];
            else
                tgt = []; brl = [];
            end
        end

        function [piv, tp, RdLock] = pointPose(obj)
            % Point lock in the display frame: pivot displacement since the lock and the
            % target point relative to the lock pivot [m]. tp = [] for a direction lock.
            Rv = TurretMath.rotz(-obj.Yaw0);
            RdLock = obj.RdLock;
            if ~obj.HasPoint || ~obj.Locked
                piv = [0; 0; 0]; tp = [];             % unlocked: the estimate is not anchored to anything
                return;
            end
            piv = Rv * obj.Piv;
            if obj.RangeM > 0, tp = Rv * obj.TP; else, tp = []; end
        end

        function m = metrics(obj, windowS)
            m = struct('errRms', NaN, 'errMax', NaN, 'baseRms', NaN, 'reduction', NaN, 'rateRms', NaN);
            if isempty(obj.t), return; end
            sel = obj.t >= obj.t(end) - windowS & obj.lockedBuf == 1;
            if nnz(sel) < 20, return; end
            e = obj.errTot(sel); b = obj.baseRot(sel);
            m.errRms = sqrt(mean(e.^2));
            m.errMax = max(e);
            m.baseRms = sqrt(mean(b.^2));
            m.rateRms = sqrt(mean(obj.rateBuf(sel).^2));
            m.reduction = 100 * (1 - m.errRms / max(m.baseRms, 1e-6));
        end
    end

    methods (Access = private)
        function setReference(obj, isLock)
            obj.Rref = obj.Rbase;
            obj.Yaw0 = TurretMath.heading(obj.Rbase);
            obj.HaveRef = true;
            obj.RdLock = TurretMath.rotz(-obj.Yaw0) * obj.Rbase;
            if isLock
                Rt = TurretMath.turretRot(obj.Az, obj.El);
                obj.TargetW = obj.Rbase * Rt * [0; 1; 0];
                obj.Roff = obj.Rbase * Rt * obj.Rbrl';
                obj.LockCount = obj.LockCount + 1;
            end
        end
    end

    methods (Static)
        function v = col(D, c, f)
            if isfield(c, f), v = D(:, c.(f)); else, v = nan(size(D, 1), 1); end
        end
    end
end
