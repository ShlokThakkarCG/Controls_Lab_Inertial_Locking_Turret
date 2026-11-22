classdef TurretModel3D < handle
    % TurretModel3D  Animated 3D mock-up of the turret on any axes (axes or uiaxes).
    %   m = TurretModel3D(ax);
    %   m.update(Rdisp, az, el, targetDir, barrelDir)
    %     Rdisp      3x3 base attitude in the display frame
    %     az, el     joint angles [deg] (servo measured)
    %     targetDir  3x1 locked pointing direction in the display frame ([] = not locked)
    %     barrelDir  3x1 barrel direction measured by the barrel IMU ([] = hide)
    %   m.update(..., pivShift, targetPt, RdLock)       point lock (firmware v2)
    %     pivShift   3x1 pivot displacement since the lock [m] (display frame)
    %     targetPt   3x1 locked point relative to the pivot at lock [m] ([] = direction lock)
    %     RdLock     3x3 base attitude at the lock (to place the lock pivot)
    %   Model units: 1 unit = UnitM metres (the plate is 2 x 2.6 units).
    %   Hierarchy: base plate -> azimuth turntable + yoke -> elevation barrel.

    properties
        Ax
        PivotZ double = 0.85      % height of the elevation axis above the plate
        UnitM double = 0.1        % metres per model unit (true scale for translation and range)
        TrailN double = 300       % pivot path samples kept
    end

    properties (Access = private)
        tBase; tAz; tEl
        hTarget; hBarrelRay; hTargetDot; hShadow
        hPoint; hPointLbl; hGhost; hTrail; hDrop
        Trail = zeros(3, 0)
        BaseLim = [2.6 -1.6 2.4]
    end

    methods
        function obj = TurretModel3D(ax)
            obj.Ax = ax;
            hold(ax, 'on');
            axis(ax, 'equal');
            L = 2.6;
            xlim(ax, [-L L]); ylim(ax, [-L L]); zlim(ax, [-1.6 2.4]);
            view(ax, 140, 20);
            grid(ax, 'on');
            box(ax, 'on');
            ax.Clipping = 'off';
            xlabel(ax, 'X  right'); ylabel(ax, 'Y  forward'); zlabel(ax, 'Z  up');

            obj.tBase = hgtransform('Parent', ax);
            obj.tAz   = hgtransform('Parent', obj.tBase);
            obj.tEl   = hgtransform('Parent', obj.tAz);

            plate  = [0.58 0.62 0.68]; dark = [0.26 0.28 0.32];
            barrel = [0.16 0.42 0.72]; accent = [0.96 0.72 0.12];
            TurretModel3D.box(obj.tBase, [-1.0 1.0], [-1.3 1.3], [-0.07 0], plate);
            TurretModel3D.box(obj.tBase, [-0.06 0.06], [0.85 1.25], [0 0.012], accent);     % "forward" stripe
            TurretModel3D.box(obj.tBase, [0.55 0.85], [-1.0 -0.7], [0 0.05], [0.2 0.6 0.3]); % base IMU
            TurretModel3D.cylZ(obj.tAz, 0.36, [0 0.22], dark);                               % turntable
            TurretModel3D.box(obj.tAz, [-0.40 -0.31], [-0.11 0.11], [0.22 obj.PivotZ + 0.1], dark);
            TurretModel3D.box(obj.tAz, [ 0.31  0.40], [-0.11 0.11], [0.22 obj.PivotZ + 0.1], dark);
            TurretModel3D.cylY(obj.tEl, 0.085, [-0.30 1.55], barrel);                         % barrel
            TurretModel3D.cylY(obj.tEl, 0.11, [1.40 1.58], [0.1 0.25 0.45]);                 % muzzle
            TurretModel3D.box(obj.tEl, [-0.12 0.12], [-0.50 -0.28], [-0.11 0.11], [0.2 0.6 0.3]); % barrel IMU

            obj.hTarget    = plot3(ax, nan, nan, nan, '-',  'Color', [0.10 0.70 0.30], 'LineWidth', 3);
            obj.hTargetDot = plot3(ax, nan, nan, nan, 'o',  'Color', [0.10 0.70 0.30], ...
                                   'MarkerFaceColor', [0.10 0.70 0.30], 'MarkerSize', 9);
            obj.hBarrelRay = plot3(ax, nan, nan, nan, '--', 'Color', [0.85 0.20 0.20], 'LineWidth', 1.4);
            obj.hShadow    = plot3(ax, nan, nan, nan, '-',  'Color', [0.6 0.6 0.6], 'LineWidth', 1);
            % point lock: the locked point, its drop line to the floor, the pivot at lock and the pivot's path
            obj.hPoint  = plot3(ax, nan, nan, nan, 'p', 'MarkerSize', 22, 'Color', [0.85 0.1 0.45], ...
                                'MarkerFaceColor', [1.0 0.45 0.7], 'LineWidth', 1.2);
            obj.hPointLbl = text(ax, nan, nan, nan, '', 'Color', [0.65 0.05 0.35], 'FontWeight', 'bold', ...
                                 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
            obj.hDrop   = plot3(ax, nan, nan, nan, ':', 'Color', [0.85 0.1 0.45], 'LineWidth', 1);
            obj.hGhost  = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 10, 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
            obj.hTrail  = plot3(ax, nan, nan, nan, '-', 'Color', [0.55 0.25 0.8], 'LineWidth', 1.6);
            light(ax, 'Position', [2 -3 5], 'Style', 'infinite');
            light(ax, 'Position', [-3 2 2], 'Style', 'infinite');
            lighting(ax, 'gouraud');
            obj.update(eye(3), 0, 0, [], []);
        end

        function update(obj, Rdisp, az, el, targetDir, barrelDir, pivShift, targetPt, RdLock)
            if nargin < 7 || isempty(pivShift), pivShift = [0; 0; 0]; end
            if nargin < 8, targetPt = []; end
            if nargin < 9, RdLock = Rdisp; end
            pivShift(~isfinite(pivShift)) = 0;
            % The pivot sits at (lock pivot + measured shift); the plate is placed under it.
            pLock = RdLock * [0; 0; obj.PivotZ];
            p = pLock + pivShift(:) / obj.UnitM;
            origin = p - Rdisp * [0; 0; obj.PivotZ];
            obj.tBase.Matrix = [Rdisp, origin; 0 0 0 1];
            obj.tAz.Matrix = makehgtform('zrotate', deg2rad(az));
            obj.tEl.Matrix = makehgtform('translate', [0 0 obj.PivotZ], 'xrotate', deg2rad(el));
            pointLock = ~isempty(targetPt) && ~isempty(targetDir);
            if pointLock
                q = pLock + targetPt(:) / obj.UnitM;            % the locked point (fixed in space)
                len = norm(q - p);
                set(obj.hTarget, 'XData', [p(1) q(1)], 'YData', [p(2) q(2)], 'ZData', [p(3) q(3)]);
                set(obj.hTargetDot, 'XData', nan, 'YData', nan, 'ZData', nan);
                set(obj.hPoint, 'XData', q(1), 'YData', q(2), 'ZData', q(3));
                set(obj.hPointLbl, 'Position', q' + [0 0 0.35], 'String', sprintf('%.2f m', len * obj.UnitM));
                set(obj.hDrop, 'XData', [q(1) q(1)], 'YData', [q(2) q(2)], 'ZData', [q(3) obj.floorZ()]);
                set(obj.hGhost, 'XData', pLock(1), 'YData', pLock(2), 'ZData', pLock(3));
                obj.Trail(:, end + 1) = p;
                if size(obj.Trail, 2) > obj.TrailN, obj.Trail = obj.Trail(:, end - obj.TrailN + 1:end); end
                set(obj.hTrail, 'XData', obj.Trail(1, :), 'YData', obj.Trail(2, :), 'ZData', obj.Trail(3, :));
                TurretModel3D.setRay(obj.hBarrelRay, p, barrelDir, len);
                obj.fitView([p q pLock]);
            else
                TurretModel3D.setRay(obj.hTarget, p, targetDir, 3.2);
                if isempty(targetDir)
                    set(obj.hTargetDot, 'XData', nan, 'YData', nan, 'ZData', nan);
                else
                    q = p + 3.2 * targetDir(:);
                    set(obj.hTargetDot, 'XData', q(1), 'YData', q(2), 'ZData', q(3));
                end
                set([obj.hPoint obj.hDrop obj.hGhost obj.hTrail], 'XData', nan, 'YData', nan, 'ZData', nan);
                set(obj.hPointLbl, 'String', '');
                obj.Trail = zeros(3, 0);
                TurretModel3D.setRay(obj.hBarrelRay, p, barrelDir, 2.8);
                obj.fitView(p);
            end
            % floor "shadow" of the plate outline helps read tilt
            c = Rdisp * [-1 1 1 -1 -1; -1.3 -1.3 1.3 1.3 -1.3; 0 0 0 0 0];
            c = c + origin;
            set(obj.hShadow, 'XData', c(1, :), 'YData', c(2, :), 'ZData', obj.floorZ() * ones(1, 5));
        end
    end

    methods (Access = private)
        function z = floorZ(obj)
            z = obj.Ax.ZLim(1);
        end

        function fitView(obj, pts)
            % Grow the axes to keep the turret and the locked point in view; shrink back when unlocked.
            L = obj.BaseLim(1);
            lo = [-L -L obj.BaseLim(2)]; hi = [L L obj.BaseLim(3)];
            lo = min(lo, min(pts, [], 2)' - 1.2);
            hi = max(hi, max(pts, [], 2)' + 1.2);
            span = max(hi(1:2) - lo(1:2));                     % keep x/y square
            cxy = (hi(1:2) + lo(1:2)) / 2;
            lo(1:2) = cxy - span / 2; hi(1:2) = cxy + span / 2;
            cur = [obj.Ax.XLim; obj.Ax.YLim; obj.Ax.ZLim];
            want = [lo' hi'];
            if any(abs(cur(:) - want(:)) > 0.5)                % avoid jitter: only move on real changes
                obj.Ax.XLim = want(1, :); obj.Ax.YLim = want(2, :); obj.Ax.ZLim = want(3, :);
            end
        end
    end

    methods (Static)
        function setRay(h, p, d, len)
            if isempty(d)
                set(h, 'XData', nan, 'YData', nan, 'ZData', nan);
            else
                q = p + len * d(:);
                set(h, 'XData', [p(1) q(1)], 'YData', [p(2) q(2)], 'ZData', [p(3) q(3)]);
            end
        end

        function h = box(parent, xr, yr, zr, col)
            [X, Y, Z] = ndgrid(xr, yr, zr);
            V = [X(:) Y(:) Z(:)];
            F = [1 3 4 2; 5 6 8 7; 1 2 6 5; 3 7 8 4; 1 5 7 3; 2 4 8 6];
            h = patch('Parent', parent, 'Vertices', V, 'Faces', F, 'FaceColor', col, ...
                      'EdgeColor', col * 0.7, 'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
        end

        function h = cylZ(parent, r, zr, col)
            [x, y, z] = cylinder(r, 32);
            z = zr(1) + z * diff(zr);
            h = surf(x, y, z, 'Parent', parent, 'FaceColor', col, 'EdgeColor', 'none', 'AmbientStrength', 0.5);
            TurretModel3D.cap(parent, x(1, :), y(1, :), zr(2) * ones(1, size(x, 2)), col);
        end

        function h = cylY(parent, r, yr, col)
            [x, z, y] = cylinder(r, 32);
            y = yr(1) + y * diff(yr);
            h = surf(x, y, z, 'Parent', parent, 'FaceColor', col, 'EdgeColor', 'none', 'AmbientStrength', 0.5);
            TurretModel3D.cap(parent, x(1, :), yr(2) * ones(1, size(x, 2)), z(1, :), col);
            TurretModel3D.cap(parent, x(1, :), yr(1) * ones(1, size(x, 2)), z(1, :), col);
        end

        function cap(parent, x, y, z, col)
            patch('Parent', parent, 'XData', x, 'YData', y, 'ZData', z, 'FaceColor', col, ...
                  'EdgeColor', 'none', 'AmbientStrength', 0.5);
        end
    end
end
