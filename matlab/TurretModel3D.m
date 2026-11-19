classdef TurretModel3D < handle
    % TurretModel3D  Animated 3D mock-up of the turret on any axes (axes or uiaxes).
    %   m = TurretModel3D(ax);
    %   m.update(Rdisp, az, el, targetDir, barrelDir)
    %     Rdisp      3x3 base attitude in the display frame
    %     az, el     joint angles [deg] (servo measured)
    %     targetDir  3x1 locked pointing direction in the display frame ([] = not locked)
    %     barrelDir  3x1 barrel direction measured by the barrel IMU ([] = hide)
    %   Hierarchy: base plate -> azimuth turntable + yoke -> elevation barrel.

    properties
        Ax
        PivotZ double = 0.85      % height of the elevation axis above the plate
    end

    properties (Access = private)
        tBase; tAz; tEl
        hTarget; hBarrelRay; hTargetDot; hShadow
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
            light(ax, 'Position', [2 -3 5], 'Style', 'infinite');
            light(ax, 'Position', [-3 2 2], 'Style', 'infinite');
            lighting(ax, 'gouraud');
            obj.update(eye(3), 0, 0, [], []);
        end

        function update(obj, Rdisp, az, el, targetDir, barrelDir)
            obj.tBase.Matrix = [Rdisp, zeros(3, 1); 0 0 0 1];
            obj.tAz.Matrix = makehgtform('zrotate', deg2rad(az));
            obj.tEl.Matrix = makehgtform('translate', [0 0 obj.PivotZ], 'xrotate', deg2rad(el));
            p = Rdisp * [0; 0; obj.PivotZ];
            TurretModel3D.setRay(obj.hTarget, p, targetDir, 3.2);
            if isempty(targetDir)
                set(obj.hTargetDot, 'XData', nan, 'YData', nan, 'ZData', nan);
            else
                q = p + 3.2 * targetDir(:);
                set(obj.hTargetDot, 'XData', q(1), 'YData', q(2), 'ZData', q(3));
            end
            TurretModel3D.setRay(obj.hBarrelRay, p, barrelDir, 2.8);
            % floor "shadow" of the plate outline helps read tilt
            c = Rdisp * [-1 1 1 -1 -1; -1.3 -1.3 1.3 1.3 -1.3; 0 0 0 0 0];
            set(obj.hShadow, 'XData', c(1, :), 'YData', c(2, :), 'ZData', -1.6 * ones(1, 5));
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
