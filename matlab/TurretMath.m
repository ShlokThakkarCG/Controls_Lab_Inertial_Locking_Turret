classdef TurretMath
    % TurretMath  Frame conventions and kinematics shared with the firmware.
    %   Every body frame: +X right, +Y forward (barrel), +Z up. Angles in degrees.
    %   +azimuth = barrel turns LEFT, +elevation = barrel tilts UP.
    %   Turret forward kinematics: R_turret(az, el) = Rz(az) * Rx(el).

    methods (Static)
        function R = quatToRot(q)
            % q = [w x y z] (1x4). Returns the 3x3 body-to-world rotation.
            q = q / norm(q);
            w = q(1); x = q(2); y = q(3); z = q(4);
            R = [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w);
                 2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w);
                 2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)];
        end

        function R = rotz(deg)
            c = cosd(deg); s = sind(deg);
            R = [c -s 0; s c 0; 0 0 1];
        end

        function R = rotx(deg)
            c = cosd(deg); s = sind(deg);
            R = [1 0 0; 0 c -s; 0 s c];
        end

        function R = turretRot(az, el)
            R = TurretMath.rotz(az) * TurretMath.rotx(el);
        end

        function yaw = heading(R)
            % Heading of the body's +Y axis about world Z (+ = turned left).
            f = R(:, 2);
            yaw = atan2d(-f(1), f(2));
        end

        function [az, el] = dirToAzEl(d)
            az = atan2d(-d(1), d(2));
            el = atan2d(d(3), hypot(d(1), d(2)));
        end

        function ang = rotAngle(R)
            % Rotation angle [deg] of a rotation matrix.
            ang = acosd(max(-1, min(1, (trace(R) - 1) / 2)));
        end
    end
end
