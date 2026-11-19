%% ENR 312 Control Engineering Lab - Simple 2-Axis Turret Stabilization
% Base MATLAB only (no toolboxes).
% Compares:
% (1) No Control
% (2) Feedback Only
% (3) Feedback + Feedforward

clear; clc; close all;

%% 1. ASSUMED PARAMETERS (replace with measured values later)

sample_time = 0.005;    % [s] Loop time step (200 Hz)
total_time  = 10.0;     % [s] Total simulation time
step_time   = 4.0;      % [s] Time of sudden base step

% Base 3D disturbance: sinusoids + sudden step [deg] and [Hz]

yaw_amplitude = 12.0;
yaw_freq_hz   = 0.35;
yaw_step_deg  = -15.0;

pitch_amplitude = 8.0;
pitch_freq_hz   = 0.50;
pitch_step_deg  = 10.0;

roll_amplitude = 10.0;
roll_freq_hz   = 0.40;
roll_step_deg  = 8.0;

% Initial servo angles at lock [deg]
% Arbitrary IMU yaw offset [deg]

azimuth_lock_deg   = 15.0;
elevation_lock_deg = 20.0;
imu_yaw_offset_deg = 40.0;

% Servo model: 1st-order lag + rate limit + joint angle limits

servo_lag_sec     = 0.04;    % [s] Servo lag time constant
servo_rate_limit  = 300.0;   % [deg/s] Maximum servo speed

azimuth_limit_deg = 120.0;   % [deg] Azimuth travel limit (+-120 deg)
elevation_min_deg = -30.0;   % [deg] Elevation lower limit
elevation_max_deg = 75.0;    % [deg] Elevation upper limit

% Plain PI controller gains on barrel pointing error

proportional_gain = 1.20;
integral_gain     = 12.0;

%% 2. BUILD TIME VECTOR & 3D BASE DISTURBANCE

time_vec  = (0:sample_time:total_time)';
num_steps = length(time_vec);

step_profile = 1 - exp(-max(0, time_vec - step_time) / 0.02);

yaw_deg = yaw_amplitude * sin(2*pi*yaw_freq_hz*time_vec) ...
          + yaw_step_deg * step_profile;

pitch_deg = pitch_amplitude * sin(2*pi*pitch_freq_hz*time_vec) ...
            + pitch_step_deg * step_profile;

roll_deg = roll_amplitude * sin(2*pi*roll_freq_hz*time_vec) ...
           + roll_step_deg * step_profile;

deg_to_rad = pi / 180;

yaw   = yaw_deg   * deg_to_rad;
pitch = pitch_deg * deg_to_rad;
roll  = roll_deg  * deg_to_rad;

azimuth_0   = azimuth_lock_deg * deg_to_rad;
elevation_0 = elevation_lock_deg * deg_to_rad;

offset_rot = rotate_z(-imu_yaw_offset_deg * deg_to_rad);

%% 3. LOCK STEP AT t = 0 AND ORTHONORMALITY SELF-CHECK

base_rot_0 = quat_to_matrix( ...
    euler_to_quat(yaw(1), pitch(1), roll(1)));

turret_rot_0 = rotate_z(azimuth_0) * rotate_y(-elevation_0);

barrel_rot_0 = quat_to_matrix( ...
    matrix_to_quat(offset_rot * base_rot_0 * turret_rot_0));

locked_dir_base_world = base_rot_0 * ...
    [cos(elevation_0)*cos(azimuth_0);
     cos(elevation_0)*sin(azimuth_0);
     sin(elevation_0)];

locked_dir_barrel_world = barrel_rot_0 * [1; 0; 0];

true_locked_direction = base_rot_0 * turret_rot_0 * [1; 0; 0];

if norm(base_rot_0' * base_rot_0 - eye(3)) < 1e-10 && ...
   norm(barrel_rot_0' * barrel_rot_0 - eye(3)) < 1e-10

    fprintf('SELF-CHECK: PASS\n');

else

    fprintf('SELF-CHECK: FAIL\n');

end

%% 4. SIMULATE 3 CASES
% (1) No Control
% (2) Feedback Only
% (3) Feedback + Feedforward

error_deg = zeros(num_steps, 3);

for mode = 1:3

    azimuth = azimuth_0;
    elevation = elevation_0;

    integral_azimuth = 0;
    integral_elevation = 0;

    for k = 1:num_steps

        %% 1. READ & ROTATE: Get rotation matrices from IMU quaternions

        base_quat = euler_to_quat(yaw(k), pitch(k), roll(k));

        base_rot = quat_to_matrix(base_quat);

        turret_rot = rotate_z(azimuth) * rotate_y(-elevation);

        barrel_quat = matrix_to_quat( ...
            offset_rot * base_rot * turret_rot);

        barrel_rot = quat_to_matrix(barrel_quat);

        %% 2. TRUE POINTING ERROR ANGLE IN DEGREES

        barrel_dir_true = base_rot * turret_rot * [1; 0; 0];

        error_deg(k, mode) = atan2( ...
            norm(cross_product(barrel_dir_true, true_locked_direction)), ...
            dot(barrel_dir_true, true_locked_direction)) / deg_to_rad;

        %% 3. FEEDFORWARD (BASE IMU): Counter-rotate base motion

        if mode == 3

            target_in_base = base_rot' * locked_dir_base_world;

            azimuth_feedforward = atan2( ...
                target_in_base(2), target_in_base(1));

            elevation_feedforward = atan2( ...
                target_in_base(3), ...
                hypot(target_in_base(1), target_in_base(2)));

        else

            azimuth_feedforward = azimuth_0;
            elevation_feedforward = elevation_0;

        end

        %% 4. FEEDBACK (BARREL IMU): PI on cross-product pointing error

        if mode >= 2

            barrel_dir_world = barrel_rot * [1; 0; 0];

            error_vector_world = cross_product( ...
                barrel_dir_world, locked_dir_barrel_world);

            error_vector_barrel = barrel_rot' * error_vector_world;

            azimuth_error = error_vector_barrel(3);

            elevation_error = -error_vector_barrel(2);

            integral_azimuth = integral_azimuth ...
                + integral_gain * azimuth_error * sample_time;

            integral_elevation = integral_elevation ...
                + integral_gain * elevation_error * sample_time;

            azimuth_feedback = proportional_gain * azimuth_error ...
                + integral_azimuth;

            elevation_feedback = proportional_gain * elevation_error ...
                + integral_elevation;

        else

            azimuth_feedback = 0;
            elevation_feedback = 0;

        end

        %% 5. FINAL SERVO COMMAND & SERVO UPDATE
        % Includes lag, rate limit and joint angle limits

        azimuth_command = min(max( ...
            azimuth_feedforward + azimuth_feedback, ...
            -azimuth_limit_deg * deg_to_rad), ...
            azimuth_limit_deg * deg_to_rad);

        elevation_command = min(max( ...
            elevation_feedforward + elevation_feedback, ...
            elevation_min_deg * deg_to_rad), ...
            elevation_max_deg * deg_to_rad);

        azimuth = servo_step( ...
            azimuth, azimuth_command, ...
            servo_lag_sec, servo_rate_limit * deg_to_rad, ...
            -azimuth_limit_deg * deg_to_rad, ...
            azimuth_limit_deg * deg_to_rad, sample_time);

        elevation = servo_step( ...
            elevation, elevation_command, ...
            servo_lag_sec, servo_rate_limit * deg_to_rad, ...
            elevation_min_deg * deg_to_rad, ...
            elevation_max_deg * deg_to_rad, sample_time);

    end

end

%% 5. PRINT SUMMARY TABLE & SAVE PLOTS

case_names = { ...
    '1. No Control', ...
    '2. Feedback Only', ...
    '3. FB + Feedforward'};

fprintf('\n%-22s | %-14s | %-14s\n', ...
    'Case', 'Max Error (deg)', 'RMS Error (deg)');

fprintf('----------------------------------------------------------\n');

for mode = 1:3

    fprintf('%-22s | %14.2f | %14.2f\n', ...
        case_names{mode}, ...
        max(error_deg(:, mode)), ...
        sqrt(mean(error_deg(:, mode).^2)));

end

%% PLOT 1: BARREL POINTING ERROR

fig1 = figure('Color', 'w');

plot(time_vec, error_deg(:,1), 'r--', ...
     time_vec, error_deg(:,2), 'b-.', ...
     time_vec, error_deg(:,3), 'g-', ...
     'LineWidth', 1.6);

grid on;

xlabel('Time (s)');
ylabel('Pointing Error (deg)');

title('Barrel Pointing Error vs. Time');

legend(case_names, 'Location', 'northwest');

saveas(fig1, 'plot1_pointing_error.png');

%% PLOT 2: BASE DISTURBANCE ANGLES

fig2 = figure('Color', 'w');

plot(time_vec, yaw_deg, 'r-', ...
     time_vec, pitch_deg, 'b-', ...
     time_vec, roll_deg, 'k-', ...
     'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Base Angle (deg)');

title('3D Base Disturbance Angles vs. Time');

legend('Yaw', 'Pitch', 'Roll', 'Location', 'northwest');

saveas(fig2, 'plot2_base_disturbance.png');

%% 6. LOCAL FUNCTIONS

function R = rotate_x(angle)

    c = cos(angle);
    s = sin(angle);

    R = [1 0 0;
         0 c -s;
         0 s c];

end

function R = rotate_y(angle)

    c = cos(angle);
    s = sin(angle);

    R = [c 0 s;
         0 1 0;
         -s 0 c];

end

function R = rotate_z(angle)

    c = cos(angle);
    s = sin(angle);

    R = [c -s 0;
         s c 0;
         0 0 1];

end

function c = cross_product(a, b)

    c = [a(2)*b(3) - a(3)*b(2);
         a(3)*b(1) - a(1)*b(3);
         a(1)*b(2) - a(2)*b(1)];

end

function q = euler_to_quat(yaw_ang, pitch_ang, roll_ang)

    q = matrix_to_quat( ...
        rotate_z(yaw_ang) * ...
        rotate_y(pitch_ang) * ...
        rotate_x(roll_ang));

end

function q = matrix_to_quat(R)

    qw = 0.5 * sqrt(max( ...
        1 + R(1,1) + R(2,2) + R(3,3), 1e-12));

    qx = (R(3,2) - R(2,3)) / (4*qw);

    qy = (R(1,3) - R(3,1)) / (4*qw);

    qz = (R(2,1) - R(1,2)) / (4*qw);

    q = [qw; qx; qy; qz];

    q = q / norm(q);

end

function R = quat_to_matrix(q)

    q = q / norm(q);

    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    R = [1-2*(qy^2+qz^2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
         2*(qx*qy+qw*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qw*qx);
         2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx^2+qy^2)];

end

function pos_next = servo_step(pos, pos_cmd, lag_sec, ...
                               rate_lim, pos_min, pos_max, dt)

    vel = min(max((pos_cmd - pos) / lag_sec, -rate_lim), rate_lim);

    pos_next = min(max(pos + vel * dt, pos_min), pos_max);

end