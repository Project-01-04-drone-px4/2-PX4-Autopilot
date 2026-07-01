%% Landing target Kalman filter Simulink demo
% This script creates and runs a Simulink model that mirrors the small
% 2-state KalmanFilter used by PX4 landing_target_estimator.
%
% PX4 source mapping:
%   state x = [relative_position; relative_velocity]
%   predict:
%       p = p + v*dt + 0.5*a*dt^2
%       v = v + a*dt
%       P = A*P*A' + G*G'*acc_unc
%   update:
%       H = [1 0]
%       residual = meas - p
%       innov_cov = P(1,1) + meas_unc
%       beta = residual^2 / innov_cov
%       reject if beta > 3.84
%       K = [P(1,1); P(2,1)] / innov_cov
%       x = x + K*residual
%       P = (I - K*H)*P

clear;
clc;
close all;

%% Simulation setup
rng(7);

script_dir = fileparts(mfilename("fullpath"));
model_name = "landing_target_kf_sim";
Ts = 0.02;
t_end = 30;
t = (0:Ts:t_end)';
n = numel(t);

% PX4-like parameters. These are variances, not standard deviations.
acc_unc = 10.0;          % LTEST_ACC_UNC, unit: (m/s^2)^2
meas_unc_param = 0.005;  % LTEST_MEAS_UNC, unit: tan(rad)^2
dist_z = 5.0;            % height used to convert angular uncertainty to m^2
meas_unc = meas_unc_param * dist_z^2;
pos_unc_init = 0.1;      % LTEST_POS_UNC_IN, unit: m^2
vel_unc_init = 0.1;      % LTEST_VEL_UNC_IN, unit: (m/s)^2
gate_threshold = 3.84;   % chi-square threshold, 1 dof, 95%

% Create a true relative target acceleration profile.
true_acc = 0.25*sin(0.7*t) + 0.10*sin(2.2*t);
true_acc(t > 8 & t < 12) = true_acc(t > 8 & t < 12) + 0.35;
true_acc(t > 19 & t < 22) = true_acc(t > 19 & t < 22) - 0.45;

true_pos = zeros(n, 1);
true_vel = zeros(n, 1);
true_pos(1) = 1.0;
true_vel(1) = -0.15;

for k = 2:n
    true_pos(k) = true_pos(k - 1) + true_vel(k - 1)*Ts + 0.5*true_acc(k - 1)*Ts^2;
    true_vel(k) = true_vel(k - 1) + true_acc(k - 1)*Ts;
end

% PX4 predict() receives an acceleration estimate. Here it is noisy.
acc_noise_std = 0.60;
acc_noisy = true_acc + acc_noise_std*randn(n, 1);

% Measurement is direct relative position with height-scaled uncertainty.
meas_noise_std = sqrt(meas_unc);
meas_pos = true_pos + meas_noise_std*randn(n, 1);

% Add a few deliberate outliers so the PX4-style innovation gate is visible.
outlier_idx = find((t > 13.0 & t < 13.20) | (t > 24.0 & t < 24.14));
meas_pos(outlier_idx) = meas_pos(outlier_idx) + 3.0;

% Simulink From Workspace inputs.
acc_noisy_ts = timeseries(acc_noisy, t);
meas_pos_ts = timeseries(meas_pos, t);

assignin("base", "Ts", Ts);
assignin("base", "acc_unc", acc_unc);
assignin("base", "meas_unc", meas_unc);
assignin("base", "pos_unc_init", pos_unc_init);
assignin("base", "vel_unc_init", vel_unc_init);
assignin("base", "gate_threshold", gate_threshold);
assignin("base", "acc_noisy_ts", acc_noisy_ts);
assignin("base", "meas_pos_ts", meas_pos_ts);

%% Build Simulink model
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

model_file = fullfile(script_dir, model_name + ".slx");

if isfile(model_file)
    delete(model_file);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, ...
    "Solver", "FixedStepDiscrete", ...
    "FixedStep", "Ts", ...
    "StopTime", num2str(t_end), ...
    "SaveOutput", "off", ...
    "SignalLogging", "off");

add_block("simulink/Sources/From Workspace", model_name + "/Noisy acceleration", ...
    "VariableName", "acc_noisy_ts", ...
    "Position", [40 70 180 100]);

add_block("simulink/Sources/From Workspace", model_name + "/Noisy position measurement", ...
    "VariableName", "meas_pos_ts", ...
    "Position", [40 145 180 175]);

add_block("simulink/User-Defined Functions/MATLAB Function", model_name + "/PX4 KalmanFilter update", ...
    "Position", [280 70 500 190]);

kalman_block_code = [
    "function y = px4_landing_target_kf(acc, meas)"
    "%#codegen"
    "% y = [pos_est; vel_est; cov_pos; cov_vel; residual; innov_cov; accepted]"
    "persistent x0 x1 P00 P01 P10 P11 initialized"
    "if isempty(initialized)"
    "    x0 = meas;"
    "    x1 = 0.0;"
    "    P00 = pos_unc_init;"
    "    P01 = 0.0;"
    "    P10 = 0.0;"
    "    P11 = vel_unc_init;"
    "    initialized = true;"
    "end"
    ""
    "% predict: same equations as KalmanFilter::predict(dt, acc, acc_unc)"
    "x0 = x0 + x1*Ts + 0.5*acc*Ts*Ts;"
    "x1 = x1 + acc*Ts;"
    ""
    "% P = A*P*A' + G*G'*acc_unc, A=[1 Ts;0 1], G=[0.5*Ts^2; Ts]"
    "q00 = 0.25*Ts^4*acc_unc;"
    "q01 = 0.5*Ts^3*acc_unc;"
    "q11 = Ts^2*acc_unc;"
    "P00p = P00 + Ts*P10 + Ts*P01 + Ts*Ts*P11 + q00;"
    "P01p = P01 + Ts*P11 + q01;"
    "P10p = P10 + Ts*P11 + q01;"
    "P11p = P11 + q11;"
    "P00 = P00p;"
    "P01 = P01p;"
    "P10 = P10p;"
    "P11 = P11p;"
    ""
    "% update: same equations as KalmanFilter::update(meas, measUnc)"
    "residual = meas - x0;"
    "innov_cov = P00 + meas_unc;"
    "beta = residual*residual / innov_cov;"
    "accepted = 1.0;"
    ""
    "if beta <= gate_threshold"
    "    K0 = P00 / innov_cov;"
    "    K1 = P10 / innov_cov;"
    "    x0 = x0 + K0*residual;"
    "    x1 = x1 + K1*residual;"
    ""
    "    % P = (I - K*H)*P, H=[1 0]"
    "    oldP00 = P00;"
    "    oldP01 = P01;"
    "    oldP10 = P10;"
    "    oldP11 = P11;"
    "    P00 = (1.0 - K0)*oldP00;"
    "    P01 = (1.0 - K0)*oldP01;"
    "    P10 = oldP10 - K1*oldP00;"
    "    P11 = oldP11 - K1*oldP01;"
    "else"
    "    accepted = 0.0;"
    "end"
    ""
    "y = [x0; x1; P00; P11; residual; innov_cov; accepted];"
    "end"
];

set_matlab_function_block_code(model_name + "/PX4 KalmanFilter update", kalman_block_code);

add_block("simulink/Sinks/To Workspace", model_name + "/kf_output", ...
    "VariableName", "kf_output", ...
    "SaveFormat", "Timeseries", ...
    "Position", [590 105 710 135]);

add_block("simulink/Sinks/Scope", model_name + "/Scope", ...
    "Position", [590 170 710 230]);

add_line(model_name, "Noisy acceleration/1", "PX4 KalmanFilter update/1", "autorouting", "on");
add_line(model_name, "Noisy position measurement/1", "PX4 KalmanFilter update/2", "autorouting", "on");
add_line(model_name, "PX4 KalmanFilter update/1", "kf_output/1", "autorouting", "on");
add_line(model_name, "PX4 KalmanFilter update/1", "Scope/1", "autorouting", "on");

save_system(model_name, model_file);

%% Run simulation
sim_out = sim(model_name);
kf_output = sim_out.get("kf_output");
kf_time = kf_output.Time;
kf_data = squeeze(kf_output.Data);

% MATLAB versions differ in To Workspace timeseries dimensions for vector
% signals. Normalize to one row per sample and one column per KF output.
if size(kf_data, 1) == 7 && size(kf_data, 2) == numel(kf_time)
    kf_data = kf_data';
elseif size(kf_data, 2) ~= 7 && size(kf_data, 1) == numel(kf_time)
    kf_data = reshape(kf_data, numel(kf_time), 7);
end

pos_est = kf_data(:, 1);
vel_est = kf_data(:, 2);
cov_pos = kf_data(:, 3);
cov_vel = kf_data(:, 4);
residual = kf_data(:, 5);
innov_cov = kf_data(:, 6);
accepted = kf_data(:, 7);

%% Plot results
figure("Name", "PX4 landing target Kalman filter Simulink demo", "Color", "w");

subplot(4, 1, 1);
plot(t, true_pos, "k-", "LineWidth", 1.4);
hold on;
plot(t, meas_pos, ".", "Color", [0.65 0.65 0.65], "MarkerSize", 5);
plot(kf_time, pos_est, "b-", "LineWidth", 1.4);
grid on;
ylabel("position (m)");
legend("true", "noisy measurement", "KF estimate", "Location", "best");
title("Relative landing target position");

subplot(4, 1, 2);
plot(t, true_vel, "k-", "LineWidth", 1.4);
hold on;
plot(kf_time, vel_est, "r-", "LineWidth", 1.4);
grid on;
ylabel("velocity (m/s)");
legend("true", "KF estimate", "Location", "best");
title("Relative landing target velocity");

subplot(4, 1, 3);
plot(t, true_acc, "k-", "LineWidth", 1.2);
hold on;
plot(t, acc_noisy, "Color", [0.25 0.65 0.25]);
grid on;
ylabel("accel (m/s^2)");
legend("true", "noisy input", "Location", "best");
title("Acceleration used by predict()");

subplot(4, 1, 4);
yyaxis left;
plot(kf_time, residual, "m-", "LineWidth", 1.2);
ylabel("residual (m)");
yyaxis right;
stairs(kf_time, accepted, "Color", [0.1 0.1 0.1], "LineWidth", 1.2);
ylim([-0.1 1.1]);
ylabel("accepted");
grid on;
xlabel("time (s)");
title("Innovation residual and PX4 chi-square gate result");

figure("Name", "KF covariance and innovation statistics", "Color", "w");

subplot(3, 1, 1);
plot(kf_time, cov_pos, "b-", "LineWidth", 1.3);
hold on;
plot(kf_time, cov_vel, "r-", "LineWidth", 1.3);
grid on;
ylabel("variance");
legend("position covariance", "velocity covariance", "Location", "best");
title("State covariance diagonal");

subplot(3, 1, 2);
plot(kf_time, residual.^2 ./ innov_cov, "LineWidth", 1.3);
hold on;
yline(gate_threshold, "r--", "gate = 3.84");
grid on;
ylabel("NIS");
title("Normalized innovation squared: residual^2 / innovation covariance");

subplot(3, 1, 3);
pos_error = pos_est - interp1(t, true_pos, kf_time);
vel_error = vel_est - interp1(t, true_vel, kf_time);
plot(kf_time, pos_error, "b-", "LineWidth", 1.2);
hold on;
plot(kf_time, vel_error, "r-", "LineWidth", 1.2);
grid on;
ylabel("error");
xlabel("time (s)");
legend("position error (m)", "velocity error (m/s)", "Location", "best");
title("Estimation error");

%% Print summary data
accepted_count = nnz(accepted > 0.5);
rejected_count = numel(accepted) - accepted_count;
rmse_pos = sqrt(mean((pos_est - interp1(t, true_pos, kf_time)).^2));
rmse_vel = sqrt(mean((vel_est - interp1(t, true_vel, kf_time)).^2));

fprintf("\nPX4 landing target Kalman filter Simulink demo\n");
fprintf("Model: %s\n", model_file);
fprintf("Sample time: %.3f s, duration: %.1f s\n", Ts, t_end);
fprintf("Measurement variance R = LTEST_MEAS_UNC * dist_z^2 = %.4f m^2\n", meas_unc);
fprintf("Accepted updates: %d\n", accepted_count);
fprintf("Rejected updates: %d\n", rejected_count);
fprintf("Position RMSE: %.4f m\n", rmse_pos);
fprintf("Velocity RMSE: %.4f m/s\n\n", rmse_vel);

open_system(model_name);

%% Local helper
function set_matlab_function_block_code(block_path, code_lines)
    rt = sfroot;
    chart = rt.find("-isa", "Stateflow.EMChart", "Path", char(block_path));

    if isempty(chart)
        error("Could not find MATLAB Function block chart for %s", block_path);
    end

    chart.Script = strjoin(code_lines, newline);
end
