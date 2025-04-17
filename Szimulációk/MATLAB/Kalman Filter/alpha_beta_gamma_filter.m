%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Alpha-Beta-Gamma Filter (Accelerating Aircraft)

    Goal: Estimate position, velocity, and acceleration of an object using
    only noisy position measurements and an Alpha-Beta-Gamma filter.

    Inspired by: "Kalman Filters from the Ground Up" by Alex Becker

    Assumptions:
    - The object undergoes constant acceleration.
    - Only position is measured (with noise).
%}

clear;
clc;
close all;

%% -------------------- System Setup --------------------

n = 100;                % Number of measurements
dt = 1;                 % Time step [s]

true_acc = 0.2;         % Constant acceleration [units/s^2]
true_vel_0 = 5;         % Initial velocity [units/s]
true_pos_0 = 0;         % Initial position

% Simulate true motion
x_true = zeros(1, n);
v_true = zeros(1, n);
a_true = true_acc * ones(1, n);

x_true(1) = true_pos_0;
v_true(1) = true_vel_0;

for k = 2:n
    v_true(k) = v_true(k-1) + dt * true_acc;
    x_true(k) = x_true(k-1) + dt * v_true(k-1) + 0.5 * dt^2 * true_acc;
end

% Generate noisy position measurements
measurement_noise_std = 2;
z = x_true + measurement_noise_std * randn(1, n);

%% ---------------- Alpha-Beta-Gamma Filter Setup ----------------

alpha = 0.8;    % Position gain
beta  = 0.4;    % Velocity gain
gamma = 0.1;    % Acceleration gain

% Initialize estimates
x_hat = zeros(1, n);    % Position estimate
v_hat = zeros(1, n);    % Velocity estimate
a_hat = zeros(1, n);    % Acceleration estimate

x_hat(1) = z(1);         % Initial position estimate
v_hat(1) = 0;            % Initial velocity guess
a_hat(1) = 0;            % Initial acceleration guess

%% ---------------- Alpha-Beta-Gamma Filter Loop ----------------

for k = 2:n
    % Prediction step
    x_pred = x_hat(k-1) + dt * v_hat(k-1) + 0.5 * dt^2 * a_hat(k-1);
    v_pred = v_hat(k-1) + dt * a_hat(k-1);
    a_pred = a_hat(k-1);

    % Measurement residual (error between prediction and measurement)
    r = z(k) - x_pred;

    % Update step
    x_hat(k) = x_pred + alpha * r;
    v_hat(k) = v_pred + (beta / dt) * r;
    a_hat(k) = a_pred + (gamma / dt^2) * r;
end

%% -------------------- Plotting Results --------------------

fig = figure("Name", "Alpha-Beta-Gamma Filter");
fig.Color = [1 1 1];

subplot(3,1,1);

hold on; 

grid on;
grid minor;

plot(1:n, z, 'k.', 'DisplayName', 'Noisy Measurements');
plot(1:n, x_true, 'b-', 'LineWidth', 2, 'DisplayName', 'True Position');
plot(1:n, x_hat, 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Position');

title('Position Estimation');

ylabel('Position'); 

legend("Location", "NorthEast");

subplot(3,1,2);

hold on; 

grid on;
grid minor;

plot(1:n, v_true, 'b-', 'LineWidth', 2, 'DisplayName', 'True Velocity');
plot(1:n, v_hat, 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Velocity');

title('Velocity Estimation');

ylabel('Velocity');

legend("Location", "NorthEast");

subplot(3,1,3);

hold on; 

grid on;
grid minor;

plot(1:n, a_true, 'b-', 'LineWidth', 2, 'DisplayName', 'True Acceleration');
plot(1:n, a_hat, 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Acceleration');

title('Acceleration Estimation');

xlabel('Time Step');
ylabel('Acceleration');

legend("Location", "NorthEast");
