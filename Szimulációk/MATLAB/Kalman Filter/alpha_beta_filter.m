%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Alpha-Beta Filter (Constant Velocity)

    Goal: Estimate the position and velocity of a moving object using
    noisy position measurements and an Alpha-Beta filter.

    Inspired by: "Kalman Filters from the Ground Up" by Alex Becker

    Assumptions:
    - Object moves with constant velocity.
    - Measurements are noisy observations of position only.
%}

clear;
clc;
close all;

%% -------------------- System Setup --------------------

n = 100;                % Number of measurements
dt = 1;                 % Time step [s]
true_velocity = 5;      % True constant velocity [units/s]
true_initial_position = 30000;

% Simulate true positions
x_true = true_initial_position + (0:n-1) * true_velocity;

% Simulate noisy measurements (position only)
measurement_noise_std = 5;
z = x_true + measurement_noise_std * randn(1, n);

%% ---------------- Alpha-Beta Filter Setup ----------------

alpha = 0.85;           % Alpha gain (position correction)
beta = 0.1;           % Beta gain (velocity correction)

% Initialize estimates
x_hat = zeros(1, n);    % Estimated position
v_hat = zeros(1, n);    % Estimated velocity

x_hat(1) = z(1);        % Initial position estimate
v_hat(1) = 0;           % Initial velocity guess

%% ---------------- Alpha-Beta Filter Loop ----------------

for k = 2 : n
    % Predict step
    x_pred = x_hat(k-1) + dt * v_hat(k-1);    % Predict next position
    v_pred = v_hat(k-1);                      % Predict next velocity (constant)

    % Measurement residual
    r = z(k) - x_pred;

    % Update step
    x_hat(k) = x_pred + alpha * r;
    v_hat(k) = v_pred + (beta / dt) * r;
end

%% -------------------- Plotting Results --------------------

fig = figure("Name", "Alpha-Beta Filter");
fig.Color = [1 1 1];

subplot(2,1,1);

hold on;

grid on;
grid minor;

plot(1:n, z, 'k.', 'DisplayName', 'Noisy Measurements');
plot(1:n, x_true, 'b-', 'LineWidth', 2, 'DisplayName', 'True Position');
plot(1:n, x_hat, 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Position');

title('Alpha-Beta Filter - Position');
xlabel('Time Step');
ylabel('Position');

legend("Location", "NorthEast");

subplot(2,1,2);

hold on;

grid on;
grid minor;

plot(1:n, repmat(true_velocity, 1, n), 'b-', 'LineWidth', 2, 'DisplayName', 'True Velocity');
plot(1:n, v_hat, 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated Velocity');

title('Alpha-Beta Filter - Velocity');
xlabel('Time Step');
ylabel('Velocity');

legend("Location", "NorthEast");
