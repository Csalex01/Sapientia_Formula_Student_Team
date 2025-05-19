%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Simulation of Second Order System

    Goal:
        To simulate the motion of a mass under a time-varying input force.
        The position is measured with noise, and from this noisy signal we
        estimate velocity, acceleration, and the acting force using simple
        numerical differentiation and filtering.

    Used formulas:
        Newton's Second Law:

            F = m * a

        Numerical integration for velocity and position:

            v(k) = v(k-1) + a(k) * dt
            x(k) = x(k-1) + v(k-1) * dt

        Numerical differentiation for estimation:

            v_est(k) ≈ (x_meas(k) - x_meas(k-1)) / dt
            a_est(k) ≈ (v_est(k) - v_est(k-1)) / dt
            F_est(k) = m * a_est(k)

        First-order IIR low-pass filter (exponential smoothing):

            y(k) = α * x(k) + (1 - α) * y(k-1)

            where:
                α = dt / (1 / (2π * fc) + dt)
                fc - cutoff frequency [Hz]
                dt - sampling time [s]
                x(k) - current raw value
                y(k) - filtered value
%}

clear;
clc;
close all;

%% System parameters

m = 10.0;

%% Simulation parameters

dt = 0.01;
Tf = 10;
t = 0 : dt : Tf;

%% Input: Force

F = 10 * sin(2 * pi * 0.1 * t);

%% State initialization
% x_1 - position
% x_2 - velocity

x = zeros(1, length(t));
v = zeros(1, length(t));
a = zeros(1, length(t));

%% Simulation + Measurement & Estimation

noise_std = 0.5;

x_measured = zeros(1, length(t));

x_filtered = zeros(1, length(t));
v_filtered = zeros(1, length(t));
a_filtered = zeros(1, length(t));
F_filtered = zeros(1, length(t));

v_estimated = zeros(1, length(t));
a_estimated = zeros(1, length(t));
F_estimated = zeros(1, length(t));

% Low-pass filter parameters

fc = 1;                             % Cut-off frequency [Hz]
alpha = dt / (1/(2*pi*fc) + dt);    % IIR low-pass filter coefficient

% Simulation loop
for k = 2 : length(t)

    % System simulation
    a(k) = F(k - 1) / m;
    v(k) = v(k - 1) + a(k) * dt;
    x(k) = x(k - 1) + v(k - 1) * dt;

    % Noisy measurement
    x_measured(k) = x(k) + noise_std * randn();

    % Filtered position
    x_filtered(k) = alpha * x_measured(k) + (1 - alpha) * x_filtered(k - 1);

    % Velocity estimate from filtered position
    v_estimated(k) = (x_filtered(k) - x_filtered(k - 1)) / dt;

    % Filtered velocity
    v_filtered(k) = alpha * v_estimated(k) + (1 - alpha) * v_filtered(k - 1);

    % Acceleration estimate from filtered velocity
    if k > 2
        a_estimated(k) = (v_filtered(k) - v_filtered(k - 1)) / dt;

        % Filtered acceleration
        a_filtered(k) = alpha * a_estimated(k) + (1 - alpha) * a_filtered(k - 1);

        % Force estimate from filtered acceleration
        F_estimated(k) = m * a_filtered(k);

        F_filtered(k) = alpha * F_estimated(k) + (1 - alpha) * F_filtered(k - 1);
    end
end

%% Plotting

fig = figure("Color", "w");
fig.Position(3:4) = [1000 800];

% Position

subplot(4, 2, 1);

grid on;
grid minor;

hold on;

plot(t, x_measured, "g-", "LineWidth", 1.0, "DisplayName", "Measured");
plot(t, x_filtered, "b-", "LineWidth", 1.0, "DisplayName", "Filtered");
plot(t, x, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

title("INPUT: Position [m]");

legend("Location", "Best");

subplot(4, 2, 2);

plot(t, x, "r-", "LineWidth", 1.0);

grid on;
grid minor;

title("Ideal Position [m]");

% Velocity

subplot(4, 2, 3);

grid on;
grid minor;

hold on;

plot(t, v_estimated, "g-", "LineWidth", 1.0, "DisplayName", "Estimated");
plot(t, v_filtered, "b-", "LineWidth", 1.0, "DisplayName", "Filtered");
plot(t, v, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

title("Velocity [m/s]");

legend("Location", "Best");

subplot(4, 2, 4);

plot(t, v, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

grid on;
grid minor;

title("Ideal Velocity [m/s]");

% Acceleration

subplot(4, 2, 5);

grid on;
grid minor;

hold on;

plot(t, a_estimated, "g-", "LineWidth", 1.0, "DisplayName", "Estimated");
plot(t, a_filtered, "b-", "LineWidth", 1.0, "DisplayName", "Filtered");
plot(t, a, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

title("Acceleration [m/s^2]");

legend("Location", "Best");

subplot(4, 2, 6);

plot(t, a, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

grid on;
grid minor;

title("Ideal Acceleration [m/s^2]");

% Force

subplot(4, 2, 7);

grid on;
grid minor;

hold on;

plot(t, F_estimated, "g-", "LineWidth", 1.0, "DisplayName", "Estimated");
plot(t, F_filtered, "b-", "LineWidth", 1.0, "DisplayName", "Filtered");
plot(t, F, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

title("Force [N]");

legend("Location", "Best");

subplot(4, 2, 8);

plot(t, F, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

grid on;
grid minor;

title("INPUT: Ideal Force [N]");