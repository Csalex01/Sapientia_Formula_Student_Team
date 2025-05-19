%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Simulation of Second Order System

    Goal:
        To simulate the motion of a mass under a time-varying input force.
        The position is measured with noise, and from this noisy signal we
        estimate velocity, acceleration, and the acting force using simple
        numerical differentiation.

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

noise_std = 0.1;

x_measured = zeros(1, length(t));
v_estimated = zeros(1, length(t));
a_estimated = zeros(1, length(t));
F_estimated = zeros(1, length(t));

for k = 2 : length(t)

    % System simulation

    a(k) = F(k - 1) / m;
    v(k) = v(k - 1) + a(k) * dt;
    x(k) = x(k - 1) + v(k - 1) * dt;

    % Measurement and Estimation

    x_measured(k) = x(1, k) + noise_std * randn();
    v_estimated(k) = (x_measured(k) - x_measured(k - 1)) / dt;

    if k > 2
        a_estimated(k) = (v_estimated(k) - v_estimated(k - 1))  /dt;
        F_estimated(k) = a_estimated(k) / m;
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

plot(t, x_measured, "r-", "LineWidth", 1.0);

title("INPUT: Measured Position [m]");

subplot(4, 2, 2);

grid on;
grid minor;

hold on;

plot(t, x, "b-", "LineWidth", 1.0);

title("Ideal Position [m]");

% Velocity

subplot(4, 2, 3);

grid on;
grid minor;

hold on;

plot(t, v_estimated, "b-", "LineWidth", 1.0);

title("Estimated Velocity [m/s]");

subplot(4, 2, 4);

grid on;
grid minor;

hold on;

plot(t, v, "b-", "LineWidth", 1.0);

title("Ideal Velocity [m/s]");

% Acceleration

subplot(4, 2, 5);

grid on;
grid minor;

hold on;

plot(t, a_estimated, "b-", "LineWidth", 1.0);

title("Estimated Acceleration [m/s^2]");

subplot(4, 2, 6);

grid on;
grid minor;

hold on;

plot(t, a, "b-", "LineWidth", 1.0);

title("Ideal Acceleration [m/s^2]");

% Force

subplot(4, 2, 7);

grid on;
grid minor;

hold on;

plot(t, F_estimated, "b-", "LineWidth", 1.0);

title("Estimated Force [N]");
xlabel("Time [s]");

subplot(4, 2, 8);

grid on;
grid minor;

hold on;

plot(t, F, "r-", "LineWidth", 1.0);

title("INPUT: Force [N]");
xlabel("Time [s]");