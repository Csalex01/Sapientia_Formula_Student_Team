%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Simulation of Second Order System (with FIR Filtering)

    Goal:
        To simulate the motion of a mass under a time-varying input force.
        The position is measured with noise, and from this noisy signal we
        estimate velocity, acceleration, and the acting force using simple
        numerical differentiation and FIR filtering.

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

        FIR low-pass filter:
            y(k) = sum(b .* x_buffer)

            where:
                b  - FIR filter coefficients
                x_buffer - sliding input window
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

%% FIR Low-pass filter parameters

fc = 1;               % Cut-off frequency [Hz]
fs = 1 / dt;          % Sampling frequency [Hz]
Wn = fc / (fs/2);     % Normalized cut-off
fir_order = 60;      % FIR filter order
b_fir = fir1(fir_order, Wn);  % LP FIR filter coefficients
buffer_len = fir_order + 1;

% Initialize buffers
x_buf = zeros(1, buffer_len);
v_buf = zeros(1, buffer_len);
a_buf = zeros(1, buffer_len);
F_buf = zeros(1, buffer_len);

% Simulation loop
for k = 2 : length(t)

    % Rendszer szimuláció
    a(k) = F(k - 1) / m;
    v(k) = v(k - 1) + a(k) * dt;
    x(k) = x(k - 1) + v(k - 1) * dt;

    % Zajos mérés
    x_measured(k) = x(k) + noise_std * randn();

    % Pozíció FIR szűrése
    x_buf = [x_measured(k), x_buf(1:end-1)];
    x_filtered(k) = sum(b_fir .* x_buf);

    % Sebesség becslés
    v_estimated(k) = (x_filtered(k) - x_filtered(k - 1)) / dt;

    % Sebesség FIR szűrése
    v_buf = [v_estimated(k), v_buf(1:end-1)];
    v_filtered(k) = sum(b_fir .* v_buf);

    if k > 2
        % Gyorsulás becslés
        a_estimated(k) = (v_filtered(k) - v_filtered(k - 1)) / dt;

        % Gyorsulás FIR szűrése
        a_buf = [a_estimated(k), a_buf(1:end-1)];
        a_filtered(k) = sum(b_fir .* a_buf);

        % Erő becslés
        F_estimated(k) = m * a_filtered(k);

        % Erő FIR szűrése
        F_buf = [F_estimated(k), F_buf(1:end-1)];
        F_filtered(k) = sum(b_fir .* F_buf);
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