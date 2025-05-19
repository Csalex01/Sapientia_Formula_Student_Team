clear;
clc;
close all;

%% Parameters

m = 1.0;

Ts = 0.01;
Tf = 10;
t = 0 : Ts : Tf;

%% Ideal position, velocity, acceleration & force.

% Parameters
L = 5;         % Maximum value
k = 5;         % Growth rate
t0 = 5;        % Midpoint

% Time vector
t = 0:0.01:10;

% Logistic function
x_ideal = L ./ (1 + exp(-k * (t - t0)));

% x_ideal = 10 * sin(2 * pi * 0.05 * t - pi / 2);
v_ideal = diff(x_ideal) / Ts;
a_ideal = diff(v_ideal) / Ts;
F_ideal = m * a_ideal;

%% Measured position, approximated velocity, acceleration & force.

x_measured = x_ideal + 0.1 * randn(size(t));
v_estimate = diff(x_measured) / Ts;
a_estimate = diff(v_estimate) / Ts;
F_estimate = m * a_estimate;

%% Low-Pass filter implementation (Butterworth-filter)

cutoff_frequency = 5;
fs = 1 / Ts;
order = 4;

[ b, a ] = butter(order, cutoff_frequency / (fs / 2));

x_lowpass = filtfilt(b, a, x_measured);
v_lowpass = diff(x_lowpass) / Ts;
a_lowpass = diff(v_lowpass) / Ts;
F_lowpass = m * a_lowpass;

%% Kalman-Filter implementation

A = [
    1   Ts  ;
    0   1   ;
];
B = [ 0 ; 0 ];
H = [ 1 0 ];
Q = [
    0.001   0       ;
    0       0.001   ;
];
R = 0.1;

x_est = [ 0 0 ]';
P = eye(2);

x_kalman = zeros(size(x_measured));
v_kalman = zeros(size(x_measured));
a_kalman = zeros(size(x_measured));

for k = 1:length(t)

    x_pred = A * x_est;
    P_pred = A * P * A' + Q;
    
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_pred + K * (x_measured(k) - H * x_pred);
    P = (eye(2) - K * H) * P_pred;
    
    x_kalman(k) = x_est(1);
    v_kalman(k) = x_est(2);
    
    if k > 2
        a_kalman(k) = (v_kalman(k) - v_kalman(k-1)) / Ts;
    end
end

F_kalman = m * a_kalman;

%% Plotting

% Position

fig = figure( ...
    "Name", "Position", ...
    "Color", [ 1 1 1 ], ...
    "Position", [ 1000 818 1000 700 ] ...
);

grid on;
grid minor;

hold on;

plot(t, x_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t, x_measured, "r-", "LineWidth", 1.0, "DisplayName", "MEASURED");
plot(t, x_lowpass, "b-", "LineWidth", 1.0, "DisplayName", "LOWPASS");
plot(t, x_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Position");
xlabel("Time [s]");
ylabel("Position [m]");

legend("Location", "NorthWest");

% Velocity

fig = figure( ...
    "Name", "Velocity", ...
    "Color", [ 1 1 1 ], ...
    "Position", [ 1000 818 1000 700 ] ...
);

subplot(2, 1, 1);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 1), v_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t(1) : Ts : t(end - 1), v_estimate, "r-", "LineWidth", 1.0, "DisplayName", "ESTIMATE");
plot(t(1) : Ts : t(end - 1), v_lowpass, "b-", "LineWidth", 1.0, "DisplayName", "LOWPASS");
plot(t, v_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Velocity");
xlabel("Time [s]");
ylabel("Velocity [m]");

legend("Location", "NorthWest");

subplot(2, 1, 2);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 1), v_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t(1) : Ts : t(end - 1), v_lowpass, "b-", "LineWidth", 1.0, "DisplayName", "LOWPASS");
plot(t, v_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Velocity");
xlabel("Time [s]");
ylabel("Velocity [m]");

legend("Location", "NorthWest");

% Acceleration

fig = figure( ...
    "Name", "Acceleration", ...
    "Color", [ 1 1 1 ], ...
    "Position", [ 1000 818 1000 700 ] ...
);

subplot(3, 1, 1);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 2), a_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t(1) : Ts : t(end - 2), a_estimate, "r-", "LineWidth", 1.0, "DisplayName", "ESTIMATE");
plot(t(1) : Ts : t(end - 2), a_lowpass, "b-", "LineWidth", 1.0, "DisplayName", "LOWPASS");
plot(t, a_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Acceleration");
xlabel("Time [s]");
ylabel("Velocity [m]");

legend("Location", "NorthWest");

subplot(3, 1, 2);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 2), a_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t(1) : Ts : t(end - 2), a_lowpass, "b-", "LineWidth", 1.0, "DisplayName", "LOWPASS");
plot(t, a_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Velocity");
xlabel("Time [s]");
ylabel("Velocity [m]");

legend("Location", "NorthWest");

subplot(3, 1, 3);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 2), a_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t, a_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Velocity");
xlabel("Time [s]");
ylabel("Velocity [m]");

legend("Location", "NorthWest");

% Force

fig = figure( ...
    "Name", "Force", ...
    "Color", [ 1 1 1 ], ...
    "Position", [ 1000 818 1000 700 ] ...
);

subplot(3, 1, 1);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 2), F_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t(1) : Ts : t(end - 2), F_estimate, "r-", "LineWidth", 1.0, "DisplayName", "ESTIMATE");
plot(t(1) : Ts : t(end - 2), F_lowpass, "b-", "LineWidth", 1.0, "DisplayName", "LOWPASS");
plot(t, a_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Force");
xlabel("Time [s]");
ylabel("Force [N]");

legend("Location", "NorthWest");

subplot(3, 1, 2);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 2), F_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t(1) : Ts : t(end - 2), F_lowpass, "b-", "LineWidth", 1.0, "DisplayName", "LOWPASS");
plot(t, F_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Force");
xlabel("Time [s]");
ylabel("Force [N]");

legend("Location", "NorthWest");

subplot(3, 1, 3);

grid on;
grid minor;

hold on;

plot(t(1) : Ts : t(end - 2), F_ideal, "k--", "LineWidth", 2.0, "DisplayName", "IDEAL");
plot(t, F_kalman, "g-", "LineWidth", 1.0, "Displayname", "KALMAN");

title("Filter Responses for Force");
xlabel("Time [s]");
ylabel("Force [N]");

legend("Location", "NorthWest");
