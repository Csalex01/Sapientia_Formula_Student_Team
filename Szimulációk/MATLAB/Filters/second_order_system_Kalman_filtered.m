%{
    Sapientia Formula Student Team
    ------------------------------
    
    Title: Simulation of Second Order System with Kalman Filter
    
    Goal:
        To simulate the motion of a mass under a time-varying input force.
        The position is measured with noise, and from this noisy signal we
        estimate velocity, acceleration, and the acting force using a 
        Kalman filter for state estimation.
    
    Used formulas:
        Newton's Second Law:
            F = m * a

        Numerical integration for velocity and position:
            v(k) = v(k-1) + a(k) * dt
            x(k) = x(k-1) + v(k-1) * dt

        State space model:
            x(k+1) = A*x(k) + B*u(k) + w(k)
            z(k) = H*x(k) + v(k)
            
        where w(k) ~ N(0,Q) is process noise and v(k) ~ N(0,R) is measurement noise.

    Kalman filter algorithm:
        
        Prediction step:
            x̂(k|k-1) = A * x̂(k-1|k-1) + B * u(k-1)
            P(k|k-1) = A * P(k-1|k-1) * A' + Q

        Update step:
            K(k) = P(k|k-1) * H' * inv(H * P(k|k-1) * H' + R)  (Kalman gain)
            x̂(k|k) = x̂(k|k-1) + K(k) * (z(k) - H * x̂(k|k-1))
            P(k|k) = (I - K(k) * H) * P(k|k-1)

        where:
            x̂(k|k-1) - predicted state estimate
            x̂(k|k)   - updated state estimate
            P(k|k-1)  - predicted estimate covariance
            P(k|k)    - updated estimate covariance
            K(k)      - Kalman gain
            z(k)      - measurement at time k
            u(k)      - control input (force)
            I         - identity matrix
%}


clear;
clc;
close all;

%% System parameters

m = 10.0;  % Mass [kg]

%% Simulation parameters

dt = 0.01;           % Sampling time [s]
Tf = 10;             % Total simulation time [s]
t = 0 : dt : Tf;     % Time vector

%% Input: Force

F = 10 * sin(2 * pi * 0.1 * t);  % Time-varying force [N]

%% State initialization

x = zeros(1, length(t));  % Position [m]
v = zeros(1, length(t));  % Velocity [m/s]
a = zeros(1, length(t));  % Acceleration [m/s^2]

%% Measurement noise

noise_std = 0.5;  % Standard deviation of measurement noise

x_measured = zeros(1, length(t));  % Measured position [m]

%% Kalman filter initialization

% State vector: [position; velocity]
% State-space model:
%   x(k+1) = A*x(k) + B*u(k) + w(k)
%   z(k) = H*x(k) + v(k)

A = [1 dt; 0 1];
B = [0; dt/m];
H = [1 0];

Q = [1e-4 0; 0 1e-4];  % Process noise covariance
R = noise_std^2;       % Measurement noise covariance

x_est = zeros(2, length(t));  % Estimated states

P = eye(2);                   % Initial estimation error covariance
P_hist = zeros(1, length(t));
P_hist(1) = trace(P);

%% Simulation loop

for k = 2:length(t)

    % True system simulation
    a(k) = F(k - 1) / m;
    v(k) = v(k - 1) + a(k) * dt;
    x(k) = x(k - 1) + v(k - 1) * dt;
    
    % Noisy measurement
    x_measured(k) = x(k) + noise_std * randn();
    
    % Prediction
    x_pred = A * x_est(:, k - 1) + B * F(k - 1);
    P_pred = A * P * A' + Q;
    
    % Update
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est(:, k) = x_pred + K * (x_measured(k) - H * x_pred);
    P = (eye(2) - K * H) * P_pred;
    P_hist(k) = trace(P);
    
end

%% Estimated quantities

x_kalman = x_est(1, :);                     % Estimated position [m]
v_kalman = x_est(2, :);                     % Estimated velocity [m/s]
a_kalman = [0 diff(v_kalman) / dt];        % Estimated acceleration [m/s^2]
F_kalman = m * a_kalman;                    % Estimated force [N]

%% Plotting

fig = figure("Color", "w");
fig.Position(3:4) = [1000 800];

% Position

subplot(4, 2, 1);

grid on;
grid minor;

hold on;

plot(t, x_measured, "g-", "LineWidth", 1.0, "DisplayName", "Measured");
plot(t, x_kalman, "b-", "LineWidth", 1.0, "DisplayName", "Estimated");
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

plot(t, v_kalman, "b-", "LineWidth", 1.0, "DisplayName", "Estimated");
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

plot(t, a_kalman, "g-", "LineWidth", 1.0, "DisplayName", "Estimated");
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

plot(t, F_kalman, "g-", "LineWidth", 1.0, "DisplayName", "Estimated");
plot(t, F, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

title("Force [N]");

legend("Location", "Best");

subplot(4, 2, 8);

plot(t, F, "r-", "LineWidth", 1.0, "DisplayName", "Ideal");

grid on;
grid minor;

title("INPUT: Ideal Force [N]");

% Error covariance matrix

fig = figure("Color", "w");
fig.Position(3:4) = [1000 800];

plot(t, P_hist, "r-", "LineWidth", 2.0);

grid on;
grid minor;

title("Error Covariance Matrix Trace");

xlabel("Time [s]");
ylabel("tr(P_k)");