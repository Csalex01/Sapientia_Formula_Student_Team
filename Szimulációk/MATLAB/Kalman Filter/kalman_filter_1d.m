%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Kalman Filter for Estimating the Weight of a Gold Bar

    Goal: Estimate the true (constant) weight of a gold bar using noisy
    measurements and a 1D Kalman Filter.

    Based on: "Kalman Filters from the Ground Up" by Alex Becker

    Assumption:
    - The system is static (true weight does not change over time).
    - Measurements are corrupted by Gaussian noise.
%}

clear;
clc;
close all;

%% -------------------- System Setup --------------------

n = 50;                     % Number of measurements
true_weight = 1000;         % True weight of the gold bar [g]
measurement_noise_std = 10; % Standard deviation of measurement noise

% Simulate noisy measurements
z = true_weight + measurement_noise_std * randn(1, n);

%% ---------------- Kalman Filter Initialization ----------------

% Initial estimate of the weight (could be off)
x_hat = zeros(1, n);
x_hat(1) = z(1);            % Start with first measurement

% Initial uncertainty in estimate
P = 1000;                   % Large initial variance
P_store = zeros(1, n);      % To store uncertainty over time
P_store(1) = P;

% Measurement and process noise
R = measurement_noise_std^2;  % Measurement noise variance
Q = 1e-5;                      % Small process noise

%% ---------------- Kalman Filter Loop ----------------

for k = 2:n
    % Prediction
    x_pred = x_hat(k-1);
    P_pred = P + Q;

    % Update
    K = P_pred / (P_pred + R);
    x_hat(k) = x_pred + K * (z(k) - x_pred);
    P = (1 - K) * P_pred;

    % Store current uncertainty
    P_store(k) = P;
end

%% -------------------- Plotting Results --------------------

fig = figure("Name", "Kalman Filter - Gold Bar Weight Estimation");
fig.Color = [1 1 1];

% Subplot 1: Estimate vs. True Value and Measurements
subplot(2,1,1);

hold on; 

grid on; 
grid minor;

plot(1:n, z, 'k--o', 'DisplayName', 'Noisy Measurements', 'LineWidth', 1.5);
plot(1:n, x_hat, 'r-', 'DisplayName', 'Kalman Estimate', 'LineWidth', 2);
yline(true_weight, 'b--', 'DisplayName', 'True Weight (1000 g)', 'LineWidth', 2);

xlabel('Measurement Index');
ylabel('Weight [g]');

title('Kalman Filter Estimation of Gold Bar Weight');

legend('Location', 'best');

% Subplot 2: Estimate Uncertainty Over Time
subplot(2,1,2);

hold on; 

grid on;
grid minor;

plot(1:n, P_store, 'm-', 'LineWidth', 2);

title('Estimate Uncertainty (Variance) Over Time');
xlabel('Measurement Index');
ylabel('P (Variance of Estimate)');

legend('Estimate Uncertainty');
