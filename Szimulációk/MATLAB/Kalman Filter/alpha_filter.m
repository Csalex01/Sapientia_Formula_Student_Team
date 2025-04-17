%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Alpha-Filter

    Goal: Estimate the weight of a gold bar with noisy measurements using
    an Alpha-Filter.

    Assumptions:
    - The system is static: the true weight of the gold bar does not change over time.
    - True weight: x = 1000 g
%}

clear;
clc;
close all;

%% -------------------- System Setup --------------------

n = 50;                 % Number of measurements/samples
x = 1000;               % True weight of the gold bar [grams]
z = x + randn(n, 1);    % Simulated noisy measurements (white Gaussian noise)

x_0 = 1000;             % Initial guess for the weight estimate [grams]

%% ---------------- Alpha-Filter Initialization ----------------

x_hat = zeros(1, n);    % Preallocate array for filtered estimates
x_hat(1) = z(1);        % Initialize estimate with the first measurement

%% ---------------- Alpha-Filter Loop ----------------

for k = 2 : n
    alpha = 1 / k;  % Decreasing alpha: gives less weight to newer measurements over time

    % Recursive Alpha-Filter update:
    %   New estimate = previous estimate + alpha * (measurement error)
    x_hat(k) = x_hat(k - 1) + alpha * (z(k) - x_hat(k - 1));
end

%% -------------------- Plotting Results --------------------

fig = figure("Name", "Alpha-Filter");
fig.Color = [1 1 1];    % Set figure background to white

hold on;
grid on;
grid minor;

% Plot noisy measurements
plot(1:n, z, "k--", "LineWidth", 2.0, "DisplayName", "Measurement");

% Plot Alpha-Filter estimates
plot(1:n, x_hat, "r--", "LineWidth", 2.0, "DisplayName", "Estimate");

% Plot true weight
yline(x, "b--", "LineWidth", 2.0, "DisplayName", "True Weight (1000 g)");

% Annotate the figure
title("Alpha-Filter Estimation of Gold Bar Weight");
xlabel("Measurement Index");
ylabel("Weight [g]");
legend("Location", "NorthEast");
