%{

    Sapientia Formula Student Team
    ------------------------------

    Title: Bicycle Model for Constant Steering

    Goal: Simulate a simplified 2D vehicle trajectory using a kinematic
    bicycle model with constant steering input.

    Applied formulas:
        x(t+1)     = x(t) + v * cos(theta) * dt
        y(t+1)     = y(t) + v * sin(theta) * dt
        theta(t+1) = theta(t) + (v / L) * tan(delta) * dt

    L           Wheelbase [m]
    delta       Steering angle [rad]
    theta       Heading angle [rad]
    v           Constant velocity [m/s]
    x, y        Vehicle position [m]

%}

clear;
clc;
close all;

%% Parameters

L = 2.5;                     % Wheelbase [m]
v = 20;                      % Constant velocity [m/s]
delta = deg2rad(5);          % Steering angle [rad]
dt = 0.01;                   % Time step [s]
T = 5;                       % Total time [s]
N = T / dt;                  % Number of steps

%% Initialization

x = zeros(1, N);             % X position [m]
y = zeros(1, N);             % Y position [m]
theta = zeros(1, N);         % Heading angle [rad]

%% Simulation loop (bicycle model)

for i = 2:N
    x(i) = x(i-1) + v * cos(theta(i-1)) * dt;
    y(i) = y(i-1) + v * sin(theta(i-1)) * dt;
    theta(i) = theta(i-1) + (v / L) * tan(delta) * dt;
end

%% Plot trajectory

figure("Name", "Bicycle Model Simulation", "Color", [1 1 1]);

plot(x, y, "r-", "LineWidth", 2.0);

grid on;
grid minor;

axis equal;

title("Vehicle Path using Bicycle Model");

xlabel("X Position [m]");
ylabel("Y Position [m]");
