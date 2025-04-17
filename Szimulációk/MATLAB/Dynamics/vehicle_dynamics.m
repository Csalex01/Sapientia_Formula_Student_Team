%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Full Vehicle Dynamics Simulation

    Goal:
    Simulate a simplified 2D vehicle model for a Formula Student car,
    capturing longitudinal, lateral, and yaw dynamics, weight transfer,
    and tire force limitations using a friction ellipse.

    Main Features:
    - Acceleration and braking behavior
    - Steering input causing lateral forces and yaw
    - Load transfer affecting tire grip
    - Tire saturation modeled via a circular friction ellipse
    - Animated output for visualization
%}

clear; clc; close all;

%% Vehicle Parameters

m = 280 * 2;                    % Total mass [kg]
g = 9.81;                       % Gravity [m/s²]
L = 1.6;                        % Wheelbase [m]
a = 0.9;                        % Distance from CG to front axle [m]
b = L - a;                      % Distance from CG to rear axle [m]
h = 0.35;                       % Height of CG [m]
mu = 1.5;                       % Friction coefficient
Iz = 230;                       % Yaw inertia [kg*m^2]
track = 1.2;                    % Track width [m]

% Tire stiffness (linear tire model until friction limit)
Cf = 80000;                     % Front cornering stiffness [N/rad]
Cr = 80000;                     % Rear cornering stiffness [N/rad]

%% Simulation Settings

dt = 0.01;                      % Timestep [s]
T_end = 10;                     % Total simulation time [s]
t = 0:dt:T_end;                 % Time vector

% Input profiles
steering_angle = 0.05 * sin(2 * pi * t / max(t));   % Steering angle input [rad]
Fx_total = zeros(size(t));                          % Longitudinal force input

% Acceleration for 4 sec, brake for 3 sec, coast for rest
Fx_total(t <= 4) = 1000;
Fx_total(t > 4 & t <= 7) = -1500;

%% State Initialization

x = 0;                          % Position x [m]
y = 0;                          % Position y [m]
yaw = 0;                        % Heading angle [rad]

vx = 0.1;                       % Longitudinal velocity [m/s]
vy = 0;                         % Lateral velocity [m/s]
r = 0;                          % Yaw rate [rad/s]

% For animation
trajectory = zeros(length(t), 2);

%% Main Simulation Loop

for i = 1:length(t)
    
    delta = steering_angle(i);  % Current steering input
    Fx = Fx_total(i);           % Total longitudinal force input

    % Weight transfer front/rear
    a_x = Fx / m;
    dFz = m * a_x * h / L;

    Fz_front = m * g * b / L - dFz;
    Fz_rear  = m * g * a / L + dFz;

    % Slip angles
    if abs(vx) < 0.5
        alpha_f = 0;
        alpha_r = 0;
    else
        alpha_f = atan2(vy + a*r, vx) - delta;
        alpha_r = atan2(vy - b*r, vx);
    end

    % Lateral tire forces (before friction limit)
    Fy_f = -Cf * alpha_f;
    Fy_r = -Cr * alpha_r;

    % Friction ellipse per axle
    Fx_f = 0.5 * Fx;  % Evenly split front/rear
    Fx_r = 0.5 * Fx;

    Fmax_f = mu * Fz_front;
    Fmax_r = mu * Fz_rear;

    % Clamp tire forces inside friction ellipse
    Fy_f = min(max(Fy_f, -sqrt(Fmax_f^2 - Fx_f^2)), sqrt(Fmax_f^2 - Fx_f^2));
    Fy_r = min(max(Fy_r, -sqrt(Fmax_r^2 - Fx_r^2)), sqrt(Fmax_r^2 - Fx_r^2));

    % Forces in vehicle coordinates
    F_total_x = Fx_f + Fx_r;
    F_total_y = Fy_f + Fy_r;

    % Dynamics update
    ax = F_total_x / m + vy * r;
    ay = F_total_y / m - vx * r;
    yaw_dot = (a * Fy_f - b * Fy_r) / Iz;

    % Euler integration
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    r = r + yaw_dot * dt;

    yaw = yaw + r * dt;
    x = x + (vx * cos(yaw) - vy * sin(yaw)) * dt;
    y = y + (vx * sin(yaw) + vy * cos(yaw)) * dt;

    trajectory(i, :) = [x, y];
end

%% Plot Results

figure("Name", "Vehicle Trajectory", "Color", "w");
plot(trajectory(:,1), trajectory(:,2), 'b', 'LineWidth', 2);
title('Vehicle Path'); xlabel('x [m]'); ylabel('y [m]');
grid on; axis equal;

