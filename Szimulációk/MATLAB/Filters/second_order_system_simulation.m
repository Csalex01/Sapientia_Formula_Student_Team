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

F = 2 * sin(2 * pi * 0.1 * t);

%% State initialization
% x_1 - position
% x_2 - velocity

x = zeros(2, length(t));
x(:, 1) = [ 0 ; 0 ];

%% Simulation

for k = 2 : length(t)

    acceleration = F(k - 1) / m;
    x(2, k) = x(2, k - 1) + acceleration * dt;
    x(1, k) = x(1, k - 1) + x(2, k - 1) * dt;

end

%% Plotting

fig = figure("Color", "w");

% Position

subplot(3, 1, 1);

plot(t, x(1, :), "r-", "LineWidth", 2.0);

grid minor;

title("Position");
xlabel("Time [s]");
ylabel("Position [m]");

% Velocity

subplot(3, 1, 2);

plot(t, x(2, :), "r-", "LineWidth", 2.0);

grid minor;

title("Velocity");
xlabel("Time [s]");
ylabel("Velocity [m]");

% Input force

subplot(3, 1, 3);

plot(t, F, "r-", "LineWidth", 2.0);

grid minor;

title("Input force");
xlabel("Time [s]");
ylabel("Force [N]");