%{

    Sapientia Formula Student Team
    ------------------------------

    Title: Braking Distance vs. Initial Speed

    Goal: Calculate and plot braking distance for different initial speeds
    based on friction and energy conservation.

    Applied formula:    KE = Friction work
                        (1/2) * m * v^2 = mu * m * g * d
                    =>  d = v^2 / (2 * mu * g)

    v       Initial speed [m/s]
    mu      Tire-road friction coefficient
    g       Gravitational constant
    d       Braking distance [m]

%}

clear;
clc;
close all;

%% Constants

g = 9.81;              % Gravity [m/s^2]
mu = 1.5;              % Friction coefficient (race tire)

%% Initial speeds to evaluate [m/s]

v0 = [10 20 30 40];             % Speed values [m/s]
d = v0.^2 ./ (2 * mu * g);    % Braking distances [m]

%% Plotting

figure("Name", "Braking Distance Simulation", "Color", [1 1 1]);

plot(v0 * 3.6, d, 'ro-', "LineWidth", 2, "MarkerSize", 6); 

grid on;
grid minor;

title("Braking Distance vs. Speed");
xlabel("Initial Speed [km/h]");
ylabel("Braking Distance [m]");
