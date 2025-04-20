%{

    Sapientia Formula Student Team
    ------------------------------

    Title: Longitudinal Acceleration Simulation (1-dimension)

    Goal: Simulate straight-line acceleration considering engine force,
    drag, and rolling resistance.

    Applied formula:    F_net = m * a
                    <=> F_engine - F_drag - F_roll = m * a
                    <=> a = (F_engine - F_drag - F_roll) / m

    F_engine                            Engine traction force (const.)
    F_drag = 0.5 * C_d * rho * A * v^2  Aerodynamic drag
    F_roll = C_r * m * g                Rolling resistance
    a                                   Resulting acceleration
    v(t) = v(t - 1) + a * dt            Euler integration to update speed

%}

clear;
clc;
close all;

%% Parameters

mass = 700;             % Mass of the vehicle [kg]
F_engine = 4000;        % Constant engine traction force [N]
C_d = 0.8;              % Aerodynamic drag coefficient [-]
A = 1.2;                % Frontal area of the car [m^2]
rho = 1.225;            % Air density at sea level [kg/m^3]
C_r = 0.015;            % Rolling resistance coefficient [-]
g = 9.81;               % Acceleration due to gravity [m/s^2]

%% Simulation setup

dt = 0.01;              % Time step for simulation [s]
t = 0 : dt : 100;       % Time vector from 0 to 100 seconds

% History for variables

F_drag_hist = zeros(size(t));
F_roll_hist = zeros(size(t));
F_net_hist = zeros(size(t));
a_hist = zeros(size(t));
v = zeros(size(t));

%% Longitudinal acceleration simulation using Euler integration

for i = 2 : length(t)
    
    % Calculate aerodynamic drag force [N]
    F_drag = 0.5 * rho * C_d * A * v(i - 1)^2;
    
    % Calculate rolling resistance force [N]
    F_roll = C_r * mass * g;
    
    % Compute net force acting on the vehicle [N]
    F_net = F_engine - F_drag - F_roll;

    % Compute acceleration from net force [m/s^2]
    a = F_net / mass;

    % Update velocity using Euler integration [m/s]
    v(i) = v(i - 1) + a * dt;

    % Update history
    F_drag_hist(i) = F_drag;
    F_roll_hist(i) = F_roll;
    F_net_hist(i) = F_net;
    a_hist(i) = a;

end

%% Plotting results

% Plot velocity over time

fig = figure("Name", "Vehicle Speed in the Longitudinal Axis");
fig.Color = [ 1 1 1 ];

plot(t, v, "r-", "LineWidth", 2.0);

grid on;
grid minor;

title("Vehicle Speed");
xlabel("Time [s]");
ylabel("Speed [m/s]");

% Plot acceleration over time;

fig = figure("Name", "Vehicle Acceleration");
fig.Color = [ 1 1 1 ];

plot(t, a_hist, "r-", "LineWidth", 2.0);

grid on;
grid minor;

title("Vehicle Acceleration in the Longitudinal Axis");
xlabel("Time [s]");
ylabel("Acceleration [m/s^2]");

% Plot forces over time

fig = figure("Name", "Forces");
fig.Color = [ 1 1 1 ];

hold on;

grid on;
grid minor;

plot(t, F_net_hist, "r-", "LineWidth", 2.0, "DisplayName", "F_{net}");
plot(t, F_drag_hist, "g-", "LineWidth", 2.0, "DisplayName", "F_{drag}");
plot(t, F_roll_hist, "b-", "LineWidth", 2.0, "DisplayName", "F_{roll}");

title("Applied Forces on Vehicle in the Longitudinal Axis");
xlabel("Time [s]");
ylabel("Force magnitude [N]");

legend("Location", "NorthEast");