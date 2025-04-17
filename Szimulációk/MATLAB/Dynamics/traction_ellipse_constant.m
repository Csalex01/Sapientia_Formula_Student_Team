%{

    Sapientia Formula Student Team
    ------------------------------

    Title: Traction Ellipse Visualization

    Goal: Plot the traction ellipse (friction circle) that illustrates the
    trade-off between lateral and longitudinal tire forces.

    Applied formula:    (Fx / Fmax)^2 + (Fy / Fmax)^2 <= 1

    Fmax   = mu * Fz    Maximum tire force
    Fx     Longitudinal force
    Fy     Lateral force

    The ellipse shows tire's friction limit.

%}

clear;
clc;
close all;

%% Parameters

mu = 1.7;              % Tire friction coefficient (soft slick tire)
Fz = 2000;             % Normal load on the tire [N]

Fmax = mu * Fz;        % Max possible tire force [N]

%% Generate ellipse data

theta = linspace(0, 2*pi, 360);    % Angle values [rad]
Fx = Fmax * cos(theta);            % Longitudinal forces
Fy = Fmax * sin(theta);            % Lateral forces

%% Plotting

fig = figure("Name", "Traction Ellipse");
fig.Color = [ 1 1 1 ];
fig.Position(1 : 2) = [ 500 350 ];
fig.Position(3 : 4) = [ 800 600 ];

hold on;

grid on;
grid minor;

axis equal;

plot(Fx, Fy, "k-", "LineWidth", 2.0, "DisplayName", "Traction Limit");
plot([-Fmax Fmax], [0 0], "r--", "LineWidth", 2.0, "DisplayName", "Max Acceleration/Brake");
plot([0 0], [-Fmax Fmax], "b--", "LineWidth", 2.0, "DisplayName", "Max Cornering");

title("Traction Ellipse");
xlabel("Longitudinal Force: F_x [N]");
ylabel("Lateral Force: F_y [N]");

arrow_origin = [-Fmax * 1.1, Fmax * 0.9];
arrow_len = Fmax * 0.15;

quiver(arrow_origin(1), arrow_origin(2), arrow_len, 0, 'MaxHeadSize', 2, 'Color', 'k', 'LineWidth', 1.5, 'AutoScale', 'off');
quiver(arrow_origin(1), arrow_origin(2), 0, arrow_len, 'MaxHeadSize', 2, 'Color', 'k', 'LineWidth', 1.5, 'AutoScale', 'off');

text(-Fmax * 0.9, 0, 'Braking', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'Color', 'red');
text(Fmax * 0.9, 0, 'Acceleration', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'Color', 'red');

text(0, Fmax * 0.9, 'Cornering Left', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'Color', 'blue');
text(0, -Fmax * 0.9, 'Cornering Right', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontSize', 12, 'Color', 'blue');

text(arrow_origin(1) + arrow_len + 50, arrow_origin(2), 'X (Fx)', 'FontSize', 10, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
text(arrow_origin(1), arrow_origin(2) + arrow_len + 50, 'Y (Fy)', 'FontSize', 10, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');