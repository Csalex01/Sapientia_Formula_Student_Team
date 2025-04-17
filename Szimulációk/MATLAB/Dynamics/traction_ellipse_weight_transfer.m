%{
    Sapientia Formula Student Team
    ------------------------------

    Title: Traction Ellipse Animation (4 Wheels) with Weight Transfer

    Goal: Animate how the tire forces (longitudinal Fx and lateral Fy)
    evolve on a Formula Student car during acceleration and braking,
    showing how weight transfer alters the traction limits.

    Key Concepts:
    - Each wheel has a traction ellipse based on current normal force (Fz)
    - Longitudinal acceleration shifts weight forward or backward
    - Normal forces (Fz) dynamically affect the max possible tire force (Fmax)
    - The tire forces are displayed as arrows inside each ellipse

    Friction ellipse equation:
        (Fx / Fmax)^2 + (Fy / Fmax)^2 <= 1

    Weight transfer:
        ΔFz = (m * a_x * h) / L

    Where:
        m     = axle mass [kg]
        g     = gravitational acceleration [m/s²]
        mu    = friction coefficient
        h     = center of gravity height [m]
        L     = wheelbase [m]
        a_x   = longitudinal acceleration [m/s²]
        Fz    = normal (vertical) load on axle [N]
        Fmax  = mu * Fz (max grip)
%}

clear; clc; close all;  % Reset environment

%% =============================
% 1. Model and Simulation Parameters
% =============================

m = 280;                  % Mass per axle [kg], total vehicle = 560 kg
g = 9.81;                 % Gravity [m/s²]
mu = 1.5;                 % Friction coefficient (rubber on asphalt)
L = 1.6;                  % Wheelbase of vehicle [m]
h = 0.35;                 % Height of center of gravity [m]

Fz_static = m * g;        % Static (unmoving) normal load per axle [N]

t = linspace(0, 10, 200); % Simulation time vector [s]

% Simulate longitudinal force: first accelerate, then brake
Fx_accel = linspace(0, 0.8 * mu * Fz_static, length(t)/2);   % Accelerating phase
Fx_brake = linspace(0.8 * mu * Fz_static, -0.8 * mu * Fz_static, length(t)/2); % Braking phase

Fx_total = [Fx_accel Fx_brake];  % Full Fx vector over time

% Simulate lateral force with sinusoidal variation (as if turning left-right)
Fy_total = 0.7 * mu * Fz_static * sin(2 * pi * t / max(t));  % [N]

%% =============================
% 2. Create Figure and Subplots for 4 Wheels
% =============================

fig = figure("Name", "Traction Ellipse with Weight Transfer");
fig.Color = [ 1 1 1 ];  % White background

tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

wheel_names = {'Front Left', 'Front Right', 'Rear Left', 'Rear Right'};  % Titles

% Preallocate ellipse and force point handles for animation
for k = 1 : 4

    nexttile;  % Move to the next subplot

    hold on;

    % Placeholder for traction ellipse (updated in animation)
    h_ellipse{k} = plot(nan, nan, 'k-');

    % Instead of a red dot, use quiver to show the force as an arrow
    h_force(k) = quiver(0, 0, 0, 0, 'LineWidth', 2, 'MaxHeadSize', 1, 'Color', 'r');

    title(wheel_names{k});
    xlabel('Fx [N]'); ylabel('Fy [N]');

    axis equal;  % Equal scaling for both axes
    axis([-mu*Fz_static * 1.5 mu*Fz_static * 1.5 -mu*Fz_static  * 1.5 mu*Fz_static  * 1.5]);  % Set plot limits

    grid on;

end

%% =============================
% 3. Animation Loop
% =============================

for i = 1:length(t)

    % Retrieve current longitudinal and lateral tire forces
    Fx = Fx_total(i);
    Fy = Fy_total(i);
    
    % Estimate total vehicle acceleration (simplified)
    a_x = Fx / (2 * m);  % Divide by 2*mass because Fx is per axle

    % ---------------------------
    % Weight transfer calculation
    % ---------------------------
    % When accelerating (a_x > 0), rear axle gains load
    % When braking (a_x < 0), front axle gains load
    dFz = (m * a_x * h) / L;

    % Dynamic normal loads for front and rear axles
    Fz_front = Fz_static - dFz;  % Less weight on front when accelerating
    Fz_rear = Fz_static + dFz;   % More weight on rear when accelerating

    % Ensure vertical loads are not negative
    Fz_front = max(0, Fz_front);
    Fz_rear = max(0, Fz_rear);

    % Max friction forces based on current normal load
    Fmax_front = mu * Fz_front;
    Fmax_rear = mu * Fz_rear;

    % ---------------------------
    % Generate friction ellipses
    % ---------------------------
    theta = linspace(0, 2*pi, 200);  % Circle parametric angle

    % Use circular friction envelope: Fx^2 + Fy^2 <= Fmax^2
    Fx_front = Fmax_front * cos(theta);
    Fy_front = Fmax_front * sin(theta);
    Fx_rear = Fmax_rear * cos(theta);
    Fy_rear = Fmax_rear * sin(theta);

    % ---------------------------
    % Update each wheel subplot
    % ---------------------------
    for k = 1:4
        if k <= 2  % Front axle (wheels 1 and 2)
            set(h_ellipse{k}, 'XData', Fx_front, 'YData', Fy_front);  % Draw ellipse
            set(h_force(k), 'UData', Fx, 'VData', Fy);                % Update arrow (force)
        else        % Rear axle (wheels 3 and 4)
            set(h_ellipse{k}, 'XData', Fx_rear, 'YData', Fy_rear);
            set(h_force(k), 'UData', Fx, 'VData', Fy);                % Update arrow (force)
        end
    end

    pause(0.02);  % Frame delay for animation

end