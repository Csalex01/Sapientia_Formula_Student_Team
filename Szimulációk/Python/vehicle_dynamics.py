import numpy as np
import matplotlib.pyplot as plt

# Vehicle Parameters
m = 280 * 2           # Total mass [kg]
g = 9.81              # Gravity [m/s^2]
L = 1.6               # Wheelbase [m]
a = 0.9               # Distance from CG to front axle [m]
b = L - a             # Distance from CG to rear axle [m]
h = 0.35              # Height of CG [m]
mu = 1.5              # Friction coefficient
Iz = 230              # Yaw inertia [kg*m^2]
track = 1.2           # Track width [m]

Cf = 80000            # Front cornering stiffness [N/rad]
Cr = 80000            # Rear cornering stiffness [N/rad]

# Simulation Settings
dt = 0.01
T_end = 10
t = np.arange(0, T_end + dt, dt)

# Input profiles
steering_angle = 0.05 * np.sin(2 * np.pi * t / max(t))
Fx_total = np.zeros_like(t)
Fx_total[t <= 4] = 1000
Fx_total[(t > 4) & (t <= 7)] = -1500

# State Initialization
x = y = yaw = 0.0
vx = 0.1
vy = 0.0
r = 0.0

trajectory = np.zeros((len(t), 2))

# Main Simulation Loop
for i in range(len(t)):
    delta = steering_angle[i]
    Fx = Fx_total[i]

    a_x = Fx / m
    dFz = m * a_x * h / L

    Fz_front = m * g * b / L - dFz
    Fz_rear = m * g * a / L + dFz

    if abs(vx) < 0.5:
        alpha_f = alpha_r = 0.0
    else:
        alpha_f = np.arctan2(vy + a * r, vx) - delta
        alpha_r = np.arctan2(vy - b * r, vx)

    Fy_f = -Cf * alpha_f
    Fy_r = -Cr * alpha_r

    Fx_f = 0.5 * Fx
    Fx_r = 0.5 * Fx

    Fmax_f = mu * Fz_front
    Fmax_r = mu * Fz_rear

    Fy_f = np.clip(Fy_f, -np.sqrt(max(0, Fmax_f**2 - Fx_f**2)), np.sqrt(max(0, Fmax_f**2 - Fx_f**2)))
    Fy_r = np.clip(Fy_r, -np.sqrt(max(0, Fmax_r**2 - Fx_r**2)), np.sqrt(max(0, Fmax_r**2 - Fx_r**2)))

    F_total_x = Fx_f + Fx_r
    F_total_y = Fy_f + Fy_r

    ax = F_total_x / m + vy * r
    ay = F_total_y / m - vx * r
    yaw_dot = (a * Fy_f - b * Fy_r) / Iz

    vx += ax * dt
    vy += ay * dt
    r += yaw_dot * dt

    yaw += r * dt
    x += (vx * np.cos(yaw) - vy * np.sin(yaw)) * dt
    y += (vx * np.sin(yaw) + vy * np.cos(yaw)) * dt

    trajectory[i, :] = [x, y]

# Plot Results
plt.figure("Vehicle Trajectory", figsize=(8, 6))
plt.plot(trajectory[:, 0], trajectory[:, 1], 'b', linewidth=2)
plt.title('Vehicle Path')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.grid(True)
plt.show()
