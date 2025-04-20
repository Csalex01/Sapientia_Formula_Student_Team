# Sapientia Formula Student Team
# ------------------------------
#
# Title: Traction Ellipse Animation (4 Wheels) with Weight Transfer
#
# Goal: Animate how the tire forces (longitudinal Fx and lateral Fy)
# evolve on a Formula Student car during acceleration and braking,
# showing how weight transfer alters the traction limits.
#
# Key Concepts:
# - Each wheel has a traction ellipse based on current normal force (Fz)
# - Longitudinal acceleration shifts weight forward or backward
# - Normal forces (Fz) dynamically affect the max possible tire force (Fmax)
# - The tire forces are displayed as arrows inside each ellipse
#
# Friction ellipse equation:
#     (Fx / Fmax)^2 + (Fy / Fmax)^2 <= 1
#
# Weight transfer:
#     ΔFz = (m * a_x * h) / L
#
# Where:
#     m     = axle mass [kg]
#     g     = gravitational acceleration [m/s²]
#     mu    = friction coefficient
#     h     = center of gravity height [m]
#     L     = wheelbase [m]
#     a_x   = longitudinal acceleration [m/s²]
#     Fz    = normal (vertical) load on axle [N]
#     Fmax  = mu * Fz (max grip)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =============================
# 1. Model and Simulation Parameters
# =============================

m = 280                  # Mass per axle [kg], total vehicle = 560 kg
g = 9.81                # Gravity [m/s²]
mu = 1.5                # Friction coefficient (rubber on asphalt)
L = 1.6                 # Wheelbase of vehicle [m]
h = 0.35                # Height of center of gravity [m]

Fz_static = m * g        # Static (unmoving) normal load per axle [N]

t = np.linspace(0, 10, 200)  # Simulation time vector [s]

# Simulate longitudinal force: first accelerate, then brake
Fx_accel = np.linspace(0, 0.8 * mu * Fz_static, len(t)//2)   # Accelerating phase
Fx_brake = np.linspace(0.8 * mu * Fz_static, -0.8 * mu * Fz_static, len(t)//2) # Braking phase
Fx_total = np.concatenate([Fx_accel, Fx_brake])

# Simulate lateral force with sinusoidal variation
Fy_total = 0.7 * mu * Fz_static * np.sin(2 * np.pi * t / t[-1])  # [N]

# =============================
# 2. Create Figure and Subplots for 4 Wheels
# =============================

fig, axs = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle("Traction Ellipse with Weight Transfer")

wheel_names = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']

ellipses = []
forces = []

for i, ax in enumerate(axs.flat):
    ax.set_title(wheel_names[i])
    ax.set_xlabel("Fx [N]")
    ax.set_ylabel("Fy [N]")
    ax.set_xlim(-mu*Fz_static*1.5, mu*Fz_static*1.5)
    ax.set_ylim(-mu*Fz_static*1.5, mu*Fz_static*1.5)
    ax.grid(True)
    ellipse, = ax.plot([], [], 'k-')
    force = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='r')
    ellipses.append(ellipse)
    forces.append(force)

# =============================
# 3. Animation Function
# =============================

def animate(i):
    Fx = Fx_total[i]
    Fy = Fy_total[i]
    a_x = Fx / (2 * m)  # Fx is per axle
    dFz = (m * a_x * h) / L

    Fz_front = max(0, Fz_static - dFz)
    Fz_rear = max(0, Fz_static + dFz)

    Fmax_front = mu * Fz_front
    Fmax_rear = mu * Fz_rear

    theta = np.linspace(0, 2*np.pi, 200)
    Fx_front = Fmax_front * np.cos(theta)
    Fy_front = Fmax_front * np.sin(theta)
    Fx_rear = Fmax_rear * np.cos(theta)
    Fy_rear = Fmax_rear * np.sin(theta)

    for k in range(4):
        if k <= 1:
            ellipses[k].set_data(Fx_front, Fy_front)
        else:
            ellipses[k].set_data(Fx_rear, Fy_rear)
        forces[k].set_UVC(Fx, Fy)

    return ellipses + forces

ani = FuncAnimation(fig, animate, frames=len(t), interval=20, blit=True)
plt.tight_layout()
plt.show()
