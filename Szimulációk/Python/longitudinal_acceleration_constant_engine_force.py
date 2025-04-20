# Sapientia Formula Student Team
# ------------------------------
#
# Title: Longitudinal Acceleration Simulation (1-dimension)
#
# Goal: Simulate straight-line acceleration considering engine force,
# drag, and rolling resistance.
#
# Applied formula:    F_net = m * a
#                 <=> F_engine - F_drag - F_roll = m * a
#                 <=> a = (F_engine - F_drag - F_roll) / m
#
# F_engine                            Engine traction force (const.)
# F_drag = 0.5 * C_d * rho * A * v^2  Aerodynamic drag
# F_roll = C_r * m * g                Rolling resistance
# a                                   Resulting acceleration
# v(t) = v(t - 1) + a * dt            Euler integration to update speed

import numpy as np
import matplotlib.pyplot as plt

# Parameters
mass = 700              # Mass of the vehicle [kg]
F_engine = 4000         # Constant engine traction force [N]
C_d = 0.8               # Aerodynamic drag coefficient [-]
A = 1.2                 # Frontal area [m^2]
rho = 1.225             # Air density [kg/m^3]
C_r = 0.015             # Rolling resistance coefficient [-]
g = 9.81                # Gravity [m/s^2]

# Simulation setup
dt = 0.01
t = np.arange(0, 100 + dt, dt)

# Preallocate arrays
F_drag_hist = np.zeros_like(t)
F_roll_hist = np.zeros_like(t)
F_net_hist = np.zeros_like(t)
a_hist = np.zeros_like(t)
v = np.zeros_like(t)

# Euler integration
for i in range(1, len(t)):
    F_drag = 0.5 * rho * C_d * A * v[i-1]**2
    F_roll = C_r * mass * g
    F_net = F_engine - F_drag - F_roll
    a = F_net / mass
    v[i] = v[i-1] + a * dt

    F_drag_hist[i] = F_drag
    F_roll_hist[i] = F_roll
    F_net_hist[i] = F_net
    a_hist[i] = a

# Plot velocity
plt.figure("Vehicle Speed in the Longitudinal Axis", facecolor='w')
plt.plot(t, v, 'r-', linewidth=2.0)
plt.grid(True, which='both')
plt.title("Vehicle Speed")
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")

# Plot acceleration
plt.figure("Vehicle Acceleration", facecolor='w')
plt.plot(t, a_hist, 'r-', linewidth=2.0)
plt.grid(True, which='both')
plt.title("Vehicle Acceleration in the Longitudinal Axis")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s²]")

# Plot forces
plt.figure("Forces", facecolor='w')
plt.plot(t, F_net_hist, 'r-', linewidth=2.0, label="F_net")
plt.plot(t, F_drag_hist, 'g-', linewidth=2.0, label="F_drag")
plt.plot(t, F_roll_hist, 'b-', linewidth=2.0, label="F_roll")
plt.grid(True, which='both')
plt.title("Applied Forces on Vehicle in the Longitudinal Axis")
plt.xlabel("Time [s]")
plt.ylabel("Force magnitude [N]")
plt.legend(loc="upper right")

plt.show()
