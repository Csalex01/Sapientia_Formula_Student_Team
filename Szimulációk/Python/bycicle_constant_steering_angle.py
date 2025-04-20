# Sapientia Formula Student Team
# ------------------------------
#
# Title: Bicycle Model for Constant Steering
#
# Goal: Simulate a simplified 2D vehicle trajectory using a kinematic
# bicycle model with constant steering input.
#
# Applied formulas:
#     x(t+1)     = x(t) + v * cos(theta) * dt
#     y(t+1)     = y(t) + v * sin(theta) * dt
#     theta(t+1) = theta(t) + (v / L) * tan(delta) * dt
#
# L           Wheelbase [m]
# delta       Steering angle [rad]
# theta       Heading angle [rad]
# v           Constant velocity [m/s]
# x, y        Vehicle position [m]

import numpy as np
import matplotlib.pyplot as plt

# Parameters
L = 2.5                 # Wheelbase [m]
v = 20                  # Constant velocity [m/s]
delta = np.deg2rad(5)   # Steering angle [rad]
dt = 0.01               # Time step [s]
T = 5                   # Total time [s]
N = int(T / dt)         # Number of steps

# Initialization
x = np.zeros(N)         # X position [m]
y = np.zeros(N)         # Y position [m]
theta = np.zeros(N)     # Heading angle [rad]

# Simulation loop (bicycle model)
for i in range(1, N):
    x[i] = x[i-1] + v * np.cos(theta[i-1]) * dt
    y[i] = y[i-1] + v * np.sin(theta[i-1]) * dt
    theta[i] = theta[i-1] + (v / L) * np.tan(delta) * dt

# Plot trajectory
plt.figure("Bicycle Model Simulation", facecolor='w')
plt.plot(x, y, 'r-', linewidth=2.0)

plt.grid(True, which='both')
plt.axis('equal')

plt.title("Vehicle Path using Bicycle Model")
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.show()
