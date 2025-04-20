# Sapientia Formula Student Team
# ------------------------------
#
# Title: Braking Distance vs. Initial Speed
#
# Goal: Calculate and plot braking distance for different initial speeds
# based on friction and energy conservation.
#
# Applied formula:    KE = Friction work
#                     (1/2) * m * v^2 = mu * m * g * d
#                 =>  d = v^2 / (2 * mu * g)
#
# v       Initial speed [m/s]
# mu      Tire-road friction coefficient
# g       Gravitational constant
# d       Braking distance [m]

import numpy as np
import matplotlib.pyplot as plt

# Constants
g = 9.81          # Gravity [m/s^2]
mu = 1.5          # Friction coefficient (race tire)

# Initial speeds to evaluate [m/s]
v0 = np.array([10, 20, 30, 40])        # Speed values [m/s]
d = v0**2 / (2 * mu * g)              # Braking distances [m]

# Plotting
plt.figure("Braking Distance Simulation", facecolor='w')
plt.plot(v0 * 3.6, d, 'ro-', linewidth=2, markersize=6)

plt.grid(True, which='both')
plt.title("Braking Distance vs. Speed")
plt.xlabel("Initial Speed [km/h]")
plt.ylabel("Braking Distance [m]")
plt.show()
