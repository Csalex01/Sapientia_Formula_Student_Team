# Sapientia Formula Student Team
# ------------------------------
#
# Title: Traction Ellipse Visualization
#
# Goal: Plot the traction ellipse (friction circle) that illustrates the
# trade-off between lateral and longitudinal tire forces.
#
# (Fx / Fmax)^2 + (Fy / Fmax)^2 <= 1
# Fmax = mu * Fz

import numpy as np
import matplotlib.pyplot as plt

# Parameters
mu = 1.7              # Friction coefficient (e.g., soft slick)
Fz = 2000             # Normal load [N]
Fmax = mu * Fz        # Max tire force

# Ellipse (circle) data
theta = np.linspace(0, 2 * np.pi, 360)
Fx = Fmax * np.cos(theta)
Fy = Fmax * np.sin(theta)

# Plotting
fig, ax = plt.subplots(figsize=(8, 6))
fig.patch.set_facecolor('white')

ax.plot(Fx, Fy, 'k-', linewidth=2, label="Traction Limit")
ax.axhline(0, color='r', linestyle='--', linewidth=2, label="Max Acceleration/Brake")
ax.axvline(0, color='b', linestyle='--', linewidth=2, label="Max Cornering")

# Axis settings
ax.set_title("Traction Ellipse", fontsize=14)
ax.set_xlabel("Longitudinal Force: $F_x$ [N]", fontsize=12)
ax.set_ylabel("Lateral Force: $F_y$ [N]", fontsize=12)
ax.grid(True, which='both')
ax.set_aspect('equal', 'box')
ax.legend(loc='upper right')

# Annotate directions
ax.text(-0.9 * Fmax, 0, 'Braking', color='red', ha='right', va='bottom', fontsize=12)
ax.text(0.9 * Fmax, 0, 'Acceleration', color='red', ha='left', va='bottom', fontsize=12)
ax.text(0, 0.9 * Fmax, 'Cornering Left', color='blue', ha='center', va='bottom', fontsize=12)
ax.text(0, -0.9 * Fmax, 'Cornering Right', color='blue', ha='center', va='top', fontsize=12)

# Coordinate axis arrows
arrow_origin = (-1.1 * Fmax, 0.9 * Fmax)
arrow_len = 0.15 * Fmax
ax.arrow(*arrow_origin, arrow_len, 0, head_width=100, head_length=100, fc='k', ec='k', linewidth=1.5)
ax.arrow(*arrow_origin, 0, arrow_len, head_width=100, head_length=100, fc='k', ec='k', linewidth=1.5)

ax.text(arrow_origin[0] + arrow_len + 50, arrow_origin[1], 'X (Fx)', fontsize=10, ha='left', va='center')
ax.text(arrow_origin[0], arrow_origin[1] + arrow_len + 50, 'Y (Fy)', fontsize=10, ha='center', va='bottom')

plt.tight_layout()
plt.show()
