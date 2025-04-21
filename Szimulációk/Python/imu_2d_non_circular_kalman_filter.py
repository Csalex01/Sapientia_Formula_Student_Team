# Sapientia Formula Student Team
# ------------------------------
#
# Title: IMU Simulator – Sinusoidal Motion & Kalman Filtering
#
# Goal: Simulate realistic IMU (Inertial Measurement Unit) data from a non-circular
# path, and apply Kalman filtering to estimate the vehicle's position, velocity,
# and acceleration in 3D.
#
# Simulated motion: Sinusoidal path in the XY plane (non-circular)
# IMU output:       Accelerometer and gyroscope readings with noise and bias
# Estimation:       Kalman filter used to reconstruct states from noisy data
#
# Key physical formulas:
#   - Velocity:           v = dx/dt (numerical derivative)
#   - Acceleration:       a = dv/dt (numerical derivative)
#   - Angular velocity:   ω_z = d(theta)/dt = d(arctan(vy/vx))/dt
#   - Kalman prediction:  x_k = A * x_{k-1} + B * u_k
#   - Kalman update:      x_k = x_k + K * (z_k - H * x_k)
#
# t               Time array [s]
# dt              Time step [s]
# r               Radius of motion [m] (used only in circular motion)
# omega           Angular velocity [rad/s]
# g               Gravity constant [m/s^2]
# accel_meas      Noisy + biased accelerometer readings [m/s^2]
# gyro_meas       Noisy + biased gyroscope readings [rad/s]

from dataclasses import dataclass, field
from typing import Optional
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

@dataclass
class IMU2D:
  
    dt: float = 0.01                             # Time step [s]
    t_end: float = 10.0                          # Total simulation time [s]
    radius: float = 1.0                          # Radius (not used for sinusoidal motion)
    omega: Optional[float] = None                # Angular velocity [rad/s] (optional)
    accel_noise_std: float = 0.05                # Accelerometer noise std dev [m/s^2]
    gyro_noise_std: float = 0.10                 # Gyroscope noise std dev [rad/s]
    accel_bias: float = 0.20                     # Constant accel bias [m/s^2]
    gyro_bias: float = 0.10                      # Constant gyro bias [rad/s]
    g: float = 9.81                              # Gravity [m/s^2]

    # Internal variables
    
    t: np.ndarray = field(init=False)
    pos: np.ndarray = field(init=False)
    vel: np.ndarray = field(init=False)
    accel_true: np.ndarray = field(init=False)
    gyro_true: np.ndarray = field(init=False)
    accel_meas: np.ndarray = field(init=False)
    gyro_meas: np.ndarray = field(init=False)

    def __post_init__(self):
      
        self.t = np.arange(0, self.t_end, self.dt)
        
        if self.omega is None:
            self.omega = 2 * np.pi / 5

    def simulate_motion(self):
      
        # Generate sinusoidal XY motion (non-circular)
        
        t = self.t
        x = t
        y = 2 * np.sin(t)
        z = np.zeros_like(t)
        self.pos = np.stack((x, y, z), axis=1)

        # Derivatives for velocity and acceleration
        
        self.vel = np.gradient(self.pos, self.dt, axis=0)
        self.accel_true = np.gradient(self.vel, self.dt, axis=0)

        # Gravity acts on Z
        
        self.accel_true[:, 2] -= self.g

        # Estimate angular velocity (Z-axis only)
        
        angular_velocity = np.zeros_like(self.pos)
        angular_velocity[:, 2] = np.gradient(np.arctan2(self.vel[:, 1], self.vel[:, 0]), self.dt)
        
        self.gyro_true = angular_velocity

    def add_noise(self, data: np.ndarray, noise_std: float, bias: float) -> np.ndarray:
      
        noise = np.random.normal(0, noise_std, size=data.shape)
        
        return data + noise + bias

    def generate_measurement(self):
      
        self.simulate_motion()
        
        self.accel_meas = self.add_noise(self.accel_true, self.accel_noise_std, self.accel_bias)
        self.gyro_meas = self.add_noise(self.gyro_true, self.gyro_noise_std, self.gyro_bias)

    def to_dataframe(self) -> pd.DataFrame:
      
        return pd.DataFrame({
            "Time": self.t,
            "Accel_X": self.accel_meas[:, 0],
            "Accel_Y": self.accel_meas[:, 1],
            "Accel_Z": self.accel_meas[:, 2],
            "Gyro_X": self.gyro_meas[:, 0],
            "Gyro_Y": self.gyro_meas[:, 1],
            "Gyro_Z": self.gyro_meas[:, 2],
        })

    def save_to_csv(self, filename: str = "imu_simulation.csv") -> None:
      
        df = self.to_dataframe()
        df.to_csv(filename, index=False)
        
        print(f"Data saved to {filename}")

    def plot(self) -> None:
      
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        
        plt.plot(self.t, self.accel_meas[:, 0], label='Accel_X', color='r')
        plt.plot(self.t, self.accel_meas[:, 1], label='Accel_Y', color='g')
        plt.plot(self.t, self.accel_meas[:, 2], label='Accel_Z', color='b')
        
        plt.title('Accelerometer Measurements')
        
        plt.xlabel('Time [s]')
        plt.ylabel('Acceleration [m/s^2]')
        
        plt.legend()

        plt.subplot(2, 1, 2)
        
        plt.plot(self.t, self.gyro_meas[:, 0], label='Gyro_X', color='r')
        plt.plot(self.t, self.gyro_meas[:, 1], label='Gyro_Y', color='g')
        plt.plot(self.t, self.gyro_meas[:, 2], label='Gyro_Z', color='b')
        
        plt.title('Gyroscope Measurements')
        
        plt.xlabel('Time [s]')
        plt.ylabel('Angular Velocity [rad/s]')
        
        plt.legend()
        
        plt.tight_layout()
        
        plt.show()

    def apply_kalman_filter(self):
      
        n = len(self.t)
        dt = self.dt
        
        A = np.array([[1, dt], [0, 1]])
        B = np.array([[0.5 * dt**2], [dt]])
        H = np.eye(2)
        Q = np.eye(2) * 1e-4
        R = np.eye(2) * self.accel_noise_std**2
        
        x_est = np.zeros((n, 2, 3))

        for axis in range(3):
          
            x = np.zeros((2, 1))
            P = np.eye(2)
            
            for i in range(n):
              
                u = self.accel_meas[i, axis]
                
                x_pred = A @ x + B * u
                P_pred = A @ P @ A.T + Q
                
                z = np.array([[self.pos[i, axis]], [self.vel[i, axis]]])
                
                S = H @ P_pred @ H.T + R
                K = P_pred @ H.T @ np.linalg.inv(S)
                y = z - H @ x_pred
                x = x_pred + K @ y
                P = (np.eye(2) - K @ H) @ P_pred
                
                x_est[i, :, axis] = x.ravel()

        self.kalman_pos = x_est[:, 0, :]
        self.kalman_vel = x_est[:, 1, :]
        self.kalman_accel = np.gradient(self.kalman_vel, dt, axis=0)

    def plot_kalman_estimates(self):
      
        labels = ['X', 'Y', 'Z']
        _, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        
        for i, label in enumerate(labels):
          
            axs[0].plot(self.t, self.pos[:, i], label=f'True Pos {label}', linewidth=1)
            axs[0].plot(self.t, self.kalman_pos[:, i], '--', label=f'Kalman Pos {label}')
            
            axs[1].plot(self.t, self.vel[:, i], label=f'True Vel {label}', linewidth=1)
            axs[1].plot(self.t, self.kalman_vel[:, i], '--', label=f'Kalman Vel {label}')
            
            axs[2].plot(self.t, self.accel_true[:, i], label=f'True Accel {label}', linewidth=1)
            axs[2].plot(self.t, self.kalman_accel[:, i], '--', label=f'Kalman Accel {label}')
            
        axs[0].set_ylabel("Position [m]")
        axs[1].set_ylabel("Velocity [m/s]")
        axs[2].set_ylabel("Acceleration [m/s²]")
        
        axs[2].set_xlabel("Time [s]")
        
        for ax in axs:
          
            ax.grid(True)
            ax.legend()
            
        plt.suptitle("Kalman Filter Estimation (3D)")
        plt.tight_layout()
        plt.show()

    def plot_xy_trajectory(self):
      
        speed = np.linalg.norm(self.kalman_vel[:, :2], axis=1)
        
        plt.figure("XY Trajectory", figsize=(10, 6))
        
        plt.plot(self.pos[:, 0], self.pos[:, 1], color='red', linewidth=2, label="True Trajectory")
        
        scatter = plt.scatter(self.kalman_pos[:, 0], self.kalman_pos[:, 1], c=speed, cmap='coolwarm', s=10, label="Estimated (Kalman)")
        cbar = plt.colorbar(scatter)
        
        cbar.set_label("Estimated Speed [m/s]")
        
        plt.title("XY Plane Trajectory: True vs. Estimated")
        
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        
        plt.axis("equal")
        
        plt.grid(True)
        
        plt.legend()
        
        plt.tight_layout()
        
        plt.show()

if __name__ == "__main__":
    sim = IMU2D()
    sim.generate_measurement()
    sim.plot()
    sim.apply_kalman_filter()
    sim.plot_kalman_estimates()
    sim.plot_xy_trajectory()
    sim.save_to_csv("imu_data.csv")