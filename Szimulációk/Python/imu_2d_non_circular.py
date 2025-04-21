# Sapientia Formula Student Team
# ------------------------------
#
# Title: IMU Simulator – Circular Motion
#
# Goal: Simulate IMU measurements (accelerometer & gyroscope) for a circular
# motion scenario. The simulator adds Gaussian noise and bias to approximate
# real IMU behavior.
#
# Simulated motion: Circular trajectory in the XY-plane
# IMU output:       Accelerometer and gyroscope readings over time
#
# Key concepts:
#   - Acceleration: a = -r * ω² for circular motion
#   - Angular velocity: ω = constant (Z-axis only)
#   - Gravity: added to Z-axis acceleration
#   - Noise: Gaussian with user-defined standard deviation
#   - Bias: constant offset added to each measurement
#
# t           Time array [s]
# dt          Time step [s]
# r           Radius of circular motion [m]
# omega       Angular velocity [rad/s]
# g           Gravity constant [m/s^2]
# accel_meas  Simulated accelerometer readings [m/s^2]
# gyro_meas   Simulated gyroscope readings [rad/s]

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


@dataclass
class IMU2D:
  """
  A class to simulate a 2D IMU (Inertial Measurement Unit) sensor.
  """

  dt:               float = 0.01                        # Time step [s]
  t_end:            float = 10.0                        # Total simulation time [s]          
  radius:           float = 1.0                         # Radius of circular path [m]
  omega:            Optional[float] = None              # Angular velocity [rad/s]
  accel_noise_std:  float = 0.05                        # Accelerometer noise standard deviation [m/s^2]
  gyro_noise_std:   float = 0.01                        # Gyroscope noise standard deviation [rad/s]
  accel_bias:       float = 0.01                        # Accelerometer bias [m/s^2]
  gyro_bias:        float = 0.01                        # Gyroscope bias [rad/s]           
  g:                float = 9.81                        # Gravity constant [m/s^2]               

  # Internal fields
  t:                np.ndarray = field(init=False)      # Time vector [s]
  pos:              np.ndarray = field(init=False)      # Position over time [m]
  vel:              np.ndarray = field(init=False)      # Velocity over time [m/s]
  accel_true:       np.ndarray = field(init=False)      # True acceleration [m/s^2]
  gyro_true:        np.ndarray = field(init=False)      # True angular velocity [rad/s]
  accel_meas:       np.ndarray = field(init=False)      # Measured acceleration [m/s^2]
  gyro_meas:        np.ndarray = field(init=False)      # Measured angular velocity [rad/s]


  def __post_init__(self) -> None:
      
    # Initialize time array and angular velocity if not provided

    self.t = np.arange(0, self.t_end, self.dt)
    
    if self.omega is None:
      self.omega = 2 * np.pi / 5

  
  def simulate_motion(self) -> None:
    # Generate a non-circular, dynamic path (e.g. sinusoidal)
    t = self.t

    x = t
    y = 2 * np.sin(1 * t)
    z = np.zeros_like(t)

    self.pos = np.stack((x, y, z), axis=1)

    # Numerical derivative for velocity
    self.vel = np.gradient(self.pos, self.dt, axis=0)

    # Numerical derivative for acceleration
    self.accel_true = np.gradient(self.vel, self.dt, axis=0)

    # Add gravity to Z acceleration
    self.accel_true[:, 2] -= self.g

    # Angular velocity estimation (gyro): here a placeholder using finite diff
    # For a real scenario, use orientation derivatives (e.g., quaternions)
    angular_velocity = np.zeros_like(self.pos)
    angular_velocity[:, 2] = np.gradient(np.arctan2(self.vel[:,1], self.vel[:,0]), self.dt)

    self.gyro_true = angular_velocity

  
  def add_noise(
    self,
    data: np.ndarray,
    noise_std: float,
    bias: float
  ) -> np.ndarray:
      
    # Add Gaussian noise and bias to the data

    noise = np.random.normal(0, noise_std, size=data.shape)
    
    return data + noise + bias


  def generate_measurement(self) -> None:
      
    # Main entry point: Simulate true motion and apply sensor noise

    self.simulate_motion()
    
    self.accel_meas = self.add_noise(
      self.accel_true, 
      self.accel_noise_std, 
      self.accel_bias
    )
    
    self.gyro_meas = self.add_noise(
      self.gyro_true, 
      self.gyro_noise_std, 
      self.gyro_bias
    )

  
  def to_dataframe(self) -> pd.DataFrame:
      
    # Convert simulation results to a Pandas DataFrame

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
        
    # Save simulation results to a CSV file

    df = self.to_dataframe()
    df.to_csv(filename, index=False)
    print(f"Data saved to {filename}")
    

  def plot(self) -> None:
      
    # Plot the simulation results

    plt.figure(figsize=(12, 8))

    # Plot accelerometer measurements
    plt.subplot(2, 1, 1)
    plt.plot(self.t, self.accel_meas[:, 0], label='Accel_X', color='r')
    plt.plot(self.t, self.accel_meas[:, 1], label='Accel_Y', color='g')
    plt.plot(self.t, self.accel_meas[:, 2], label='Accel_Z', color='b')
    plt.title('Accelerometer Measurements')
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s^2]')
    plt.legend()
    
    # Plot gyroscope measurements
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
    
    
  def plot_path_colored_by_velocity(self) -> None:
    # Visualize 2D XY path, colored from blue to red based on speed
    vel_magnitude = np.linalg.norm(self.vel, axis=1)

    plt.figure("2D Path Colored by Velocity", figsize=(8, 6))
    scatter = plt.scatter(
        self.pos[:, 0], self.pos[:, 1],
        c=vel_magnitude,
        cmap='coolwarm',  # 🔁 blue → red
        s=10
    )

    cbar = plt.colorbar(scatter)
    cbar.set_label("Velocity Magnitude [m/s]")
    plt.title("Path Colored by Speed (Blue → Red)")
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
    

if __name__ == "__main__":
  sim = IMU2D()
  sim.generate_measurement()
  sim.plot()
  sim.plot_path_colored_by_velocity()
  