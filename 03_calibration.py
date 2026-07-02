import numpy as np
import matplotlib.pyplot as plt

# Load Data
true_data = np.load('ground_truth.npz')
sensor_data = np.load('sensor_measurements.npz')

time = true_data['time']
vx = true_data['vx']
vy = true_data['vy']
yaw = true_data['yaw']

acc_x_raw = sensor_data['imu_accel_x']
acc_y_raw = sensor_data['imu_accel_y']
gyro_raw = sensor_data['imu_gyro_z']
DT = sensor_data['dt']

# These values match the noise parameters used in 02_sensor_model.py
ACCEL_BIAS_X = 0.2
ACCEL_BIAS_Y = 0.1
GYRO_BIAS = np.radians(0.2)

ACCEL_SCALE_X = 1.01
ACCEL_SCALE_Y = 0.99
GYRO_SCALE = 1.05

# - Calibration Formula -
# (measurement / scale) - bias
acc_x_cal = (acc_x_raw / ACCEL_SCALE_X) - ACCEL_BIAS_X
acc_y_cal = (acc_y_raw / ACCEL_SCALE_Y) - ACCEL_BIAS_Y
gyro_cal = (gyro_raw / GYRO_SCALE) - GYRO_BIAS

# - Calculate True Values (For Comparison) -
# ax/ay are already stored in ground_truth.npz
ax_true = true_data['ax']
ay_true = true_data['ay']

# yaw is already in radians (from 02_sensor_model.py), no np.radians() needed
yaw_rate_true = np.diff(np.unwrap(yaw)) / DT
yaw_rate_true = np.append(yaw_rate_true, yaw_rate_true[-1])

# - Visualization -
plt.figure(figsize=(12, 5))
plt.plot(time, acc_x_cal, label="Calibrated X Accel", linewidth=2, color='green')
plt.plot(time, acc_x_raw, label="Raw X Accel", color='orange', linewidth=1)
plt.plot(time, ax_true, label="True X Accel", color='blue')
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s²)")
plt.title("X Axis Acceleration Calibration")
plt.grid(True)
plt.legend()
plt.savefig("calibration_acceleration.png")
plt.close()

plt.figure(figsize=(12, 5))
plt.plot(time, np.degrees(gyro_cal), label="Calibrated Gyro (°/s)", linewidth=2, color='green')
plt.plot(time, np.degrees(gyro_raw), label="Raw Gyro (°/s)", color='orange', linewidth=1)
plt.plot(time, np.degrees(yaw_rate_true), label="True Angular Vel (°/s)", color='blue')
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (°/s)")
plt.title("Gyroscope Calibration")
plt.grid(True)
plt.legend()
plt.savefig("calibration_gyroscope.png")
plt.close()

# - Save Calibrated Data -
np.savez('calibrated_imu.npz',
         acc_x=acc_x_cal,
         acc_y=acc_y_cal,
         gyro_z=gyro_cal,
         dt=DT)