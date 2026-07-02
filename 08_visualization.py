import os
import numpy as np
import matplotlib.pyplot as plt

# - Load Data -
true = np.load("ground_truth.npz")
sensor = np.load("sensor_measurements.npz")
imu = np.load("calibrated_imu.npz")
ekf_data = np.load("ekf_results.npz")
complementary_data = np.load("complementary_filter_results.npz")

# Basic data arrays
time = true['time']
x_true = true['x']
y_true = true['y']
ax_true = np.diff(true['vx']) / sensor['dt'][()]
ay_true = np.diff(true['vy']) / sensor['dt'][()]
ax_true = np.append(ax_true, ax_true[-1])
ay_true = np.append(ay_true, ay_true[-1])

acc_x_raw = sensor['imu_accel_x']
acc_y_raw = sensor['imu_accel_y']
acc_x_cal = imu['acc_x']
acc_y_cal = imu['acc_y']

gyro_raw = sensor['imu_gyro_z']
gyro_cal = imu['gyro_z']
yaw_true = true['yaw']
yaw_rad_unwrapped = np.unwrap(yaw_true)  # yaw is already in radians, just unwrap
yaw_rate_true = np.diff(yaw_rad_unwrapped) / sensor['dt'][()]
yaw_rate_true = np.append(yaw_rate_true, yaw_rate_true[-1])

gnss_x = sensor['gnss_x']
gnss_y = sensor['gnss_y']
gnss_time = sensor['gnss_time']

x_estimates = ekf_data['x_estimates']
time_est = ekf_data['time']
P_all = ekf_data['P_all']

# - 1. Position Plot -
plt.figure(figsize=(10, 6))
plt.plot(gnss_x, gnss_y, 'r.', markersize=2, label="GNSS Position")
plt.plot(x_true, y_true, label="Ground Truth Position (x,y)", color='black')
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Position Comparison (Ground Truth vs GNSS)")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_01_position_comparison.png")
plt.close()

# - 2. GNSS X Position over Time -
mask_gnss = (gnss_time >= 290) & (gnss_time <= 510)
mask_true = (time >= 290) & (time <= 510)

plt.figure(figsize=(10, 5))
plt.plot(gnss_time[mask_gnss], gnss_x[mask_gnss], 'r.', label='GNSS X', markersize=4)
plt.plot(time[mask_true], x_true[mask_true], 'b-', label='Ground Truth X')
plt.xlabel("Time (s)")
plt.ylabel("X Position (m)")
plt.title("GNSS vs Ground Truth X Position (Zoomed 290-510s)")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_02_x_position_zoomed.png")
plt.close()

# - 3. GNSS Y Position over Time -
plt.figure(figsize=(10, 5))
plt.plot(gnss_time[mask_gnss], gnss_y[mask_gnss], 'r.', label='GNSS Y', markersize=4)
plt.plot(time[mask_true], y_true[mask_true], 'b-', label='Ground Truth Y')
plt.xlabel("Time (s)")
plt.ylabel("Y Position (m)")
plt.title("GNSS vs Ground Truth Y Position (Zoomed 290-510s)")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_03_y_position_zoomed.png")
plt.close()

# - 4. X-Axis Acceleration Calibration -
plt.figure(figsize=(10, 5))
plt.plot(time, acc_x_raw, color='green', linewidth=2, label='Raw X Acceleration')
plt.plot(time, acc_x_cal, color='orange', linewidth=1, label='Calibrated X Acceleration')
plt.plot(time, ax_true, color='blue', label='True X Acceleration')
plt.xlabel("Time (s)")
plt.ylabel("Acceleration X (m/s²)")
plt.title("X-Axis Acceleration Comparison")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_04_acc_x_calibration.png")
plt.close()

# - 5. Y-Axis Acceleration Calibration -
plt.figure(figsize=(10, 5))
plt.plot(time, acc_y_raw, color='green', linewidth=2, label="Raw Y Acceleration")
plt.plot(time, acc_y_cal, color='orange', linewidth=1, label="Calibrated Y Acceleration")
plt.plot(time, ay_true, label="True Y Acceleration", color='blue')
plt.xlabel("Time (s)")
plt.ylabel("Acceleration Y (m/s²)")
plt.title("Y-Axis Acceleration Comparison")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_05_acc_y_calibration.png")
plt.close()

# - 6. Angular Velocity Comparison -
plt.figure(figsize=(10, 5))
plt.plot(time, np.degrees(gyro_raw), label="Raw Gyroscope (°/s)", color='green', linewidth=2)
plt.plot(time, np.degrees(gyro_cal), label="Calibrated Gyroscope (°/s)", color='orange', linewidth=1)
plt.plot(time, np.degrees(yaw_rate_true), label="True Angular Velocity (°/s)", color='blue')
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (°/s)")
plt.title("Angular Velocity (Yaw Rate) Comparison")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_06_yaw_rate_comparison.png")
plt.close()

# - 7. Position Comparison (x, y) -
plt.figure(figsize=(10, 6))
plt.plot(x_true, y_true, label="Ground Truth Position", color='black', linewidth=2)
plt.plot(gnss_x, gnss_y, 'r.', markersize=3, label='GNSS Measurement')
plt.plot(x_estimates[:, 0], x_estimates[:, 1], 'g-', label='EKF Estimation')
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Position Comparison (x, y)")
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_07_position_comparison_ekf.png")
plt.close()

# - 8. Velocity Comparison (VX) -
vx_true = true['vx']
vx_gnss = sensor['gnss_vx']
vx_ekf = x_estimates[:, 2] * np.cos(x_estimates[:, 3])
plt.figure(figsize=(10, 5))
plt.plot(time, vx_true, label='Ground Truth VX', color='black', linewidth=2)
plt.plot(gnss_time, vx_gnss, 'r.', alpha=0.2, markersize=2, label='GNSS VX')
plt.plot(time_est, vx_ekf, label='EKF VX Estimation', linewidth=2, color='green', zorder=1)
plt.xlabel("Time (s)")
plt.ylabel("Velocity X (m/s)")
plt.title("Velocity Comparison – X Component")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_08_vx_comparison.png")
plt.close()

# - 9. Velocity Comparison (VY) -
vy_true = true['vy']
vy_gnss = sensor['gnss_vy']
vy_ekf = x_estimates[:, 2] * np.sin(x_estimates[:, 3])
plt.figure(figsize=(10, 5))
plt.plot(time, vy_true, label='Ground Truth VY', color='black', linewidth=2)
plt.plot(gnss_time, vy_gnss, 'r.', alpha=0.2, markersize=2, label='GNSS VY')
plt.plot(time_est, vy_ekf, label='EKF VY Estimation', linewidth=2, color='green', zorder=1)
plt.xlabel("Time (s)")
plt.ylabel("Velocity Y (m/s)")
plt.title("Velocity Comparison – Y Component")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_09_vy_comparison.png")
plt.close()

# - 10. Heading (Yaw) Comparison -
yaw_true_deg = true['yaw']
yaw_complementary_deg = complementary_data['yaw_deg']
complementary_time = complementary_data['time']
yaw_ekf_deg = np.degrees(x_estimates[:, 3])

plt.figure(figsize=(10, 5))
plt.plot(time, yaw_true_deg, label="Ground Truth Heading", color='black', zorder=2)
plt.plot(complementary_time, yaw_complementary_deg, label="Complementary Filter", color='blue', zorder=3)
plt.plot(time_est, yaw_ekf_deg, label="EKF Heading Estimation", color='orange')
plt.xlabel("Time (s)")
plt.ylabel("Yaw (degrees)")
plt.title("Heading (Yaw - ψ) Comparison")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_10_yaw_comparison.png")
plt.close()

# - 11. EKF vs GNSS vs Ground Truth (Zoom: 290-510s) -
mask_zoom_true = (time >= 290) & (time <= 510)
mask_zoom_gnss = (gnss_time >= 290) & (gnss_time <= 510)
mask_zoom_est = (time_est >= 290) & (time_est <= 510)

plt.figure(figsize=(12, 5))
plt.plot(time_est[mask_zoom_est], x_estimates[mask_zoom_est, 0], label='EKF X')
plt.plot(gnss_time[mask_zoom_gnss], gnss_x[mask_zoom_gnss], 'r.', alpha=0.8, label='GNSS X')
plt.plot(time[mask_zoom_true], x_true[mask_zoom_true], label='Ground Truth X')
plt.title("X Position – EKF vs GNSS vs Ground Truth (Zoomed 290-510s)")
plt.xlabel("Time (s)")
plt.ylabel("X (m)")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_11_x_position_zoomed_ekf.png")
plt.close()

# - 12.1 EKF Covariance Matrix over Time (Subplots) -
rad2deg_squared = (180 / np.pi) ** 2
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
# Position and velocity variances
axs[0].plot(time_est, P_all[:, 0, 0], label="Var(x)", color='#1f77b4')
axs[0].plot(time_est, P_all[:, 1, 1], label="Var(y)", color='#2ca02c')
axs[0].plot(time_est, P_all[:, 2, 2], label="Var(v)", color='#ff7f0e')
axs[0].set_ylabel("Position & Velocity Variance")
axs[0].legend()
axs[0].grid(True)
# Yaw variance
axs[1].plot(time_est, P_all[:, 3, 3] * rad2deg_squared, label="Var(yaw) (deg²)", color='red')
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Yaw Variance")
axs[1].legend()
axs[1].grid(True)
plt.suptitle("EKF Covariance Components (Over Time)")
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_12_ekf_covariance_subplots.png")
plt.close()

# - 12.2 EKF Covariance Matrix (Logarithmic Y-Axis) -
plt.figure(figsize=(12, 6))
plt.plot(time_est, P_all[:, 0, 0], label="Var(x)", color='#1f77b4')       # soft blue
plt.plot(time_est, P_all[:, 1, 1], label="Var(y)", color='#2ca02c')       # green
plt.plot(time_est, P_all[:, 2, 2], label="Var(v)", color='#ff7f0e')       # orange
plt.plot(time_est, P_all[:, 3, 3] * rad2deg_squared, label="Var(yaw) (deg²)", color='#d62728', zorder=1)  # red
plt.yscale("log")
plt.ylim(1e-2, 1e4)
plt.xlabel("Time (s)")
plt.ylabel("Variance (log scale)")
plt.title("EKF Covariance Components (Logarithmic Y-Axis)")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_12_ekf_covariance_log.png")
plt.close()

# - 13. EKF Error Analysis: Heading (Yaw) ±3σ Confidence Interval -
yaw_est_deg = np.degrees(x_estimates[:, 3])  # EKF yaw estimation (degrees)
yaw_true_deg = true['yaw']  # Ground truth yaw (degrees)
yaw_std_deg = np.degrees(np.sqrt(P_all[:, 3, 3]))  # Standard deviation σ in degrees

plt.figure(figsize=(12, 5))
plt.plot(time_est, yaw_est_deg, label='EKF Yaw Estimation', color='green')
plt.fill_between(time_est,
                 yaw_est_deg - 3 * yaw_std_deg,
                 yaw_est_deg + 3 * yaw_std_deg,
                 color='orange', alpha=0.3, label='±3σ Interval')
plt.plot(time, yaw_true_deg, label='Ground Truth Yaw', color='black', alpha=0.7)
plt.xlabel("Time (s)")
plt.ylabel("Yaw (degrees)")
plt.title("EKF Yaw Estimation with ±3σ Confidence Interval")
plt.grid(True)
plt.legend()
plt.tight_layout()
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/vis_13_yaw_error_bounds.png")
plt.close()
