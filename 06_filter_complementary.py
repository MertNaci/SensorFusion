import numpy as np
import matplotlib.pyplot as plt

# Load calibrated IMU gyroscope data
imu_data = np.load('calibrated_imu.npz')
gyro_z = imu_data['gyro_z']
dt = imu_data['dt']

# Load GNSS data
sensor_data = np.load('sensor_measurements.npz')
gnss_vx = sensor_data['gnss_vx']
gnss_vy = sensor_data['gnss_vy']
gnss_t = sensor_data['gnss_time']

# Complementary filter parameter
alpha = 0.95

# Initial heading (yaw) estimation
yaw_est = [np.arctan2(gnss_vy[0], gnss_vx[0])]

for i in range(1, len(gnss_t)):
    # GNSS heading
    yaw_gnss = np.arctan2(gnss_vy[i], gnss_vx[i])

    # IMU prediction (Assuming gyro_z corresponds to GNSS timestamp)
    yaw_imu = yaw_est[-1] + gyro_z[int(gnss_t[i] / dt)] * 0.2

    # Complementary filter fusion
    yaw_fused = alpha * yaw_imu + (1 - alpha) * yaw_gnss

    # Normalize angle to range [-pi, pi]
    yaw_fused = (yaw_fused + np.pi) % (2 * np.pi) - np.pi
    yaw_est.append(yaw_fused)

# Convert results to degrees
yaw_est_deg = np.degrees(yaw_est)
yaw_gnss_deg = np.degrees(np.arctan2(gnss_vy, gnss_vx))

# Save output
np.savez("complementary_filter_results.npz",
         yaw_deg=yaw_est_deg,
         time=gnss_t)

# Visualization
plt.figure(figsize=(12, 5))
plt.plot(gnss_t, yaw_gnss_deg, 'r.', label="GNSS Heading")
plt.plot(gnss_t, yaw_est_deg, 'g-', label="Complementary Filter Heading")
plt.xlabel("Time (s)")
plt.ylabel("Yaw (degrees)")
plt.title("Heading Estimation via Complementary Filter")
plt.grid(True)
plt.legend()
plt.savefig("filter_complementary.png")
plt.close()
