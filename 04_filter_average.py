import numpy as np
import matplotlib.pyplot as plt

# Load calibrated IMU data and GNSS data
imu_data = np.load('calibrated_imu.npz')
sensor_data = np.load('sensor_measurements.npz')

acc_x = imu_data['acc_x']
acc_y = imu_data['acc_y']
dt = imu_data['dt']

gnss_vx = sensor_data['gnss_vx']
gnss_vy = sensor_data['gnss_vy']
gnss_t = sensor_data['gnss_time']

# Calculate velocity from IMU (integrate acceleration)
vx_imu = [0]
vy_imu = [0]

for i in range(1, len(acc_x)):
    vx_imu.append(vx_imu[-1] + acc_x[i] * dt)
    vy_imu.append(vy_imu[-1] + acc_y[i] * dt)

vx_imu = np.array(vx_imu)
vy_imu = np.array(vy_imu)

# Match IMU velocities to GNSS timestamps (GNSS is 5 Hz, IMU is 50 Hz)
vx_imu_gnss = []
vy_imu_gnss = []

for t in gnss_t:
    idx = int(t / dt)
    vx_imu_gnss.append(vx_imu[idx])
    vy_imu_gnss.append(vy_imu[idx])

vx_imu_gnss = np.array(vx_imu_gnss)
vy_imu_gnss = np.array(vy_imu_gnss)

# Simple fusion: calculate arithmetic mean
vx_avg = (vx_imu_gnss + gnss_vx) / 2
vy_avg = (vy_imu_gnss + gnss_vy) / 2

# Visualization
plt.figure(figsize=(12, 5))
plt.plot(gnss_t, gnss_vx, 'r.', label="GNSS vx", zorder=1)
plt.plot(gnss_t, vx_avg, 'g-', label="Fusion vx (Average)", zorder=2)
plt.plot(gnss_t, vx_imu_gnss, 'b--', label="IMU vx", zorder=3, linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Velocity X (m/s)")
plt.title("VX Comparison – Average Filter")
plt.grid(True)
plt.legend()
plt.savefig("filter_average.png")
plt.close()
