import numpy as np
import matplotlib.pyplot as plt

# Load calibrated IMU and GNSS data
imu_data = np.load('calibrated_imu.npz')
sensor_data = np.load('sensor_measurements.npz')

acc_x = imu_data['acc_x']
acc_y = imu_data['acc_y']
dt = imu_data['dt']

gnss_vx = sensor_data['gnss_vx']
gnss_vy = sensor_data['gnss_vy']
gnss_t = sensor_data['gnss_time']

# Calculate velocity by integrating IMU acceleration
vx = [0]
vy = [0]
for i in range(1, len(acc_x)):
    vx.append(vx[-1] + acc_x[i] * dt)
    vy.append(vy[-1] + acc_y[i] * dt)

vx = np.array(vx)
vy = np.array(vy)

# Match IMU velocities to GNSS timestamps
vx_imu = []
vy_imu = []

for t in gnss_t:
    idx = int(t / dt)
    vx_imu.append(vx[idx])
    vy_imu.append(vy[idx])
vx_imu = np.array(vx_imu)
vy_imu = np.array(vy_imu)

# Apply low-pass filter
def low_pass_filter(data, alpha=0.9):
    filtered = [data[0]]
    for n in range(1, len(data)):
        filts = alpha * filtered[-1] + (1 - alpha) * data[n]
        filtered.append(filts)
    return np.array(filtered)

# Execution
vx_lpf = low_pass_filter(vx_imu, alpha=0.9)
vy_lpf = low_pass_filter(vy_imu, alpha=0.9)

# Visualization
plt.figure(figsize=(12, 5))
plt.plot(gnss_t, gnss_vx, 'r.', label="GNSS vx")
plt.plot(gnss_t, vx_imu, 'blue', alpha=0.4, label="IMU vx (Raw)")
plt.plot(gnss_t, vx_lpf, 'g-', label="IMU vx (Low-Pass Filtered)")
plt.xlabel("Time (s)")
plt.ylabel("Velocity X (m/s)")
plt.title("VX – Low-Pass Filter Results")
plt.grid(True)
plt.legend()
plt.savefig("filter_lowpass.png")
plt.close()
