import numpy as np
import matplotlib.pyplot as plt

# Kalibre edilmiş IMU verisi
imu_data = np.load('calibrated_imu_data.npz')
sensor_data = np.load('sensor_data.npz')

acc_x = imu_data['acc_x']
acc_y = imu_data['acc_y']
dt = imu_data['dt']

gnss_vx = sensor_data['gnss_vx']
gnss_vy = sensor_data['gnss_vy']
gnss_t = sensor_data['gnss_time']

# IMU ivmesini entegre ederek hız hesapla
vx = [0]
vy = [0]
for i in range(1, len(acc_x)):
    vx.append(vx[-1] + acc_x[i] * dt)
    vy.append(vy[-1] + acc_y[i] * dt)

vx = np.array(vx)
vy = np.array(vy)

# GNSS zamanlarına karşılık gelen IMU hızları
vx_imu = []
vy_imu = []

for t in gnss_t:
    idx = int(t / dt)
    vx_imu.append(vx[idx])
    vy_imu.append(vy[idx])
vx_imu = np.array(vx_imu)
vy_imu = np.array(vy_imu)

# Alçak geçiren filtre uygulanacak
def low_pass_filter(data, alpha=0.9):
    filtered = [data[0]]
    for n in range(1, len(data)):
        filts = alpha * filtered[-1] + (1 - alpha) * data[n]
        filtered.append(filts)
    return np.array(filtered)
# Uygulama
vx_lpf = low_pass_filter(vx_imu, alpha=0.9)
vy_lpf = low_pass_filter(vy_imu, alpha=0.9)

# Karşılaştırma Grafiği
plt.figure(figsize=(12, 5))
plt.plot(gnss_t, gnss_vx, 'r.', label="GNSS vx")
plt.plot(gnss_t, vx_imu, 'blue', alpha=0.4, label="IMU vx (ham)")
plt.plot(gnss_t, vx_lpf, 'g-', label="IMU vx (Low-Pass)")
plt.xlabel("Zaman (s)")
plt.ylabel("vx (m/s)")
plt.title("VX – Alçak Geçiren Filtre Sonucu")
plt.grid(True)
plt.legend()
plt.show()
