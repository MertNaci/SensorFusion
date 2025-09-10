import numpy as np
import matplotlib.pyplot as plt

# Kalibre edilmiş IMU verilerini ve GNSS verilerini yükle
imu_data = np.load('calibrated_imu_data.npz')
sensor_data = np.load('sensor_data.npz')
acc_x = imu_data['acc_x']
acc_y = imu_data['acc_y']
dt = imu_data['dt']
gnss_vx = sensor_data['gnss_vx']
gnss_vy = sensor_data['gnss_vy']
gnss_t = sensor_data['gnss_time']

# IMU'dan hız hesaplamak (ivmeyi entegre et)
vx_imu = [0]
vy_imu = [0]

for i in range(1, len(acc_x)):
    vx_imu.append(vx_imu[-1] + acc_x[i] * dt)
    vy_imu.append(vy_imu[-1] + acc_y[i] * dt)

vx_imu = np.array(vx_imu)
vy_imu = np.array(vy_imu)

# GNSS verisi 5 Hz, IMU 50 Hz → GNSS zamanlarına karşılık gelen IMU hızlarını eşleştir
vx_imu_gnss = []
vy_imu_gnss = []

for t in gnss_t:
    idx = int(t / dt)
    vx_imu_gnss.append(vx_imu[idx])
    vy_imu_gnss.append(vy_imu[idx])

vx_imu_gnss = np.array(vx_imu_gnss)
vy_imu_gnss = np.array(vy_imu_gnss)

# Basit füzyon: ortalama al
vx_avg = (vx_imu_gnss + gnss_vx) / 2
vy_avg = (vy_imu_gnss + gnss_vy) / 2

# Grafikle karşılaştır
plt.figure(figsize=(12, 5))
plt.plot(gnss_t, gnss_vx, 'r.', label="GNSS vx")
plt.plot(gnss_t, vx_imu_gnss, 'b--', label="IMU'dan vx")
plt.plot(gnss_t, vx_avg, 'g-', label="Füzyon vx (Ortalama)")
plt.xlabel("Zaman (s)")
plt.ylabel("vx (m/s)")
plt.title("VX Karşılaştırması – Ortalama Filtre")
plt.grid(True)
plt.legend()
plt.show()
