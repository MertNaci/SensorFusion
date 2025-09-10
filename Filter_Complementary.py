import numpy as np
import matplotlib.pyplot as plt
# Kalibre edilmiş IMU jiroskop verisini yükle
imu_data = np.load('calibrated_imu_data.npz')
gyro_z = imu_data['gyro_z']
dt = imu_data['dt']
# GNSS verileri
sensor_data = np.load('sensor_data.npz')
gnss_vx = sensor_data['gnss_vx']
gnss_vy = sensor_data['gnss_vy']
gnss_t = sensor_data['gnss_time']

# Complementary filter parametresi
alpha = 0.95

# Başlangıç yönelimi
yaw_est = [np.arctan2(gnss_vy[0], gnss_vx[0])]

for i in range(1, len(gnss_t)):
    # GNSS yönelimi
    yaw_gnss = np.arctan2(gnss_vy[i], gnss_vx[i])

    # IMU ile tahmin (gyro_z GNSS zamanına denk geldiği varsayılıyor)
    yaw_imu = yaw_est[-1] + gyro_z[int(gnss_t[i] / dt)] * dt

    # Complementary filtre
    yaw_fused = alpha * yaw_imu + (1 - alpha) * yaw_gnss

    # Açıyı -pi ile pi arasına getir
    yaw_fused = (yaw_fused + np.pi) % (2 * np.pi) - np.pi
    yaw_est.append(yaw_fused)

# Sonucu dereceye çevir
yaw_est_deg = np.degrees(yaw_est)
yaw_gnss_deg = np.degrees(np.arctan2(gnss_vy, gnss_vx))
# Kaydet
np.savez("complementary_filter.npz",
         yaw_deg=yaw_est_deg,
         time=gnss_t)
# Görselleştirme
plt.figure(figsize=(12, 5))
plt.plot(gnss_t, yaw_gnss_deg, 'r.', label="GNSS Yönelimi")
plt.plot(gnss_t, yaw_est_deg, 'g-', label="Complementary Filter Yönelimi")
plt.xlabel("Zaman (saniye)")
plt.ylabel("Yaw (derece)")
plt.title("Tamamlayıcı Filtre ile Yönelim Tahmini")
plt.grid(True)
plt.legend()
plt.show()



