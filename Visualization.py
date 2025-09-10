import numpy as np
import matplotlib.pyplot as plt
# - Veri Yükleme -
true = np.load("true_state_data.npz")
sensor = np.load("sensor_data.npz")
imu = np.load("calibrated_imu_data.npz")
ekf_data = np.load("ekf_estimates.npz")
complementary_data = np.load("complementary_filter.npz")

# Temel veriler
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
yaw_rad_unwrapped = np.unwrap(np.radians(yaw_true))  # Ani sıçramaları önlemek için düzleştirme yapıldı
yaw_rate_true = np.diff(yaw_rad_unwrapped) / sensor['dt'][()]
yaw_rate_true = np.append(yaw_rate_true, yaw_rate_true[-1])


gnss_x = sensor['gnss_x']
gnss_y = sensor['gnss_y']
gnss_time = sensor['gnss_time']
x_estimates = ekf_data['x_estimates']
time_est = ekf_data['time']
P_all = ekf_data['P_all']

# - 1. Pozisyon Grafiği -
plt.figure(figsize=(10, 6))
plt.plot(gnss_x, gnss_y, 'r.', markersize=2, label="GNSS Pozisyonu")
plt.plot(x_true, y_true, label="Gerçek Pozisyon (x,y)", color='black')
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Pozisyon Karşılaştırması (Gerçek vs GNSS)")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()
plt.show()

# - 2. GNSS X Pozisyonu Zamana Karşı Değişimi -
mask_gnss = (gnss_time >= 290) & (gnss_time <= 510)
mask_true = (time >= 290) & (time <= 510)

plt.figure(figsize=(10, 5))
plt.plot(gnss_time[mask_gnss], gnss_x[mask_gnss], 'r.', label='GNSS X', markersize=4)
plt.plot(time[mask_true], x_true[mask_true], 'b-', label='Gerçek X')
plt.xlabel("Zaman (saniye)")
plt.ylabel("X Pozisyonu (metre)")
plt.title("GNSS vs Gerçek X Pozisyonu (290–510 sn yakınlaştırılmış)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 3. GNSS Y Pozisyonu Zamana Karşı Değişimi -
plt.figure(figsize=(10, 5))
plt.plot(gnss_time[mask_gnss], gnss_y[mask_gnss], 'r.', label='GNSS Y', markersize=4)
plt.plot(time[mask_true], y_true[mask_true], 'b-', label='Gerçek Y')
plt.xlabel("Zaman (saniye)")
plt.ylabel("Y Pozisyonu (metre)")
plt.title("GNSS vs Gerçek Y Pozisyonu (290–510 sn yakınlaştırılmış)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 4. X Ekseninde İvme Kalibrasyonu -
plt.figure(figsize=(10, 5))
plt.plot(time, acc_x_raw, color='green',linewidth=2, label='Ham X ivme')
plt.plot(time, acc_x_cal, color='orange',linewidth=1, label='Kalibre X ivme')
plt.plot(time, ax_true, color='blue',label='Gerçek X ivme')

plt.xlabel("Zaman (s)")
plt.ylabel("Ivme X (m/s²)")
plt.title("X Ekseninde İvme Karşılaştırması")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 5. Y Ekseninde İvme Kalibrasyonu -
plt.figure(figsize=(10, 5))
plt.plot(time, acc_y_raw,color='green',linewidth=2, label="Ham Y ivme")
plt.plot(time, acc_y_cal,color='orange',linewidth=1, label="Kalibre Y ivme" )
plt.plot(time, ay_true, label="Gerçek Y ivme", color='blue')
plt.xlabel("Zaman (s)")
plt.ylabel("Ivme Y (m/s²)")
plt.title("Y Ekseninde İvme Karşılaştırması")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 6. Açısal Hız Karşılaştırması -
plt.figure(figsize=(10, 5))
plt.plot(time, np.degrees(gyro_raw), label="Ham Jiroskop (°/s)", color='green',linewidth=2)
plt.plot(time, np.degrees(gyro_cal), label="Kalibre Jiroskop (°/s)",color='orange',linewidth=1)
plt.plot(time, np.degrees(yaw_rate_true), label="Gerçek Açısal Hız (°/s)", color='blue')
plt.xlabel("Zaman (s)")
plt.ylabel("Açısal Hız (°/s)")
plt.title("Açısal Hız (Yaw Rate) Karşılaştırması")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 7. Pozisyon Karşılaştırması (x, y) -
plt.figure(figsize=(10, 6))
plt.plot(x_true, y_true, label="Gerçek Pozisyon", color='black', linewidth=2)
plt.plot(gnss_x, gnss_y, 'r.', markersize=3, label='GNSS Ölçümü')
plt.plot(x_estimates[:, 0], x_estimates[:, 1], 'g-', label='EKF Tahmini')
plt.xlabel("X Pozisyonu (m)")
plt.ylabel("Y Pozisyonu (m)")
plt.title("Pozisyon Karşılaştırması (x, y)")
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 8. Hız Karşılaştırması (VX) -
vx_true = true['vx']
vx_gnss = sensor['gnss_vx']
vx_ekf = x_estimates[:, 2] * np.cos(x_estimates[:, 3])
plt.figure(figsize=(10, 5))
plt.plot(time, vx_true, label='Gerçek VX', color='black',linewidth=2)
plt.plot(gnss_time, vx_gnss, 'r.',alpha=0.2, markersize=2, label='GNSS VX')
plt.plot(time_est, vx_ekf, label='EKF VX Tahmini',linewidth=2, color='green',zorder=1)
plt.xlabel("Zaman (s)")
plt.ylabel("VX (m/s)")
plt.title("Hız Karşılaştırması – X Bileşeni")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 9. Hız Karşılaştırması (VY) -
vy_true = true['vy']
vy_gnss = sensor['gnss_vy']
vy_ekf = x_estimates[:, 2] * np.sin(x_estimates[:, 3])
plt.figure(figsize=(10, 5))
plt.plot(time, vy_true, label='Gerçek VY', color='black',linewidth=2)
plt.plot(gnss_time, vy_gnss, 'r.',alpha=0.2, markersize=2, label='GNSS VY')
plt.plot(time_est, vy_ekf, label='EKF VY Tahmini',linewidth=2, color='green',zorder=1)
plt.xlabel("Zaman (s)")
plt.ylabel("VY (m/s)")
plt.title("Hız Karşılaştırması – Y Bileşeni")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 10. Yönelim Karşılaştırması -
yaw_true_deg = true['yaw']
yaw_complementary_deg = complementary_data['yaw_deg']
complementary_time = complementary_data['time']
yaw_ekf_deg = np.degrees(x_estimates[:, 3])
plt.figure(figsize=(10, 5))
plt.plot(time, yaw_true_deg, label="Gerçek Yönelim", color='black',zorder=2)
plt.plot(complementary_time, yaw_complementary_deg, label="Complementary Filter", color='blue' ,zorder=3)
plt.plot(time_est, yaw_ekf_deg, label="EKF Yönelim Tahmini", color='orange')
plt.xlabel("Zaman (saniye)")
plt.ylabel("Yaw (derece)")
plt.title("Yönelim (Yaw - ψ) Karşılaştırması")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 11. EKF vs GNSS vs Gerçek (Zoom: 290–510s) -
mask_zoom_true = (time >= 290) & (time <= 510)
mask_zoom_gnss = (gnss_time >= 290) & (gnss_time <= 510)
mask_zoom_est = (time_est >= 290) & (time_est <= 510)
plt.figure(figsize=(12, 5))
plt.plot(time_est[mask_zoom_est], x_estimates[mask_zoom_est, 0], label='EKF X')
plt.plot(gnss_time[mask_zoom_gnss], gnss_x[mask_zoom_gnss], 'r.',alpha=0.8, label='GNSS X')
plt.plot(time[mask_zoom_true], x_true[mask_zoom_true], label='Gerçek X')
plt.title("X Pozisyonu – EKF vs GNSS vs Gerçek (Zoom: 290–510s)")
plt.xlabel("Zaman (saniye)")
plt.ylabel("X (metre)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 12.1 EKF Kovaryans Matrisinin Zamanla Değişimi (Alt Alta Ayrı Grafiklerle) -
# Radyandan dereceye dönüşüm için katsayı
rad2deg_squared = (180 / np.pi) ** 2
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
# Pozisyon ve hız varyansları
axs[0].plot(time_est, P_all[:, 0, 0], label="Var(x)", color='#1f77b4')
axs[0].plot(time_est, P_all[:, 1, 1], label="Var(y)", color='#2ca02c')
axs[0].plot(time_est, P_all[:, 2, 2], label="Var(v)",color='#ff7f0e')
axs[0].set_ylabel("Pozisyon & Hız Varyansı")
axs[0].legend()
axs[0].grid(True)
# Yaw varyansı
axs[1].plot(time_est, P_all[:, 3, 3] * rad2deg_squared, label="Var(yaw) (deg²)", color='red')
axs[1].set_xlabel("Zaman (s)")
axs[1].set_ylabel("Yaw Varyansı")
axs[1].legend()
axs[1].grid(True)
plt.suptitle("EKF Kovaryans Bileşenleri (Zamanla Değişim - Alt Grafiklerle)")
plt.tight_layout()
plt.show()

# - 12.2 EKF Kovaryans Matrisinin Logaritmik Y Ekseniyle Çizimi -
plt.figure(figsize=(12, 6))
plt.plot(time_est, P_all[:, 0, 0], label="Var(x)", color='#1f77b4')       # soft blue
plt.plot(time_est, P_all[:, 1, 1], label="Var(y)", color='#2ca02c')       # green
plt.plot(time_est, P_all[:, 2, 2], label="Var(v)", color='#ff7f0e')       # orange
plt.plot(time_est, P_all[:, 3, 3] * rad2deg_squared, label="Var(yaw) (deg²)", color='#d62728', zorder=1)  # red
plt.yscale("log")
plt.ylim(1e-2, 1e4)
plt.xlabel("Zaman (s)")
plt.ylabel("Varyans (log)")
plt.title("EKF Kovaryans Bileşenleri (Logaritmik Y Ekseni)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# - 13. EKF Hata Analizi: Yönelim (Yaw) ±3σ Güven Aralığı -
yaw_est_deg = np.degrees(x_estimates[:, 3])  # EKF yaw tahmini (derece)
yaw_true_deg = true['yaw']  # Gerçek yaw (derece)
yaw_std_deg = np.degrees(np.sqrt(P_all[:, 3, 3]))  # Radyan^2 → derece cinsinden σ

plt.figure(figsize=(12, 5))
plt.plot(time_est, yaw_est_deg, label='EKF Yaw Tahmini', color='green')
plt.fill_between(time_est,
                 yaw_est_deg - 3 * yaw_std_deg,
                 yaw_est_deg + 3 * yaw_std_deg,
                 color='orange', alpha=0.3, label='±3σ Aralığı')
plt.plot(time, yaw_true_deg, label='Gerçek Yaw', color='black', alpha=0.7)
plt.xlabel("Zaman (saniye)")
plt.ylabel("Yaw (derece)")
plt.title("EKF Yaw Tahmini ±3σ Güven Aralığı ile")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

