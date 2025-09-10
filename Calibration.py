import numpy as np
import matplotlib.pyplot as plt
#Verileri yükledim
true_data = np.load('true_state_data.npz')
sensor_data = np.load('sensor_data.npz')
time = true_data['time']
vx = true_data['vx']
vy = true_data['vy']
yaw = true_data['yaw']
acc_x_raw = sensor_data['imu_accel_x']
acc_y_raw = sensor_data['imu_accel_y']
gyro_raw = sensor_data['imu_gyro_z']
DT = sensor_data['dt']

#Alttaki değerleri sensor.py de kullanmıştık
ACCEL_BIAS_X = 0.1
ACCEL_BIAS_Y = 0.05
GYRO_BIAS = np.radians(0.1)

ACCEL_SCALE_X = 1.01
ACCEL_SCALE_Y = 0.99
GYRO_SCALE = 1.02

# - Kalibrasyon Formülü -
# (ölçüm / scale) - bias
acc_x_cal = (acc_x_raw / ACCEL_SCALE_X) - ACCEL_BIAS_X
acc_y_cal = (acc_y_raw / ACCEL_SCALE_Y) - ACCEL_BIAS_Y
gyro_cal = (gyro_raw / GYRO_SCALE) - GYRO_BIAS

# - Gerçek Değerleri Hesaplama (Karşılaştırma için) -
ax_true = np.diff(vx) / DT
ay_true = np.diff(vy) / DT
ax_true = np.append(ax_true, ax_true[-1])
ay_true = np.append(ay_true, ay_true[-1])
yaw_rad = np.radians(yaw)
yaw_rate_true = np.diff(yaw_rad) / DT
yaw_rate_true = np.append(yaw_rate_true, yaw_rate_true[-1])

# - Görselleştirme -
plt.figure(figsize=(12, 5))
plt.plot(time, acc_x_cal, label="Kalibre X ivme",linewidth =2, color='green')
plt.plot(time, acc_x_raw, label="Ham X ivme", color='orange',linewidth=1)
plt.plot(time, ax_true, label="Gerçek X ivme", color='blue')
plt.xlabel("Zaman (s)")
plt.ylabel("Ivme (m/s²)")
plt.title("X Ekseninde Ivme Kalibrasyonu")
plt.grid(True)
plt.legend()

plt.figure(figsize=(12, 5))
plt.plot(time, np.degrees(gyro_cal), label="Kalibre Jiroskop (°/s)",linewidth=2, color='green')
plt.plot(time, np.degrees(gyro_raw), label="Ham Jiroskop (°/s)", color='orange',linewidth=1)
plt.plot(time, np.degrees(yaw_rate_true), label="Gerçek Açısal Hız (°/s)", color='blue')
plt.xlabel("Zaman (s)")
plt.ylabel("Açısal Hız (°/s)")
plt.title("Jiroskop Kalibrasyonu")
plt.grid(True)
plt.legend()
plt.show()

# - Kalibre Edilmiş Veriyi Kaydet -
np.savez('calibrated_imu_data.npz',
         acc_x=acc_x_cal,
         acc_y=acc_y_cal,
         gyro_z=gyro_cal,
         dt=DT)