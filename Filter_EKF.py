import numpy as np
import matplotlib.pyplot as plt

def f(x, u, dt):
    x_pos, y_pos, v, yaw = x
    a, omega = u
    x_pos += v * np.cos(yaw) * dt
    y_pos += v * np.sin(yaw) * dt
    v += a * dt
    yaw += omega * dt
    return np.array([x_pos, y_pos, v, yaw])

def jacobian_f(x, u, dt):
    _, _, v, yaw = x
    F = np.eye(4)
    F[0, 2] = np.cos(yaw) * dt
    F[0, 3] = -v * np.sin(yaw) * dt
    F[1, 2] = np.sin(yaw) * dt
    F[1, 3] = v * np.cos(yaw) * dt
    return F

def h(x):
    x_pos, y_pos, v, yaw = x
    vx = v * np.cos(yaw)
    vy = v * np.sin(yaw)
    return np.array([x_pos, y_pos, vx, vy])

def jacobian_h(x):
    _, _, v, yaw = x
    H = np.zeros((4, 4))
    H[0, 0] = 1
    H[1, 1] = 1
    H[2, 2] = np.cos(yaw)
    H[2, 3] = -v * np.sin(yaw)
    H[3, 2] = np.sin(yaw)
    H[3, 3] = v * np.cos(yaw)
    return H

def ekf_predict(x, P, u, dt, Q):
    F = jacobian_f(x, u, dt)
    x_pred = f(x, u, dt)
    P_pred = F @ P @ F.T + Q
    P_pred = np.clip(P_pred, a_min=None, a_max=1e3)
    return x_pred, P_pred

def ekf_update(x_pred, P_pred, z, R):
    H = jacobian_h(x_pred)
    y = z - h(x_pred)
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_updated = x_pred + K @ y
    P_updated = (np.eye(len(x_pred)) - K @ H) @ P_pred
    return x_updated, P_updated

# Veri Yükleme
imu_data = np.load('calibrated_imu_data.npz')
sensor_data = np.load('sensor_data.npz')

acc_x = imu_data['acc_x']
acc_y = imu_data['acc_y']
gyro_z = imu_data['gyro_z']
dt = imu_data['dt'].item()

gnss_x = sensor_data['gnss_x']
gnss_y = sensor_data['gnss_y']
gnss_vx = sensor_data['gnss_vx']
gnss_vy = sensor_data['gnss_vy']
gnss_time = sensor_data['gnss_time']

#EKF Döngüsü
x = np.array([gnss_x[0], gnss_y[0], 0.0, np.deg2rad(90)])
P = np.eye(4) * 1.0
Q = np.diag([0.1, 0.1, 0.3, np.radians(2)])**2
R = np.diag([3.0, 3.0, 0.5, 0.5])**2
x_estimates = []
time_estimates = []
P_all = []
gnss_index = 0

for i in range(len(acc_x)):
    a_forward = acc_x[i] * np.cos(x[3]) + acc_y[i] * np.sin(x[3])
    u = np.array([a_forward, gyro_z[i]])
    # Prediction
    x, P = ekf_predict(x, P, u, dt, Q)
    # GNSS varsa update
    t = i * dt
    if gnss_index < len(gnss_time) and np.isclose(t, gnss_time[gnss_index], atol=dt/2):
        z = np.array([gnss_x[gnss_index], gnss_y[gnss_index], gnss_vx[gnss_index], gnss_vy[gnss_index]])
        x, P = ekf_update(x, P, z, R)
        gnss_index += 1
    x_estimates.append(x.copy())
    time_estimates.append(t)
    P_all.append(P.copy())

x_estimates = np.array(x_estimates)
time_estimates = np.array(time_estimates)
P_all_array = np.stack(P_all, axis=0)

#Kaydet (Visualization.py için)
np.savez("ekf_estimates.npz",
         x_estimates=x_estimates,
         time=time_estimates,
         P_all=P_all_array)

# Görselleştirme
plt.figure(figsize=(10, 6))
plt.plot(gnss_x, gnss_y, 'r.', alpha=0.4, markersize=3, label='GNSS Konumu')
plt.plot(x_estimates[:, 0], x_estimates[:, 1], 'g-',linewidth=2, label='EKF Konum Tahmini')
plt.xlabel("X Pozisyonu (m)")
plt.ylabel("Y Pozisyonu (m)")
plt.title("EKF ile Konum Tahmini")
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()


