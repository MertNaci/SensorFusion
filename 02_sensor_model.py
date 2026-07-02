import os
import numpy as np
import matplotlib.pyplot as plt

# --- PARAMETERS ---
DT = 1 / 50.0  # 50 Hz
TOTAL_TIME = 1000.0
NUM_STEPS = int(TOTAL_TIME / DT)

# Sensor Noise Parameters
ACCEL_NOISE_STD = 1.0  # m/s^2
GYRO_NOISE_STD = np.radians(0.5)  # rad/s

ACCEL_BIAS_X = 0.2
ACCEL_BIAS_Y = 0.1
GYRO_BIAS = np.radians(0.2)

ACCEL_SCALE_X = 1.01
ACCEL_SCALE_Y = 0.99
GYRO_SCALE = 1.05

GNSS_POS_NOISE_STD = 10.0  # meters
GNSS_VEL_NOISE_STD = 1.5  # m/s
GNSS_UPDATE_RATE = 5  # Hz
GNSS_UPDATE_INTERVAL = int(1 / (GNSS_UPDATE_RATE * DT))  # Every 10 steps
GNSS_DELAY_STEPS = int(0.2 / DT)  # 200ms delay = 10 steps


# --- HELPER FUNCTION ---
def kmh_to_ms(kmh): return kmh / 3.6


# --- DATA STORAGE STRUCTURES ---
# Ground Truth
true_data = {
    "time": [], "x": [], "y": [], "vx": [], "vy": [],
    "ax": [], "ay": [], "yaw": [], "yaw_rate": [], "speed": []
}

# Sensor Measurements
sensor_data = {
    "time": [],  # Sensor time
    "imu_ax": [], "imu_ay": [], "imu_gyro": [],
    "gnss_time": [], "gnss_x": [], "gnss_y": [], "gnss_vx": [], "gnss_vy": []
}

# --- SIMULATION VARIABLES ---
x, y = 0.0, 0.0
speed = 0.0
yaw = np.radians(90.0)

# GNSS Signal Outage/Flatline Flags
flatline_active = False
flatline_data = None

# --- MAIN LOOP (ROUTE + SENSOR) ---
for step in range(NUM_STEPS):
    t = step * DT

    # ---------------------------------------------------------
    # PART 1: GROUND TRUTH KINEMATICS
    # ---------------------------------------------------------
    acc_mag = 0.0
    angular_vel = 0.0

    # Scenario Phases
    if t < 10.0:  # Acceleration
        acc_mag = (kmh_to_ms(100) - 0) / 10.0
    elif 10.0 <= t < 20.0:  # Turn
        angular_vel = np.radians(-45) / 10.0
    elif 200.0 <= t < 230.0:  # Acceleration 100->150
        acc_mag = (kmh_to_ms(150) - kmh_to_ms(100)) / 30.0
    elif 230.0 <= t < 260.0:  # Turn + Deceleration
        angular_vel = np.radians(-180) / 30.0
        acc_mag = (kmh_to_ms(100) - kmh_to_ms(150)) / 30.0
    elif 500.0 <= t < 510.0:  # Deceleration 100->50
        acc_mag = (kmh_to_ms(50) - kmh_to_ms(100)) / 10.0
    elif 800.0 <= t < 830.0:  # Turn + Acceleration 50->100
        angular_vel = np.radians(-90) / 30.0
        acc_mag = (kmh_to_ms(100) - kmh_to_ms(50)) / 30.0
    elif t >= (TOTAL_TIME - 10.0):  # Stop
        if speed > 0.01:
            acc_mag = -speed / (TOTAL_TIME - t)  # Simple stop
        else:
            speed, acc_mag = 0, 0

    # Kinematic Update
    yaw += angular_vel * DT
    speed += acc_mag * DT
    if speed < 0: speed = 0

    vx = speed * np.cos(yaw)
    vy = speed * np.sin(yaw)

    # Global Acceleration (Reference for IMU)
    global_ax = acc_mag * np.cos(yaw) - speed * np.sin(yaw) * angular_vel
    global_ay = acc_mag * np.sin(yaw) + speed * np.cos(yaw) * angular_vel

    x += vx * DT
    y += vy * DT

    # Record True Data
    true_data["time"].append(t)
    true_data["x"].append(x)
    true_data["y"].append(y)
    true_data["vx"].append(vx)
    true_data["vy"].append(vy)
    true_data["ax"].append(global_ax)
    true_data["ay"].append(global_ay)
    true_data["yaw"].append(yaw)
    true_data["yaw_rate"].append(angular_vel)
    true_data["speed"].append(speed)

    # ---------------------------------------------------------
    # PART 2: SENSOR SIMULATION
    # ---------------------------------------------------------

    # -- A) IMU MODEL (50 Hz) --
    # Model: (True + Bias) * Scale + Noise
    imu_ax_meas = (global_ax + ACCEL_BIAS_X) * ACCEL_SCALE_X + np.random.normal(0, ACCEL_NOISE_STD)
    imu_ay_meas = (global_ay + ACCEL_BIAS_Y) * ACCEL_SCALE_Y + np.random.normal(0, ACCEL_NOISE_STD)
    imu_gyro_meas = (angular_vel + GYRO_BIAS) * GYRO_SCALE + np.random.normal(0, GYRO_NOISE_STD)

    sensor_data["time"].append(t)
    sensor_data["imu_ax"].append(imu_ax_meas)
    sensor_data["imu_ay"].append(imu_ay_meas)
    sensor_data["imu_gyro"].append(imu_gyro_meas)

    # -- B) GNSS MODEL (5 Hz with Latency) --
    if step % GNSS_UPDATE_INTERVAL == 0:
        # Latency/Delay
        delay_idx = step - GNSS_DELAY_STEPS
        if delay_idx < 0: delay_idx = 0

        # Delayed Ground Truth
        d_x = true_data["x"][delay_idx]
        d_y = true_data["y"][delay_idx]
        d_vx = true_data["vx"][delay_idx]
        d_vy = true_data["vy"][delay_idx]

        # -- SPECIAL DISTURBANCE SCENARIOS --
        gnss_outage = False

        # 1. Signal Outage (300-310 s)
        if 300.0 <= t <= 310.0:
            gnss_outage = True

        # 2. Position Jump Error (500m jump at 400 s)
        if 400.0 <= t < 401.0:
            d_x += 500.0
            d_y += 500.0

        # 3. Signal Flatline (Frozen output for 5 s at 500 s)
        if 500.0 <= t < 505.0:
            if not flatline_active:
                flatline_data = (d_x, d_y, d_vx, d_vy)
                flatline_active = True
            # Use frozen data
            d_x, d_y, d_vx, d_vy = flatline_data
        else:
            flatline_active = False

        # GNSS Measurement (Adding Noise) — skip if outage
        if not gnss_outage:
            gnss_x = d_x + np.random.normal(0, GNSS_POS_NOISE_STD)
            gnss_y = d_y + np.random.normal(0, GNSS_POS_NOISE_STD)
            gnss_vx = d_vx + np.random.normal(0, GNSS_VEL_NOISE_STD)
            gnss_vy = d_vy + np.random.normal(0, GNSS_VEL_NOISE_STD)

            sensor_data["gnss_time"].append(t)
            sensor_data["gnss_x"].append(gnss_x)
            sensor_data["gnss_y"].append(gnss_y)
            sensor_data["gnss_vx"].append(gnss_vx)
            sensor_data["gnss_vy"].append(gnss_vy)

# --- FILE SAVING ---
np.savez("ground_truth.npz",
         time=true_data["time"], x=true_data["x"], y=true_data["y"],
         vx=true_data["vx"], vy=true_data["vy"],
         ax=true_data["ax"], ay=true_data["ay"],
         yaw=true_data["yaw"], speed=true_data["speed"])

np.savez("sensor_measurements.npz",
         imu_accel_x=sensor_data["imu_ax"], imu_accel_y=sensor_data["imu_ay"], imu_gyro_z=sensor_data["imu_gyro"],
         gnss_time=sensor_data["gnss_time"], gnss_x=sensor_data["gnss_x"], gnss_y=sensor_data["gnss_y"],
         gnss_vx=sensor_data["gnss_vx"], gnss_vy=sensor_data["gnss_vy"],
         dt=DT)

# --- VERIFICATION PLOTS ---
plt.figure(figsize=(10, 8))
plt.plot(true_data["x"], true_data["y"], 'b-', label='Ground Truth Route', linewidth=2)
plt.plot(sensor_data["gnss_x"], sensor_data["gnss_y"], 'r.', label='GNSS Measurements', markersize=2, alpha=0.5)
plt.title("Sensor Simulation Result: GNSS vs Ground Truth")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend()
plt.grid(True)
plt.axis("equal")
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/sensor_check_gnss.png")

plt.figure(figsize=(10, 4))
plt.plot(true_data["time"], true_data["ax"], 'b', label='True Ax')
plt.plot(sensor_data["time"], sensor_data["imu_ax"], 'orange', alpha=0.5, label='IMU Ax (Noisy)')
plt.xlim(0, 100)  # View first 100 seconds
plt.title("IMU Acceleration Verification (First 100s)")
plt.legend()
plt.grid(True)
os.makedirs("plots", exist_ok=True)
plt.savefig("plots/sensor_check_imu.png")