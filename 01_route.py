import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# --- PARAMETERS ---
DT = 1 / 50.0  # 50 Hz
TOTAL_TIME = 1000.0
NUM_STEPS = int(TOTAL_TIME / DT)


# --- HELPER FUNCTIONS ---
def kmh_to_ms(kmh):
    return kmh / 3.6


# --- DATA STORAGE ---
# Using a dictionary for a cleaner structure
history = {
    "time": [],
    "x": [], "y": [],
    "vx": [], "vy": [],
    "ax": [], "ay": [],  # Global accelerations
    "yaw": [], "yaw_rate": [],
    "speed": []
}

# --- INITIAL CONDITIONS ---
x = 0.0
y = 0.0
speed = 0.0  # Speed magnitude (m/s)
yaw = np.radians(90.0)  # 90 degrees (North)

# --- SIMULATION LOOP ---
for step in range(NUM_STEPS):
    t = step * DT

    # -- 1. TARGET DETERMINATION (Acceleration and yaw rate based on scenario) --

    acc_mag = 0.0  # Linear acceleration magnitude (m/s^2)
    angular_vel = 0.0  # Angular velocity (rad/s)

    # Phase 1: 0-10s -> Acceleration (0 -> 100 km/h)
    if t < 10.0:
        target_v = kmh_to_ms(100)
        acc_mag = (target_v - 0) / 10.0

    # Phase 2: 10-20s -> 45 degree right turn (Constant speed)
    elif 10.0 <= t < 20.0:
        angular_vel = np.radians(-45) / 10.0  # Right turn (-)

    # Phase 3 (Transition): 20-200s -> Constant speed and direction
    elif 20.0 <= t < 200.0:
        pass  # Zero acceleration, zero angular velocity

    # Phase 4: 200-230s -> Acceleration (100 -> 150 km/h)
    elif 200.0 <= t < 230.0:
        v_start = kmh_to_ms(100)
        v_end = kmh_to_ms(150)
        acc_mag = (v_end - v_start) / 30.0

    # Phase 5: 230-260s -> 180 degree right turn and Deceleration (150 -> 100 km/h)
    elif 230.0 <= t < 260.0:
        angular_vel = np.radians(-180) / 30.0  # Assumed right turn
        v_start = kmh_to_ms(150)
        v_end = kmh_to_ms(100)
        acc_mag = (v_end - v_start) / 30.0  # Will be negative due to deceleration

    # Phase 6 (Transition): 260-500s -> Constant speed
    elif 260.0 <= t < 500.0:
        pass

    # Phase 7: 500-510s -> Deceleration (100 -> 50 km/h)
    elif 500.0 <= t < 510.0:
        v_start = kmh_to_ms(100)
        v_end = kmh_to_ms(50)
        acc_mag = (v_end - v_start) / 10.0

    # Phase 8 (Transition): 510-800s -> Constant speed
    elif 510.0 <= t < 800.0:
        pass

    # Phase 9: 800-830s -> 90 degree right turn and Acceleration (50 -> 100 km/h)
    elif 800.0 <= t < 830.0:
        angular_vel = np.radians(-90) / 30.0
        v_start = kmh_to_ms(50)
        v_end = kmh_to_ms(100)
        acc_mag = (v_end - v_start) / 30.0

    # Phase 10: Stop within the last 10 seconds
    elif t >= (TOTAL_TIME - 10.0):
        # Prevent negative speed if already stopped
        if speed > 0.01:
            v_start = speed  # Use actual current speed
            v_end = 0
            acc_mag = (v_end - v_start) / 10.0  # Hard braking
        else:
            speed = 0
            acc_mag = 0

    # -- 2. KINEMATIC UPDATE (EULER) --

    # Update yaw and speed magnitude first
    yaw += angular_vel * DT
    speed += acc_mag * DT

    # Prevent negative speed
    if speed < 0: speed = 0

    # Calculate velocity components (vx, vy) based on new yaw
    vx = speed * np.cos(yaw)
    vy = speed * np.sin(yaw)

    # Calculate Global Acceleration (Physics rule: a = dv/dt)
    # This formula includes both longitudinal acceleration and centrifugal force
    # global_ax = acc_linear * cos(yaw) - speed * sin(yaw) * yaw_rate
    global_ax = acc_mag * np.cos(yaw) - speed * np.sin(yaw) * angular_vel
    global_ay = acc_mag * np.sin(yaw) + speed * np.cos(yaw) * angular_vel

    # Update position
    x += vx * DT
    y += vy * DT

    # -- 3. RECORD DATA --
    history["time"].append(t)
    history["x"].append(x)
    history["y"].append(y)
    history["vx"].append(vx)
    history["vy"].append(vy)
    history["ax"].append(global_ax)
    history["ay"].append(global_ay)
    history["yaw"].append(yaw)
    history["yaw_rate"].append(angular_vel)
    history["speed"].append(speed)

# Create DataFrame
df_truth = pd.DataFrame(history)

# --- VISUALIZATION ---
plt.figure(figsize=(10, 8))
plt.plot(df_truth["x"], df_truth["y"], label="True Route", linewidth=2)
plt.title("Step 1: Ground Truth Route")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.savefig("corrected_route.png")

plt.figure(figsize=(10, 4))
plt.plot(df_truth["time"], df_truth["speed"] * 3.6, label="Speed (km/h)", color="orange")
plt.title("Speed Profile")
plt.xlabel("Time (s)")
plt.ylabel("Speed (km/h)")
plt.grid(True)
plt.savefig("speed_profile.png")