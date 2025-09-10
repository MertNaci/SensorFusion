import numpy as np
import matplotlib.pyplot as plt
# rotadaki verileri yükle
loaded_data = np.load('true_state_data.npz')
time_true = loaded_data['time']
x_true = loaded_data['x']
y_true = loaded_data['y']
vx_true = loaded_data['vx']
vy_true = loaded_data['vy']
yaw_true = loaded_data['yaw']
speed_true = loaded_data['speed']

# İvme ve Açısal hızı doğrudan rota simülasyonundan alınmadığı için türevini almamız gerekecek
# İvme (a_x_true, a_y_true)
DT = time_true[1] - time_true[0]  # Zaman adımını tekrar alalım
ax_true = np.diff(vx_true) / DT
ay_true = np.diff(vy_true) / DT
# Diff işlemi dizinin boyutunu 1 kısaltır, son elemanı tekrar ettim
ax_true = np.append(ax_true, ax_true[-1])  # Son elemanı tekrarladım
ay_true = np.append(ay_true, ay_true[-1])

# Açısal hız (yaw rate)
# Yaw değeri dereceden radyana çevrilmişti, diff almadan önce radyana çevirelim
yaw_rad_true = np.radians(yaw_true)
angular_velocity_true = np.diff(yaw_rad_true) / DT
angular_velocity_true = np.append(angular_velocity_true, angular_velocity_true[-1])
angular_velocity_true_deg = np.degrees(angular_velocity_true)  # Derece/saniye olarak kaydetmek için

# Toplam adım sayısı
NUM_STEPS = len(time_true)
# DT zaten yukarıda tanımlı

# - SENSÖR PARAMETRELERİ -
# IMU (Ivmeölçer ve Jiroskop)
# Gürültü (Standart sapma - Örnek değerler, gerçek sensör datasheet'inden alıncak)
ACCEL_NOISE_STD = 1  # m/s^2 (ivme ölçer gürültüsü)
GYRO_NOISE_STD = np.radians(0.5)  # rad/s (jiroskop gürültüsü, 0.5 derece/s)

# Bias (Sapma - Sensörün sıfır okuması olması gerektiğinde verdiği sabit hata)
ACCEL_BIAS_X = 0.2  # m/s^2
ACCEL_BIAS_Y = 0.1  # m/s^2
GYRO_BIAS = np.radians(0.2)  # rad/s (0.2 derece/s)

# Scale Factor (Ölçek Faktörü - Sensörün gerçek değeri yanlış ölçeklendirmesi)
ACCEL_SCALE_X = 1.01
ACCEL_SCALE_Y = 0.99
GYRO_SCALE = 1.05

# GNSS (GPS)
GNSS_POS_NOISE_STD = 10.0  # metre (Pozisyon gürültüsü)
GNSS_VEL_NOISE_STD = 1.5  # m/s (Hız gürültüsü)

GNSS_UPDATE_RATE = 5  # Hz
GNSS_UPDATE_INTERVAL = int(1 / (GNSS_UPDATE_RATE * DT))  # GNSS kaç DT'de bir güncelleme yapacak (50Hz / 5Hz = 10)
GNSS_DELAY_SECONDS = 0.2  # saniye (200 milisaniye gecikme)
GNSS_DELAY_STEPS = int(GNSS_DELAY_SECONDS / DT)  # Kaç adım gecikme olacak

# GNSS sabit veri üretme (flatline) durumu için global flag ve değerler
_gnss_flatline_active = False
_flatline_x_val = 0.0
_flatline_y_val = 0.0
_flatline_vx_val = 0.0
_flatline_vy_val = 0.0

# Sensör verilerini depolamak için listeler
imu_accel_x_meas = []
imu_accel_y_meas = []
imu_gyro_z_meas = []  # Jiroskop Z ekseni (Yaw için)

gnss_x_meas = []
gnss_y_meas = []
gnss_vx_meas = []
gnss_vy_meas = []
gnss_time_meas = []  # GNSS ölçüm zamanları

# - SENSÖR SİMÜLASYONU DÖNGÜSÜ -
for i in range(NUM_STEPS):
    # - IMU SİMÜLASYONU -
    # Hızların türevini alarak ivmeyi ve yaw'ın türevini alarak açısal hızı hesaplıcam
    # IMU ölçüm modeli: ölçülen = (gerçek + bias) * scale_factor + gürültü
    # Ivmeölçer gerçek ivme değerlerine bias eklencek ve scale factor eklenerek sonra gürültü(0 ile 0.5 arasında) eklenecek
    imu_x_accel = (ax_true[i] + ACCEL_BIAS_X) * ACCEL_SCALE_X + np.random.normal(0, ACCEL_NOISE_STD)
    imu_y_accel = (ay_true[i] + ACCEL_BIAS_Y) * ACCEL_SCALE_Y + np.random.normal(0, ACCEL_NOISE_STD)

    # Jiroskop - Z ekseni (Yaw rate)
    # Jiroskop gerçek açısal hıza gürültü, bias ve scale factor eklenerek modellenecektir.
    imu_gyro = (angular_velocity_true[i] + GYRO_BIAS) * GYRO_SCALE + np.random.normal(0, GYRO_NOISE_STD)

    imu_accel_x_meas.append(imu_x_accel)
    imu_accel_y_meas.append(imu_y_accel)
    imu_gyro_z_meas.append(imu_gyro)

    # -GNSS SİMÜLASYONU-
    # GNSS pozisyon ve hız çıktıları üretecektir. Belirlenen frekansta ve gecikmeyle çalışacaktır.
    if i % GNSS_UPDATE_INTERVAL == 0:
        # Gecikme uygulanacak olan anın indeksi
        delay_index = i - GNSS_DELAY_STEPS
        # Gecikme indeksi 0'dan küçükse (simülasyonun başıysa), gecikme uygulanmaz, mevcut anın verisi alınır
        if delay_index < 0:
            delayed_x = x_true[i]
            delayed_y = y_true[i]
            delayed_vx = vx_true[i]
            delayed_vy = vy_true[i]
        else:
            delayed_x = x_true[delay_index]
            delayed_y = y_true[delay_index]
            delayed_vx = vx_true[delay_index]
            delayed_vy = vy_true[delay_index]

            # - GNSS Sensörü Ek Bozuklukları (Adım 3 Gereklilikleri) -
        current_time = time_true[i]

        # 1. 300. ve 310. saniyeler arasında çıktı üretmeme
        if 300.0 <= current_time <= 310.0:
            continue  # Bu adımda GNSS ölçümü yapılmaz, sonraki iterasyona geç

        # 2. 400. saniyede 500 metre hatalı gösterme ve 1 saniye sonra düzeltme
        if 400.0 <= current_time < (400.0 + 1.0):
            delayed_x += 500.0
            delayed_y += 500.0

        # 3. 500. saniyede 5 saniye boyunca sabit veri üretme
        if 500.0 <= current_time < 505.0:
            if not _gnss_flatline_active:
                # Sabitleme ilk kez başlıyor, o anki gecikmeli gerçek değeri kaydet
                _flatline_x_val = delayed_x
                _flatline_y_val = delayed_y
                _flatline_vx_val = delayed_vx
                _flatline_vy_val = delayed_vy
                _gnss_flatline_active = True

            # Sabitlenmiş değerleri kullan
            delayed_x = _flatline_x_val
            delayed_y = _flatline_y_val
            delayed_vx = _flatline_vx_val
            delayed_vy = _flatline_vy_val
        else:
            # Sabitleme süresi dışındaysa flag'i sıfırladık
            _gnss_flatline_active = False

        # GNSS ölçüm modeli: ölçülen = gerçek (gecikmeli) + gürültü (ek bozucular uygulandıktan sonra)
        gnss_x_m = delayed_x + np.random.normal(0, GNSS_POS_NOISE_STD)
        gnss_y_m = delayed_y + np.random.normal(0, GNSS_POS_NOISE_STD)
        gnss_vx_m = delayed_vx + np.random.normal(0, GNSS_VEL_NOISE_STD)
        gnss_vy_m = delayed_vy + np.random.normal(0, GNSS_VEL_NOISE_STD)

        gnss_x_meas.append(gnss_x_m)
        gnss_y_meas.append(gnss_y_m)
        gnss_vx_meas.append(gnss_vx_m)
        gnss_vy_meas.append(gnss_vy_m)
        gnss_time_meas.append(time_true[i])  # GNSS ölçümünün yapıldığı zaman

# -SENSÖR ÇIKTILARINI GÖRSELLEŞTİRME-
# GNSS Pozisyonu (Gerçek Rota ile Karşılaştırma)
plt.figure(figsize=(10, 8))
plt.plot(gnss_x_meas, gnss_y_meas, 'r.', label='GNSS Pozisyon Ölçümleri (Gecikmeli ve Gürültülü)', alpha=0.5,
         markersize=3)
plt.plot(x_true, y_true, color='blue', label='Gerçek Rota', alpha=0.7)
plt.xlabel('X Pozisyonu (metre)')
plt.ylabel('Y Pozisyonu (metre)')
plt.title('GNSS Pozisyon Ölçümleri vs. Gerçek Rota')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
# IMU Ivme (Gerçek Ivme ile Karşılaştırma)
plt.figure(figsize=(12, 6))
plt.plot(time_true, imu_accel_x_meas, color='orange', label='IMU X Ivme Ölçümü', alpha=0.9)
plt.plot(time_true, ax_true, color='blue', label='Gerçek X Ivmesi', alpha=0.7)
plt.xlabel('Zaman (saniye)')
plt.ylabel('Ivme (m/s^2)')
plt.title('IMU X Ivme Ölçümü vs. Gerçek X Ivmesi')
plt.grid(True)
plt.legend()
plt.show()

plt.figure(figsize=(12, 6))
plt.plot(time_true, imu_accel_y_meas, color='orange', label='IMU Y Ivme Ölçümü', alpha=0.9)
plt.plot(time_true, ay_true, color='blue', label='Gerçek Y Ivmesi', alpha=0.7)
plt.xlabel('Zaman (saniye)')
plt.ylabel('Ivme (m/s^2)')
plt.title('IMU Y Ivme Ölçümü vs. Gerçek Y Ivmesi')
plt.grid(True)
plt.legend()
plt.show()

# IMU Açısal Hız (Jiroskop) (Gerçek Açısal Hız ile Karşılaştırma)
plt.figure(figsize=(12, 6))
plt.plot(time_true, np.degrees(imu_gyro_z_meas), color='orange', label='IMU Jiroskop Ölçümü (derece/s)',
         alpha=0.9)  # Radyandan dereceye çevir
plt.plot(time_true, angular_velocity_true_deg, color='blue', label='Gerçek Açısal Hız (derece/s)', alpha=0.7)
plt.xlabel('Zaman (saniye)')
plt.ylabel('Açısal Hız (derece/s)')
plt.title('IMU Jiroskop Ölçümü vs. Gerçek Açısal Hız')
plt.grid(True)
plt.legend()
plt.show()

# GNSS Hız (Gerçek Hız ile Karşılaştırma)
# GNSS ölçümleri GNSS_UPDATE_INTERVAL'da bir yapıldığı için,
# GNSS verilerini çizerken zaman eksenini gnss_time_meas kullanmalıyız.
plt.figure(figsize=(12, 6))
gnss_speed_meas = np.sqrt(np.array(gnss_vx_meas) ** 2 + np.array(gnss_vy_meas) ** 2)
plt.plot(gnss_time_meas, gnss_speed_meas, 'r.', label='GNSS Hız Ölçümleri (Gecikmeli ve Gürültülü)', alpha=0.5,
         markersize=3)
plt.plot(time_true, speed_true, color='blue', label='Gerçek Hız (m/s)', alpha=0.7)
plt.xlabel('Zaman (saniye)')
plt.ylabel('Hız (m/s)')
plt.title('GNSS Hız Ölçümleri vs. Gerçek Hız')
plt.grid(True)
plt.legend()
plt.show()

# Simüle edilmiş sensör verilerini kaydet
np.savez('sensor_data.npz',
         imu_accel_x=np.array(imu_accel_x_meas),
         imu_accel_y=np.array(imu_accel_y_meas),
         imu_gyro_z=np.array(imu_gyro_z_meas),
         gnss_x=np.array(gnss_x_meas),
         gnss_y=np.array(gnss_y_meas),
         gnss_vx=np.array(gnss_vx_meas),
         gnss_vy=np.array(gnss_vy_meas),
         gnss_time=np.array(gnss_time_meas),
         gnss_delay_steps=GNSS_DELAY_STEPS,  # Gecikme adım sayısı da kaydedilsin
         dt=DT  # Zaman adımı da kaydedilsin
         )