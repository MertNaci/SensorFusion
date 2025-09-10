import numpy as np
import matplotlib.pyplot as plt

# - PARAMETRELERİN TANIMLANMASI -
# Sistemin çalışma frekansı
DT = 1 / 50  #50 Hz çalışacak şekilde örnekleme zamanı
TOTAL_TIME = 1000
NUM_STEPS = int(TOTAL_TIME / DT)  # Toplam adım sayısı 1000 x 0.2 = 50000 adım

# Başlangıç Koşulları
x = 0.0  # metre
y = 0.0  # metre
vx = 0.0  # m/s
vy = 0.0  # m/s
yaw = np.radians(90.0)  # Radyan (90 derece Kuzey'e bakıyor)

# Verileri depolamak için listeler
time_data = []
x_data = []
y_data = []
vx_data = []
vy_data = []
yaw_data = []
speed_data = []  # Toplam hız (büyüklüğü)

# - YARDIMCI FONKSİYONLAR -
def kmh_to_ms(kmh):
    return kmh / 3.6

def ms_to_kmh(ms):
    return ms * 3.6

# - ROTA SİMÜLASYONU DÖNGÜSÜ -
current_time = 0.0 # Zamanı takip etmek için bir değişken

for i in range(NUM_STEPS):
    # Verileri kaydet
    time_data.append(current_time)
    x_data.append(x)
    y_data.append(y)
    vx_data.append(vx)
    vy_data.append(vy)
    yaw_data.append(np.degrees(yaw))  # Derece olarak kaydet
    speed_data.append(np.sqrt(vx ** 2 + vy ** 2))  # Hızın büyüklüğü

    # Mevcut hızın büyüklüğü (km/saat)
    current_speed_kmh = ms_to_kmh(np.sqrt(vx ** 2 + vy ** 2))

    # - ROTA AŞAMALARI -
    # Aşama 1: Başlangıçtan 10 saniyede 100 km/sa hıza çıkış
    if current_time <= 10.0:
        target_speed_ms = kmh_to_ms(100)
        # Hızlanma ivmesi
        ax = (target_speed_ms / 10.0) * np.cos(yaw)  # ivme = hedef hız / süre
        ay = (target_speed_ms / 10.0) * np.sin(yaw)
        angular_velocity = 0.0  # Açısal hız

    # Aşama 2: Sonraki 10 saniye boyunca 45 derece sağa dönüş
    elif 10.0 < current_time <= 20.0:  # 10.0 saniyeden sonraki 10 saniye
        target_speed_ms = kmh_to_ms(100)  # Hız sabit kalacak
        ax = 0.0
        ay = 0.0
        # Açısal hız: 45 derece / 10 saniye
        angular_velocity = np.radians(-45) / 10.0  # Sağa dönüş negatif açısal hız

    # Aşama 3: 200. saniyede hızını 30 saniyede 150 km/s hıza çıkaracak
    elif 200.0 < current_time <= 230.0:  # 200.0 ile 230.0 saniye arası
        start_speed_kmh = ms_to_kmh(np.sqrt(vx ** 2 + vy ** 2))  # Mevcut hız
        target_speed_kmh = 150
        # Hızlanma ivmesi
        # delta_v_kmh = target_speed_kmh - start_speed_kmh
        avg_accel_magnitude = (kmh_to_ms(target_speed_kmh) - kmh_to_ms(current_speed_kmh)) / 30.0
        ax = avg_accel_magnitude * np.cos(yaw)
        ay = avg_accel_magnitude * np.sin(yaw)
        angular_velocity = 0.0  # Düz gidiyor

    # Aşama 4: Sonraki 30 saniye boyunca 180 derece dönüş yapacak ve hızını 150 km/s'den 100 km/s'ye indirecek
    elif 230.0 < current_time <= 260.0:  # 230.0 ile 260.0 saniye arası
        start_speed_kmh = 150
        end_speed_kmh = 100
        avg_accel_magnitude = (kmh_to_ms(end_speed_kmh) - kmh_to_ms(start_speed_kmh)) / 30.0
        ax = avg_accel_magnitude * np.cos(yaw)
        ay = avg_accel_magnitude * np.sin(yaw)
        # Açısal hız: 180 derece / 30 saniye
        #(yön belirtilmemiş ben de sağa doğru döndüğünü varsaydım bu yüzden açısal hızı negatif aldım)
        angular_velocity = np.radians(-180) / 30.0  # Sağa dönüş

    # Aşama 5: 500. saniyede hızını 10 saniyede 50 km/s hıza indirecek
    elif 500.0 < current_time <= 510.0:  # 500.0 ile 510.0 saniye arası
        start_speed_kmh = ms_to_kmh(np.sqrt(vx ** 2 + vy ** 2))
        target_speed_kmh = 50
        avg_accel_magnitude = (kmh_to_ms(target_speed_kmh) - kmh_to_ms(start_speed_kmh)) / 10.0
        ax = avg_accel_magnitude * np.cos(yaw)
        ay = avg_accel_magnitude * np.sin(yaw)
        angular_velocity = 0.0  # Düz gidiyor

    # Aşama 6: 800. saniyede 30 saniye boyunca 90 derece sağa dönerken hızını 100 km/saate çıkaracak
    elif 800.0 < current_time <= 830.0:  # 800.0 ile 830.0 saniye arası
        start_speed_kmh = ms_to_kmh(np.sqrt(vx ** 2 + vy ** 2))
        target_speed_kmh = 100
        avg_accel_magnitude = (kmh_to_ms(target_speed_kmh) - kmh_to_ms(start_speed_kmh)) / 30.0
        ax = avg_accel_magnitude * np.cos(yaw)
        ay = avg_accel_magnitude * np.sin(yaw)

        # Açısal hız: 90 derece / 30 saniye
        angular_velocity = np.radians(-90) / 30.0  # Sağa dönüş

    # Aşama 7: 1000. saniyeye son 10 saniye kaldığında araç duracak
    elif TOTAL_TIME - 10.0 <= current_time < TOTAL_TIME:  # 990.0 ile 1000.0 saniye arası
        start_speed_ms = np.sqrt(vx ** 2 + vy ** 2)
        # Hedef hız 0, süre 10 saniye
        avg_decel_magnitude = (0 - start_speed_ms) / 10.0
        ax = avg_decel_magnitude * np.cos(yaw)
        ay = avg_decel_magnitude * np.sin(yaw)
        angular_velocity = 0.0

    # Aşama 8: Diğer zamanlarda hız ve yönelim sabit kalacak (yani ivme ve açısal hız 0)
    else:
        ax = 0.0
        ay = 0.0
        angular_velocity = 0.0
        # Hızın neredeyse tam 0 olması ve aracın durması için (float sayılarda sıkıntı olabiliyor)
        if current_time >= TOTAL_TIME - 10.0 and np.sqrt(vx ** 2 + vy ** 2) < 0.1:  # Neredeyse durduysa
            ax = 0.0
            ay = 0.0
            vx = 0.0
            vy = 0.0

    # Euler Integrasyonu
    # Hız güncelleme
    vx += ax * DT
    vy += ay * DT

    # Pozisyon güncelleme
    x += vx * DT
    y += vy * DT

    # Yönelim (yaw) güncelleme
    yaw += angular_velocity * DT

    # Zamanı ilerlet
    current_time += DT

# Son verileri kaydet (döngü bittikten sonraki son hali)
time_data.append(current_time)
x_data.append(x)
y_data.append(y)
vx_data.append(vx)
vy_data.append(vy)
yaw_data.append(np.degrees(yaw))
speed_data.append(np.sqrt(vx ** 2 + vy ** 2))

# - SONUÇLARI GÖRSELLEŞTİRME (ROTA ÇİZİMİ) -
plt.figure(figsize=(10, 8))
plt.plot(x_data, y_data, label='Gerçek Rota')
plt.xlabel('X Pozisyonu (metre)')
plt.ylabel('Y Pozisyonu (metre)')
plt.title('Aracın Gerçek Rotası')
plt.grid(True)
plt.axis('equal')  # Eksen oranlarını eşit tutar
plt.legend()
plt.show()

# Hız grafiği
plt.figure(figsize=(12, 6))
plt.plot(time_data, speed_data, label='Gerçek Hız (m/s)')
plt.xlabel('Zaman (saniye)')
plt.ylabel('Hız (m/s)')
plt.title('Aracın Gerçek Hızı')
plt.grid(True)
plt.legend()
plt.show()

# Yönelim grafiği
plt.figure(figsize=(12, 6))
plt.plot(time_data, yaw_data, label='Gerçek Yönelim (derece)')
plt.xlabel('Zaman (saniye)')
plt.ylabel('Yönelim (derece)')
plt.title('Aracın Gerçek Yönelimi')
plt.grid(True)
plt.legend()
plt.show()

# Bu verileri daha sonra kullanmak üzere kaydedelim
np.savez('true_state_data.npz',
        time=np.array(time_data),
        x=np.array(x_data),
        y=np.array(y_data),
        vx=np.array(vx_data),
        vy=np.array(vy_data),
        yaw=np.array(yaw_data),
        speed=np.array(speed_data))
