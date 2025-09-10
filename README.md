# SensorFusion
Sensor Fusion and State Estimation Graduation Project.  
Vehicle motion simulation, IMU and GNSS sensor models, disturbances, calibration, and filtering (average, low-pass, complementary, Kalman/EKF) are implemented.

# Sensör Füzyonu ve Durum Kestirimi - Bitirme Projesi
Sensör Füzyonu ve Durum Kestirimi Bitirme Projesi.  
Araç hareket simülasyonu, IMU ve GNSS sensör modelleri, bozucular, kalibrasyon ve filtreleme (ortalama, alçak geçiren, tamamlayıcı, Kalman/EKF) uygulanmıştır.

---

## 🇹🇷 Türkçe Açıklama
Bu proje, **Otonom Sürüş Teknolojileri Uzmanlık Programı** kapsamında verilen bitirme ödevidir.  
Amaç, bir aracın hareketini simüle etmek, sensör (IMU ve GNSS) çıktıları üretmek, bozucular eklemek, sensör kalibrasyonu yapmak ve farklı filtreleme teknikleri ile durum kestirimi gerçekleştirmektir.

### 📌 Proje Adımları
1. **Araç hareket simülasyonu**  
   - Belirlenen rota üzerinde aracın pozisyon, hız ve yönelim değerleri Euler entegrasyonu ile çıkarıldı.

2. **Sensör modeli**  
   - IMU: ivmeölçer + jiroskop  
   - GNSS: pozisyon + hız  
   - Gürültü, bias, scale factor, gecikme (200 ms) ve çeşitli bozucular eklendi.  

3. **GNSS bozucular**  
   - 300–310 s: GNSS verisi yok  
   - 400 s: 500 m hata (1 sn)  
   - 500–505 s: sabit veri  
   - Bu senaryolar filtrelerin davranışını incelemek için eklendi.  

4. **Kalibrasyon**  
   - IMU sensöründeki bias ve scale factor hataları düzeltilmiştir.

5. **Filtreleme teknikleri**  
   - Ortalama (sensör verilerini toplayıp ikiye bölme)  
   - Alçak geçiren filtre  
   - Tamamlayıcı (complementary) filtre  
   - Kalman Filtresi / Genişletilmiş Kalman Filtresi (EKF)

6. **Sonuçların görselleştirilmesi**  
   - Pozisyon (x,y)  
   - Hız (x,y)  
   - Yönelim (ψ)  
   - İvme (IMU)  
   - Açısal hız (IMU)  
   - Kalman filtresi kovaryans ve hata grafikleri (±3σ aralığı)

---

## 🇬🇧 English Description
This project is the **Graduation Assignment** for the Autonomous Driving Technologies Specialization Program.  
The aim is to simulate vehicle motion, generate sensor outputs (IMU and GNSS), add disturbances, perform sensor calibration, and apply different filtering techniques for state estimation.

### 📌 Project Steps
1. **Vehicle motion simulation**  
   - Vehicle’s position, velocity, and orientation are obtained using Euler integration along the predefined route.

2. **Sensor modeling**  
   - IMU: accelerometer + gyroscope  
   - GNSS: position + velocity  
   - Noise, bias, scale factor, delay (200 ms), and other disturbances are added.  

3. **GNSS disturbances**  
   - 300–310 s: no GNSS data  
   - 400 s: 500 m position error (1 s)  
   - 500–505 s: frozen data  
   - These scenarios are used to analyze filter performance.  

4. **Calibration**  
   - IMU bias and scale factor errors are corrected.

5. **Filtering techniques**  
   - Averaging (simple fusion of sensor outputs)  
   - Low-pass filter  
   - Complementary filter  
   - Kalman Filter / Extended Kalman Filter (EKF)

6. **Visualization**  
   - Position (x,y)  
   - Velocity (x,y)  
   - Orientation (ψ)  
   - Acceleration (IMU)  
   - Angular velocity (IMU)  
   - Kalman filter covariance and error plots (±3σ bounds)

---

## 📂 File Structure
- `rota.py` → Vehicle motion simulation  
- `Sensor.py` → IMU and GNSS sensor data simulation  
- `Calibration.py` → Sensor calibration  
- `Filter_*.py` → Filtering methods  
   - `Filter_Average.py` → Averaging  
   - `Filter_Lowpass.py` → Low-pass filter  
   - `Filter_Complementary.py` → Complementary filter  
   - `Filter_EKF.py` → Kalman/EKF  
- `Visualization.py` → Visualization of results  
- `*.npz` → Simulation data  

## 📑 Ekler
- [Project Presentation PDF](BİTİRME_ÖDEVİ.pdf)
  
---
