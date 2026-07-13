import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
import re

# --- KONFIGURASI MQTT ---
MQTT_BROKER = "mqtt-broker" # Nama service di docker-compose (broker bisa diakses via network internal Docker)
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/radar"

# --- INISIALISASI DATA ---
max_distance = 200  # Batas jangkauan radar dalam cm (bisa disesuaikan)
data_points = {}    # Kamus untuk menyimpan data {sudut: jarak}

# --- SETUP GRAFIK RADAR POLAR ---
plt.style.use('dark_background')
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, polar=True)
ax.set_ylim(0, max_distance)

# Membuat plot awal untuk jarum radar (garis pemindai) dan titik rintangan
line, = ax.plot([], [], color='#00ff00', linewidth=2)  # Garis hijau ala radar
scatter = ax.scatter([], [], color='#ff0000', s=15)    # Titik merah rintangan
text_luas = fig.text(0.02, 0.02, "Menghitung Luas...", fontsize=12, color='white')

def hitung_luas_ruangan(points):
    """Menghitung luas poligon ruangan menggunakan Algoritma Shoelace"""
    if len(points) < 3:
        return 0
    
    # Urutkan sudut agar poligon terbentuk berurutan
    sudut_urut = sorted(points.keys())
    
    # Konversi semua koordinat Polar ke Kartesian (X, Y)
    x_coords = []
    y_coords = []
    for s in sudut_urut:
        rad = np.deg2rad(s)
        r = points[s]
        x_coords.append(r * np.cos(rad))
        y_coords.append(r * np.sin(rad))
    
    # Rumus Shoelace Formula
    x = np.array(x_coords)
    y = np.array(y_coords)
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

def on_message(client, userdata, msg):
    global data_points
    payload = msg.payload.decode('utf-8')
    
    # Validasi format data "sudut,jarak" (contoh: "90,150")
    match = re.match(r"^(\d+),(\d+)$", payload)
    if match:
        sudut = int(match.group(1))
        jarak = int(match.group(2))
        
        # Batasi jarak maksimum agar grafik tidak rusak
        if jarak > max_distance or jarak == 0:
            jarak = max_distance
            
        # Simpan/update data pada sudut tersebut
        data_points[sudut] = jarak
        
        # --- UPDATE VISUALISASI ---
        # 1. Update Garis Jarum Radar
        line.set_data([0, np.deg2rad(sudut)], [0, jarak])
        
        # 2. Update Semua Titik Rintangan yang Sudah Terkumpul
        all_angles = [np.deg2rad(s) for s in data_points.keys()]
        all_distances = list(data_points.values())
        scatter.set_offsets(np.c_[all_angles, all_distances])
        
        # 3. Hitung dan Tampilkan Luas Ruangan (Setiap Putaran Penuh)
        if sudut == 180 or sudut == 0:
            luas_cm2 = hitung_luas_ruangan(data_points)
            luas_m2 = luas_cm2 / 10000  # Konversi cm² ke Meter Persegi
            text_luas.set_text(f"Estimasi Luas Ruangan: {luas_m2:.2f} m²")
            
        plt.draw()
        plt.pause(0.001)

# --- START KONEKSI MQTT ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.subscribe(MQTT_TOPIC)

print(f"Mendengarkan data radar di topik '{MQTT_TOPIC}'...")
client.loop_start()
plt.show()  # Membuka jendela grafik jendela radar