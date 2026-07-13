import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
import re
import threading
import time
from matplotlib.animation import FuncAnimation

# --- KONFIGURASI MQTT ---
MQTT_BROKER = "mqtt-broker"
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/radar"

# --- INISIALISASI DATA ---
max_distance = 200
data_points = {}             # {sudut: jarak} dari ESP32
current_angle = 0
current_distance = 0
sweep_angle = 0              # sudut untuk gerakan putar otomatis
auto_sweep_active = False    # True jika tidak ada data MQTT masuk dalam 2 detik

# --- SETUP GRAFIK ---
plt.style.use('dark_background')
fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, polar=True)
ax.set_ylim(0, max_distance)
ax.set_title("IoT Space Mapping", color='#00ff00', fontsize=14, pad=20)
ax.grid(color='#003300', linestyle='--', linewidth=0.5)

line, = ax.plot([], [], color='#00ff00', linewidth=2.5, label='Pemindai')
polygon_line, = ax.plot([], [], color='#00ffff', linewidth=1.8, alpha=0.8, label='Mapping Dinding')
scatter = ax.scatter([], [], color='#ff0000', s=25, zorder=5)
text_luas = fig.text(0.02, 0.02, "Menghitung Luas...", fontsize=12, color='white')
text_sudut = fig.text(0.9, 0.02, "Sudut: 0\u00b0", fontsize=10, color='#00ff00', ha='right')
text_status = fig.text(0.5, 0.97, "", fontsize=10, color='#ffaa00', ha='center', va='top', style='italic')
legend = ax.legend(loc='upper right', fontsize=9, framealpha=0.3)

def hitung_luas_ruangan(points):
    """Shoelace Formula."""
    if len(points) < 3:
        return 0
    sudut_urut = sorted(points.keys())
    x, y = [], []
    for s in sudut_urut:
        rad = np.deg2rad(s)
        r = points[s]
        x.append(r * np.cos(rad))
        y.append(r * np.sin(rad))
    x = np.array(x)
    y = np.array(y)
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

def update_data(sudut, jarak):
    """Simpan data dari MQTT."""
    global data_points
    data_points[sudut] = jarak

def animate(frame):
    """Callback animasi - update grafik setiap frame."""
    global current_angle, current_distance, sweep_angle, auto_sweep_active

    # --- GERAKAN JARUM RADAR ---
    if auto_sweep_active:
        # Putar otomatis maju-mundur (sweep)
        sweep_angle += 1
        if sweep_angle >= 180:
            sweep_angle = 180
        # Arah balik ditangani dengan membalik arah di luar
        # Kita simulasikan sweep 0->180->0
        if sweep_angle >= 180:
            # Tunggu beberapa frame lalu balik
            pass
        
        # Simple triangle wave: 0,1,2,...,179,180,179,...,1,0,1,...
        # Gunakan frame counter
        period = 360
        pos = frame % period
        if pos <= 180:
            sweep_angle = pos
        else:
            sweep_angle = 360 - pos
        
        display_angle = sweep_angle
        # Jarak untuk gerakan otomatis: gunakan data terdekat dari data_points atau default
        if data_points:
            # Cari data terdekat dengan sudut saat ini
            nearest = min(data_points.keys(), key=lambda k: abs(k - display_angle))
            display_distance = data_points[nearest]
        else:
            # Jika belum ada data sama sekali, jarum memanjang penuh ke batas max
            display_distance = max_distance
        
        line.set_data([0, np.deg2rad(display_angle)], [0, display_distance])
        current_angle = display_angle
        current_distance = display_distance
    else:
        # Mode MQTT: jarum mengikuti data dari ESP32
        line.set_data([0, np.deg2rad(current_angle)], [0, current_distance])

    # --- GAMBAR TITIK & MAPPING ---
    sorted_angles = sorted(data_points.keys())
    if sorted_angles:
        all_angles_rad = [np.deg2rad(s) for s in sorted_angles]
        all_distances = [data_points[s] for s in sorted_angles]
        scatter.set_offsets(np.c_[all_angles_rad, all_distances])

        if len(all_angles_rad) > 1:
            # Tutup poligon mapping
            angles_closed = all_angles_rad + [all_angles_rad[0]]
            dists_closed = all_distances + [all_distances[0]]
            polygon_line.set_data(angles_closed, dists_closed)

    # --- HITUNG LUAS ---
    if current_angle in (0, 180):
        luas_cm2 = hitung_luas_ruangan(data_points)
        luas_m2 = luas_cm2 / 10000
        text_luas.set_text(f"Estimasi Luas Ruangan: {luas_m2:.2f} m\u00b2")

    # --- STATUS ---
    if auto_sweep_active:
        text_status.set_text("\u26A1 Auto-Sweep (menunggu data ESP32...)")
    else:
        text_status.set_text("\u25C9 Menerima data dari ESP32")

    text_sudut.set_text(f"Sudut: {current_angle}\u00b0")
    return line, scatter, polygon_line, text_luas, text_sudut, text_status

# --- CALLBACK MQTT ---
last_mqtt_time = time.time()

def on_message(client, userdata, msg):
    global current_angle, current_distance, auto_sweep_active, last_mqtt_time
    payload = msg.payload.decode('utf-8')
    match = re.match(r"^(\d+),(\d+)$", payload)
    if match:
        sudut = int(match.group(1))
        jarak = int(match.group(2))
        if jarak > max_distance or jarak <= 0:
            jarak = max_distance
        current_angle = sudut
        current_distance = jarak
        update_data(sudut, jarak)
        last_mqtt_time = time.time()
        auto_sweep_active = False  # Ada data MQTT, matikan auto-sweep

def check_timeout():
    """Thread cek apakah MQTT timeout, jika ya aktifkan auto-sweep."""
    global auto_sweep_active
    while True:
        time.sleep(0.5)
        if time.time() - last_mqtt_time > 2.0:
            auto_sweep_active = True

# --- START MQTT ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.subscribe(MQTT_TOPIC)
print(f"Mendengarkan data radar di topik '{MQTT_TOPIC}'...")
client.loop_start()

# --- THREAD TIMEOUT ---
timeout_thread = threading.Thread(target=check_timeout, daemon=True)
timeout_thread.start()

# --- ANIMASI ---
ani = FuncAnimation(fig, animate, interval=30, cache_frame_data=False)
plt.show()