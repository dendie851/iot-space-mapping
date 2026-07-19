import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
import re
import threading
import time
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Wedge, Arc

# --- KONFIGURASI MQTT ---
MQTT_BROKER = "mqtt-broker"
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/radar"

# --- INISIALISASI DATA ---
max_distance = 20
data_points = {}
current_angle = 0
current_distance = 0
auto_sweep_active = False

# --- SETUP GRAFIK ---
plt.style.use('dark_background')
fig = plt.figure(figsize=(10, 9), facecolor='#0a0a0a')
ax = fig.add_subplot(111, polar=True, facecolor='#0d1117')

ax.set_ylim(0, max_distance)
ax.set_title("RADAR", color='#00ff88', fontsize=16, pad=25,
             fontweight='bold', fontfamily='monospace')

# --- KUSTOMISASI GRID & RADIUS ---
ax.set_rgrids([5, 10, 15, 20], labels=['5cm', '10cm', '15cm', '20cm'],
              angle=45, fontsize=8, color='#00aa44', alpha=0.6)
ax.set_thetagrids(range(0, 360, 30), fontsize=7, color='#00aa44', alpha=0.5)
ax.grid(color='#00aa44', linestyle='--', linewidth=0.4, alpha=0.3)
ax.set_facecolor('#0d1117')

# --- ELEMEN VISUAL ---
# 1. Garis jarum utama (lebih tebal, glow)
line, = ax.plot([], [], color='#00ff88', linewidth=3.0, label='Pemindai',
                solid_capstyle='round')

# 2. Sinar radar (beam) - area transparan di belakang jarum
beam_patch = ax.fill([], [], color='#00ff88', alpha=0.08, zorder=1)[0]

# 3. Titik rintangan (Point Cloud — tanpa garis penghubung)
scatter = ax.scatter([], [], color='#ff3333', s=40, zorder=5, edgecolors='#ff8888',
                     linewidth=0.5, alpha=0.9)

# 5. Lingkaran titik pusat radar
center_dot = ax.scatter([0], [0], color='#00ff88', s=120, zorder=10,
                        edgecolors='#ffffff', linewidth=1.0)

# 6. Teks info
text_luas = fig.text(0.03, 0.03, "MENGHITUNG LUAS...", fontsize=11, color='#00ff88',
                     fontfamily='monospace', fontweight='bold')
text_sudut = fig.text(0.85, 0.03, "SUDUT: 0\u00b0", fontsize=11, color='#00ff88',
                      fontfamily='monospace', fontweight='bold', ha='right')
text_status = fig.text(0.5, 0.96, "", fontsize=10, color='#ffaa00',
                       ha='center', va='top', fontfamily='monospace')
text_coord = fig.text(0.03, 0.07, "JARAK: 0cm", fontsize=9, color='#00aa55',
                      fontfamily='monospace')

legend = ax.legend(loc='upper right', fontsize=8, framealpha=0.15,
                   facecolor='#0a0a0a', edgecolor='#00aa44', labelcolor='#00ff88')

# --- FUNGSI KEYBOARD ---
def on_key_press(event):
    global data_points
    if event.key == 'escape':
        data_points = {}
        print("LAYAR DIBERSIHKAN (ESC)")

# --- FUNGSI ---
def hitung_luas_ruangan(points):
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
    global data_points
    data_points[sudut] = jarak

def animate(frame):
    global current_angle, current_distance, auto_sweep_active

    # --- AUTO SWEEP ---
    if auto_sweep_active:
        period = 360
        pos = frame % period
        if pos <= 180:
            sweep_angle = pos
        else:
            sweep_angle = 360 - pos

        if data_points:
            nearest = min(data_points.keys(), key=lambda k: abs(k - sweep_angle))
            display_distance = data_points[nearest]
        else:
            display_distance = max_distance

        current_angle = sweep_angle
        current_distance = display_distance

    # --- UPDATE JARUM ---
    angle_rad = np.deg2rad(current_angle)
    line.set_data([0, angle_rad], [0, current_distance])

    # --- UPDATE BEAM (area sinar) ---
    beam_width = np.deg2rad(3)
    beam_angles = np.linspace(angle_rad - beam_width, angle_rad + beam_width, 10)
    beam_x = np.append(beam_angles, beam_angles[::-1])
    beam_y = np.append(np.full_like(beam_angles, 0),
                       np.full_like(beam_angles, current_distance))
    beam_patch.set_xy(np.column_stack([beam_x, beam_y]))

    # --- UPDATE TITIK & MAPPING ---
    sorted_angles = sorted(data_points.keys())
    if sorted_angles:
        all_angles_rad = [np.deg2rad(s) for s in sorted_angles]
        all_distances = [data_points[s] for s in sorted_angles]
        scatter.set_offsets(np.c_[all_angles_rad, all_distances])


    # --- UPDATE LUAS ---
    if current_angle in (0, 180):
        luas_cm2 = hitung_luas_ruangan(data_points)
        luas_m2 = luas_cm2 / 10000
        text_luas.set_text(f"LUAS: {luas_m2:.2f} m\u00b2")

    # --- UPDATE STATUS ---
    if auto_sweep_active:
        text_status.set_text("\u26A1 AUTO-SWEEP (MENUNGGU DATA ESP32...)")
    else:
        text_status.set_text("\u25C9 MENERIMA DATA DARI ESP32")

    text_sudut.set_text(f"SUDUT: {current_angle}\u00b0")
    text_coord.set_text(f"JARAK: {current_distance}cm")

    return (line, beam_patch, scatter,
            text_luas, text_sudut, text_status, text_coord)

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
            return
        current_angle = sudut
        current_distance = jarak
        update_data(sudut, jarak)
        last_mqtt_time = time.time()
        auto_sweep_active = False

def check_timeout():
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

# --- EVENT KEYBOARD ---
fig.canvas.mpl_connect('key_press_event', on_key_press)

# --- ANIMASI ---
ani = FuncAnimation(fig, animate, interval=30, cache_frame_data=False)
plt.show()
