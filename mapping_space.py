#!/usr/bin/env python3
"""
================================================================================
 MAPPING_SPACE.py — Mobile 2D Grid Mapping dengan Sensor Ultrasonik + Servo
================================================================================
Sistem pemetaan ruangan 2-Dimensi (kartesian) secara mobile / berjalan.
- Posisi alat (robot) bisa digerakkan dengan keyboard (WASD + Q/E).
- Sensor ultrasonik (HC-SR04) + servo menscan dinding di sekitar.
- Titik-titik dinding tersimpan PERMANEN (point cloud) dalam koordinat global.
- Mode SIMULASI (tanpa hardware) sudah built-in.
- Mode REAL (terhubung ke ESP32 via MQTT) juga tersedia.

CARA MENJALANKAN:
    # Mode simulasi (standalone, tanpa hardware):
    python mapping_space.py

    # Mode real (terhubung ke MQTT broker + ESP32):
    python mapping_space.py --real

KONTROL KEYBOARD (Mode Simulasi):
    W / S    : Maju / Mundur (50cm per tekan)
    A / D    : Geser kiri / kanan (50cm per tekan)
    Q / E    : Putar heading (15 derajat per tekan)
    R        : Reset posisi ke (0, 0)
    SPACE    : Scan manual (satu kali pengukuran)
    +/-      : Zoom in/out
    ESC      : Keluar

LIBRARY YANG DIBUTUHKAN:
    pip install pygame numpy paho-mqtt

INTEGRASI HARDWARE (ESP32 + HC-SR04 + Servo):
    ESP32 mengirim data via MQTT dengan format: "sudut,jarak"
        contoh: "45,120"  (sudut=45°, jarak=120cm)
    Pastikan MQTT_BROKER diatur ke IP laptop/PC Anda.

================================================================================
"""

import pygame
import numpy as np
import math
import sys
import os
import time
import random
import argparse
import threading
import re

# --- MQTT (opsional, hanya jika hardware terhubung) ---
MQTT_AVAILABLE = False
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    pass

# ============================================================================
# KONFIGURASI
# ============================================================================

# --- Jendela & Tampilan ---
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 850
MAP_AREA_HEIGHT = WINDOW_HEIGHT - 50  # 50px untuk info bar
MAP_CENTER_X = WINDOW_WIDTH // 2
MAP_CENTER_Y = MAP_AREA_HEIGHT // 2

# --- Skala peta (cm ke pixel) ---
SCALE = 2.0  # 1 cm = 2 pixel (default)

# --- Sensor ---
MAX_SENSOR_CM = 500      # Jarak maksimum sensor (cm)
SERVO_STEP = 2           # Step sudut servo (derajat)
SERVO_MIN = 0            # Sudut minimum servo
SERVO_MAX = 180          # Sudut maksimum servo
SERVO_DELAY = 0.05       # Delay antar step (detik)

# --- Grid ---
GRID_SPACING_CM = 100    # Garis grid setiap 1 meter (100 cm)

# --- Warna ---
COLOR_BG            = (8, 8, 16)
COLOR_GRID          = (25, 35, 50)
COLOR_GRID_MAJOR    = (35, 50, 70)
COLOR_GRID_LABEL    = (60, 80, 100)
COLOR_ROBOT         = (0, 255, 100)
COLOR_ROBOT_EDGE    = (180, 255, 200)
COLOR_HEADING_LINE  = (0, 255, 150)
COLOR_SCANNER       = (255, 220, 50)
COLOR_BEAM          = (0, 255, 80)
COLOR_WALL_POINT    = (255, 50, 50)
COLOR_WALL_GLOW     = (255, 90, 90)
COLOR_TEXT          = (0, 255, 150)
COLOR_TEXT_DIM      = (80, 140, 100)
COLOR_STATUS        = (255, 200, 50)
COLOR_INFO_BG       = (15, 15, 30)

# ============================================================================
# STATE GLOBAL
# ============================================================================

# Posisi robot (koordinat global, cm)
robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0  # radian (0 = menghadap kanan / east)

# Data sensor
current_servo_angle = 90.0   # derajat (relatif terhadap robot)
current_distance = MAX_SENSOR_CM

# Point cloud dinding (koordinat global, cm)
wall_points = []  # list of (x, y) tuples
MAX_POINTS = 100000

# Status
simulation_mode = True
auto_sweep = True
servo_sweep_dir = 1  # 1 = naik, -1 = turun
last_scan_time = 0
scan_counter = 0
fps = 0

# MQTT
mqtt_connected = False
last_mqtt_time = 0

# ============================================================================
# RUANGAN VIRTUAL UNTUK SIMULASI
# ============================================================================
# Dinding ruangan sebagai segmen garis (x1, y1, x2, y2) dalam cm
virtual_walls = [
    # --- Dinding luar (8m x 6m) ---
    (-400, -300,  400, -300),   # Utara
    ( 400, -300,  400,  300),   # Timur
    ( 400,  300, -400,  300),   # Selatan
    (-400,  300, -400, -300),   # Barat

    # --- Partisi dalam (dinding tambahan) ---
    (-200, -300, -200,    0),   # Partisi vertikal kiri
    ( 200,    0,  200,  300),   # Partisi vertikal kanan

    # --- Objek persegi (kolom) ---
    ( -80,  -80,   80,  -80),
    (  80,  -80,   80,   80),
    (  80,   80,  -80,   80),
    ( -80,   80,  -80,  -80),

    # --- Dinding diagonal ---
    (-300,  100, -150,  250),

    # --- Penghalang kecil ---
    ( 250, -200,  350, -200),
    ( 350, -200,  350, -100),
]


def line_intersection(p1, p2, p3, p4):
    """
    Hitung titik potong antara dua segmen garis.
    p1-p2 = sinar dari sensor
    p3-p4 = segmen dinding
    Returns (x, y) atau None jika tidak berpotongan.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

    if 0 <= t <= 1 and 0 <= u <= 1:
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None


def simulate_sensor_ray():
    """
    SIMULASI: Raycasting dari posisi robot.
    Mengembalikan (jarak_ke_dinding, titik_global) atau (MAX, None).
    """
    global robot_x, robot_y, robot_yaw, current_servo_angle

    # Arah sinar = heading robot + sudut servo (relatif)
    ray_angle = robot_yaw + math.radians(current_servo_angle)
    ray_start = (robot_x, robot_y)
    ray_end = (
        robot_x + math.cos(ray_angle) * MAX_SENSOR_CM,
        robot_y + math.sin(ray_angle) * MAX_SENSOR_CM
    )

    min_dist = MAX_SENSOR_CM
    hit_point = None

    for wall in virtual_walls:
        p3 = (wall[0], wall[1])
        p4 = (wall[2], wall[3])
        hit = line_intersection(ray_start, ray_end, p3, p4)
        if hit:
            dist = math.sqrt((hit[0] - robot_x)**2 + (hit[1] - robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                hit_point = hit

    return min_dist, hit_point


def convert_to_global(distance_cm, servo_angle_deg):
    """
    KONVERSI KOORDINAT GLOBAL (Rumus dari spesifikasi):
      X_relatif = Jarak_Sensor * cos(Sudut_Servo)
      Y_relatif = Jarak_Sensor * sin(Sudut_Servo)
      X_dinding = X_pos + X_relatif
      Y_dinding = Y_pos + Y_relatif

    Catatan: Sudut servo (0° = depan robot) dikonversi ke arah global
    dengan menambahkan heading robot (yaw).
    """
    global robot_x, robot_y, robot_yaw

    # Sudut absolut = heading robot + sudut servo
    abs_angle = robot_yaw + math.radians(servo_angle_deg)

    x_rel = distance_cm * math.cos(abs_angle)
    y_rel = distance_cm * math.sin(abs_angle)

    x_global = robot_x + x_rel
    y_global = robot_y + y_rel

    return x_global, y_global


def add_wall_point(x, y):
    """Tambahkan titik dinding ke point cloud (dengan deduplikasi jarak)."""
    global wall_points

    # Cek apakah sudah ada titik di sekitar sini (radius 2 cm)
    for px, py in wall_points:
        if (px - x)**2 + (py - y)**2 < 4.0:  # jarak kuadrat < 4 => < 2cm
            return False

    wall_points.append((x, y))

    # Batasi ukuran point cloud untuk performa
    if len(wall_points) > MAX_POINTS:
        # Sampling: ambil setiap titik ke-2
        wall_points[:] = wall_points[::2]

    return True


def do_scan():
    """
    Lakukan satu pengukuran sensor.
    - Simulasi: raycasting ke dinding virtual
    - Real: data sudah diterima dari MQTT
    """
    global current_distance, simulation_mode

    if simulation_mode:
        dist, hit = simulate_sensor_ray()
        current_distance = dist

        if hit and dist < MAX_SENSOR_CM - 5:
            # Tambah noise realistis (±2 cm)
            nx = hit[0] + random.gauss(0, 1.5)
            ny = hit[1] + random.gauss(0, 1.5)
            add_wall_point(nx, ny)

        return dist
    return current_distance


def update_auto_sweep():
    """Auto-sweep servo: bergerak maju-mundur antara 0-180°."""
    global current_servo_angle, servo_sweep_dir, last_scan_time

    now = time.time()
    if now - last_scan_time < SERVO_DELAY:
        return

    # Update sudut servo
    current_servo_angle += SERVO_STEP * servo_sweep_dir

    if current_servo_angle >= SERVO_MAX:
        current_servo_angle = SERVO_MAX
        servo_sweep_dir = -1
    elif current_servo_angle <= SERVO_MIN:
        current_servo_angle = SERVO_MIN
        servo_sweep_dir = 1

    # Lakukan scan pada sudut baru
    do_scan()
    last_scan_time = now


# ============================================================================
# RENDERER PYGAME
# ============================================================================

def world_to_screen(wx, wy, scale):
    """Konversi koordinat dunia (cm) ke pixel layar."""
    sx = MAP_CENTER_X + wx * scale
    sy = MAP_CENTER_Y - wy * scale  # Y dibalik (screen: down = positif)
    return int(sx), int(sy)


def draw_grid(surface, scale, offset_x, offset_y):
    """
    Gambar grid kartesian 2D.
    Grid minor tiap 1m, grid mayor tiap 5m.
    """
    # Hitung rentang grid yang terlihat
    visible_cm_x = WINDOW_WIDTH / scale
    visible_cm_y = MAP_AREA_HEIGHT / scale

    # Konversi offset pixel ke cm
    offset_cm_x = (MAP_CENTER_X - offset_x) / scale  # HATI-HATI: ini tidak
    offset_cm_y = (MAP_CENTER_Y - offset_y) / scale  # digunakan sederhana

    # Gambar grid dari posisi robot
    start_x = int((robot_x - visible_cm_x / 2) // GRID_SPACING_CM) * GRID_SPACING_CM
    start_y = int((robot_y - visible_cm_y / 2) // GRID_SPACING_CM) * GRID_SPACING_CM
    end_x = int(robot_x + visible_cm_x / 2) + GRID_SPACING_CM
    end_y = int(robot_y + visible_cm_y / 2) + GRID_SPACING_CM

    font = pygame.font.SysFont('monospace', 10, bold=True)

    # Grid vertikal
    for gx in range(start_x, end_x, GRID_SPACING_CM):
        sx, _ = world_to_screen(gx, 0, scale)
        if 0 <= sx <= WINDOW_WIDTH:
            # Garis grid
            is_major = (gx % (GRID_SPACING_CM * 5) == 0)
            color = COLOR_GRID_MAJOR if is_major else COLOR_GRID
            pygame.draw.line(surface, color, (sx, 0), (sx, MAP_AREA_HEIGHT), 1 if is_major else 1)

            # Label (setiap 2m)
            if gx % 200 == 0 and gx != 0:
                label = font.render(f"{gx//100:.0f}m", True, COLOR_GRID_LABEL)
                surface.blit(label, (sx + 3, MAP_AREA_HEIGHT - 18))

    # Grid horizontal
    for gy in range(start_y, end_y, GRID_SPACING_CM):
        _, sy = world_to_screen(0, gy, scale)
        if 0 <= sy <= MAP_AREA_HEIGHT:
            is_major = (gy % (GRID_SPACING_CM * 5) == 0)
            color = COLOR_GRID_MAJOR if is_major else COLOR_GRID
            pygame.draw.line(surface, color, (0, sy), (WINDOW_WIDTH, sy), 1 if is_major else 1)

            # Label (setiap 2m)
            if gy % 200 == 0 and gy != 0:
                label = font.render(f"{gy//100:.0f}m", True, COLOR_GRID_LABEL)
                surface.blit(label, (3, sy - 12))


def draw_robot(surface, scale):
    """Gambar robot (titik hijau) + garis heading + area scanner."""
    sx, sy = world_to_screen(robot_x, robot_y, scale)

    # --- Lingkaran luar (glow) ---
    pygame.draw.circle(surface, (0, 255, 100, 30), (sx, sy), int(15 * scale / 2), 3)

    # --- Tubuh robot (lingkaran) ---
    radius = max(8, int(8 * scale / 2))
    pygame.draw.circle(surface, COLOR_ROBOT, (sx, sy), radius)
    pygame.draw.circle(surface, COLOR_ROBOT_EDGE, (sx, sy), radius, 2)

    # --- Garis heading (arah hadap) ---
    heading_len = int(30 * scale)
    hx = sx + int(math.cos(robot_yaw) * heading_len)
    hy = sy - int(math.sin(robot_yaw) * heading_len)
    pygame.draw.line(surface, COLOR_HEADING_LINE, (sx, sy), (hx, hy), 3)
    # Ujung panah
    arrow_size = 6
    a1 = (hx - int(math.cos(robot_yaw - 0.5) * arrow_size),
          hy + int(math.sin(robot_yaw - 0.5) * arrow_size))
    a2 = (hx - int(math.cos(robot_yaw + 0.5) * arrow_size),
          hy + int(math.sin(robot_yaw + 0.5) * arrow_size))
    pygame.draw.polygon(surface, COLOR_HEADING_LINE, [(hx, hy), a1, a2])

    # --- Garis scanner (arah servo saat ini) ---
    scan_angle = robot_yaw + math.radians(current_servo_angle)
    scan_len = int(current_distance * scale)
    if scan_len > 5:
        ssx = sx + int(math.cos(scan_angle) * scan_len)
        ssy = sy - int(math.sin(scan_angle) * scan_len)
        pygame.draw.line(surface, COLOR_SCANNER, (sx, sy), (ssx, ssy), 2)

        # Titik ujung scanner (titik dinding yang terdeteksi)
        pygame.draw.circle(surface, COLOR_WALL_POINT, (ssx, ssy), 4)
        pygame.draw.circle(surface, COLOR_WALL_GLOW, (ssx, ssy), 6, 1)


def draw_wall_points(surface, scale):
    """Gambar semua titik dinding yang terdeteksi (point cloud permanen)."""
    # Optimasi: gambar dengan batch
    if not wall_points:
        return

    # Untuk performa, gambar sebagai pixel atau lingkaran kecil
    points_to_draw = []
    glow_points = []

    for wx, wy in wall_points:
        sx, sy = world_to_screen(wx, wy, scale)
        if 0 <= sx <= WINDOW_WIDTH and 0 <= sy <= MAP_AREA_HEIGHT:
            points_to_draw.append((sx, sy))
            glow_points.append((sx, sy))

    # Gambar glow (lingkaran besar tipis) — hanya untuk subset untuk performa
    step = max(1, len(glow_points) // 1000)
    for i in range(0, len(glow_points), step):
        sx, sy = glow_points[i]
        pygame.draw.circle(surface, (255, 50, 50, 80), (sx, sy), 3, 1)

    # Gambar titik utama (dengan batch)
    for sx, sy in points_to_draw:
        surface.set_at((sx, sy), COLOR_WALL_POINT)
        # Titik yang lebih jelas
        if len(points_to_draw) < 5000:
            pygame.draw.circle(surface, (255, 60, 60), (sx, sy), 1)


def draw_info_bar(surface, scale):
    """Info bar di bagian bawah layar."""
    # Background
    pygame.draw.rect(surface, COLOR_INFO_BG, (0, MAP_AREA_HEIGHT, WINDOW_WIDTH, 50))
    pygame.draw.line(surface, COLOR_GRID_MAJOR, (0, MAP_AREA_HEIGHT),
                     (WINDOW_WIDTH, MAP_AREA_HEIGHT), 2)

    font = pygame.font.SysFont('monospace', 12, bold=True)
    font_small = pygame.font.SysFont('monospace', 10)

    y = MAP_AREA_HEIGHT + 8

    # Posisi robot
    text_pos = font.render(
        f"POS: ({robot_x:.0f}, {robot_y:.0f}) cm | HEADING: {math.degrees(robot_yaw):.0f}°",
        True, COLOR_TEXT
    )
    surface.blit(text_pos, (10, y))

    # Data sensor
    text_sensor = font.render(
        f"SERVO: {current_servo_angle:.0f}° | JARAK: {current_distance:.0f} cm",
        True, COLOR_TEXT
    )
    surface.blit(text_sensor, (10, y + 18))

    # Mode & status — bedakan antara SIMULASI aktif vs STANDBY menunggu data
    if not simulation_mode:
        mode_str = "REAL (MQTT)"
        mode_color = (50, 200, 255)
    elif mqtt_connected and len(wall_points) == 0:
        mode_str = "STANDBY (Menunggu ESP32...)"
        mode_color = (255, 200, 50)
    else:
        mode_str = "SIMULASI"
        mode_color = (255, 200, 50)
    text_mode = font.render(f"MODE: {mode_str}", True, mode_color)
    surface.blit(text_mode, (WINDOW_WIDTH - 250, y))

    # Jumlah titik
    text_pts = font_small.render(
        f"WALL POINTS: {len(wall_points)} | FPS: {fps:.0f}",
        True, COLOR_TEXT_DIM
    )
    surface.blit(text_pts, (WINDOW_WIDTH - 250, y + 18))

    # Skala
    text_scale = font_small.render(
        f"ZOOM: {scale:.1f}x | [W/A/S/D] Gerak | [Q/E] Rotasi | [+/-] Zoom | [ESC] Keluar",
        True, COLOR_TEXT_DIM
    )
    surface.blit(text_scale, (WINDOW_WIDTH // 2 - 250, y + 18))


def draw_minimap(surface):
    """Minimap kecil di pojok kanan atas."""
    minimap_size = 120
    minimap_x = WINDOW_WIDTH - minimap_size - 15
    minimap_y = 15
    mm_center = minimap_size // 2

    # Background
    pygame.draw.rect(surface, (5, 5, 15), (minimap_x, minimap_y, minimap_size, minimap_size))
    pygame.draw.rect(surface, COLOR_GRID_MAJOR, (minimap_x, minimap_y, minimap_size, minimap_size), 1)

    # Skala minimap (1:20 dari peta utama)
    mm_scale = scale * 0.15

    # Gambar wall points di minimap (subset)
    step = max(1, len(wall_points) // 200)
    for i in range(0, len(wall_points), step):
        wx, wy = wall_points[i]
        sx = minimap_x + mm_center + (wx - robot_x) * mm_scale
        sy = minimap_y + mm_center - (wy - robot_y) * mm_scale
        if 0 <= sx <= minimap_x + minimap_size and 0 <= sy <= minimap_y + minimap_size:
            surface.set_at((int(sx), int(sy)), (180, 40, 40))

    # Posisi robot di minimap
    rx = minimap_x + mm_center
    ry = minimap_y + mm_center
    pygame.draw.circle(surface, COLOR_ROBOT, (rx, ry), 4)
    pygame.draw.circle(surface, COLOR_ROBOT_EDGE, (rx, ry), 4, 1)

    # Garis heading di minimap
    hx = rx + int(math.cos(robot_yaw) * 12)
    hy = ry - int(math.sin(robot_yaw) * 12)
    pygame.draw.line(surface, COLOR_HEADING_LINE, (rx, ry), (hx, hy), 2)

    # Label
    font = pygame.font.SysFont('monospace', 8)
    label = font.render("MINIMAP", True, COLOR_TEXT_DIM)
    surface.blit(label, (minimap_x + 5, minimap_y + 3))


def draw_status_overlay(surface):
    """Overlay status di tengah layar (jika ada event penting)."""
    pass  # Cadangan untuk notifikasi


# ============================================================================
# MQTT HANDLER (untuk mode REAL dengan hardware ESP32)
# ============================================================================

def setup_mqtt():
    """Setup koneksi MQTT untuk menerima data dari ESP32."""
    global mqtt_connected, simulation_mode

    if not MQTT_AVAILABLE:
        print("[MQTT] Library paho-mqtt tidak terinstall. Jalankan: pip install paho-mqtt")
        print("[MQTT] Kembali ke mode SIMULASI.")
        return None, None

    MQTT_BROKER = os.environ.get("MQTT_BROKER", "localhost")
    MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))
    MQTT_TOPIC = os.environ.get("MQTT_TOPIC", "esp32/radar")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    def on_connect(client, userdata, flags, rc, properties=None):
        global mqtt_connected
        if rc == 0:
            mqtt_connected = True
            print(f"[MQTT] Terhubung ke broker {MQTT_BROKER}:{MQTT_PORT}")
        else:
            print(f"[MQTT] Gagal terhubung, rc={rc}")

    def on_message(client, userdata, msg):
        """
        Callback MQTT.
        Data dari ESP32 format: "sudut,jarak"
        Contoh: "45,120" -> sudut=45°, jarak=120cm
        """
        global current_servo_angle, current_distance, last_mqtt_time, simulation_mode

        payload = msg.payload.decode('utf-8').strip()
        match = re.match(r"^(\d+),(\d+)$", payload)
        if match:
            sudut = int(match.group(1))
            jarak = int(match.group(2))

            # Filter jarak
            if jarak <= 0 or jarak > MAX_SENSOR_CM:
                jarak = MAX_SENSOR_CM

            current_servo_angle = sudut
            current_distance = jarak

            # Konversi ke koordinat global dan simpan
            if jarak < MAX_SENSOR_CM - 5:
                gx, gy = convert_to_global(jarak, sudut)
                add_wall_point(gx, gy)

            # ✅ Kembalikan ke mode REAL setiap kali ada data dari ESP32
            simulation_mode = False
            last_mqtt_time = time.time()

    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.subscribe(MQTT_TOPIC)
        client.loop_start()
        print(f"[MQTT] Mendengarkan topik '{MQTT_TOPIC}'...")
        return client, MQTT_TOPIC
    except Exception as e:
        print(f"[MQTT] Error koneksi: {e}")
        print("[MQTT] Kembali ke mode SIMULASI.")
        return None, None


def mqtt_timeout_checker(mqtt_client):
    """
    Thread checker: memantau kedatangan data MQTT dari ESP32.
    - Jika ada data MQTT → mode REAL (menampilkan data sensor)
    - Jika tidak ada data → mode STANDBY (diam, menunggu data)
    - TIDAK ADA auto-sweep simulasi!
    """
    global simulation_mode, last_mqtt_time

    while True:
        time.sleep(0.5)
        if mqtt_client is not None:
            # Jika mode STANDBY, cek apakah ada data MQTT baru
            if simulation_mode:
                if time.time() - last_mqtt_time < 5.0:
                    simulation_mode = False
                    print("[MQTT] Data diterima! Beralih ke mode REAL.")
            # Jika mode REAL, cek timeout → kembali ke STANDBY (bukan simulasi!)
            else:
                if time.time() - last_mqtt_time > 5.0:
                    print("[MQTT] Timeout! Menunggu data dari ESP32... (STANDBY)")
                    simulation_mode = True


# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    global robot_x, robot_y, robot_yaw, current_servo_angle
    global current_distance, simulation_mode, auto_sweep, fps
    global wall_points, scan_counter, last_scan_time, scale

    # --- Parse argument ---
    parser = argparse.ArgumentParser(description="Mobile 2D Grid Mapping")
    parser.add_argument("--real", action="store_true",
                        help="Mode real: terima data dari MQTT (ESP32)")
    parser.add_argument("--broker", type=str, default=None,
                        help="MQTT broker address (default: localhost atau env MQTT_BROKER)")
    args = parser.parse_args()

    scale = SCALE  # Gunakan scale dari parameter

    # --- Setup MQTT — SELALU DI AWAL, sebelum print mode ---
    mqtt_client = None
    mqtt_topic = None
    print("[SYSTEM] Mencoba koneksi MQTT ke broker...")
    mqtt_client, mqtt_topic = setup_mqtt()
    
    if args.real or mqtt_client is not None:
        simulation_mode = False
        if args.broker:
            os.environ["MQTT_BROKER"] = args.broker
        print("[MODE] REAL — Menerima data dari ESP32 via MQTT")
        print(f"        Broker: {os.environ.get('MQTT_BROKER', 'localhost')}:{os.environ.get('MQTT_PORT', '1883')}")
        print(f"        Topik : {os.environ.get('MQTT_TOPIC', 'esp32/radar')}")
        # Thread timeout checker (switching otomatis real ↔ simulasi)
        if mqtt_client is not None:
            t = threading.Thread(target=mqtt_timeout_checker, args=(mqtt_client,), daemon=True)
            t.start()
    else:
        simulation_mode = True
        print("[MODE] SIMULASI — Menggunakan dinding virtual built-in")
        print("  Kontrol: W/A/S/D=Gerak, Q/E=Rotasi, Spasi=Scan, +/-=Zoom, ESC=Keluar")

    # --- Inisialisasi Pygame ---
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("MAPPING SPACE — Mobile 2D Grid Mapping")
    clock = pygame.time.Clock()

    # Font untuk FPS
    font_fps = pygame.font.SysFont('monospace', 10)

    # Variabel untuk auto-sweep
    last_scan_time = time.time()
    
    # Jika mode REAL (MQTT terhubung), matikan auto_sweep agar tidak simulasi
    if not simulation_mode:
        auto_sweep = False
        print("[SYSTEM] Auto-sweep dimatikan — menunggu data dari ESP32...")
    
    # Kirim data dummy MQTT agar ESP32 tahu mapping-space siap menerima
    if mqtt_client is not None:
        try:
            mqtt_client.publish("esp32/radar/status", "mapping_ready")
        except:
            pass

    # --- MAIN LOOP ---
    running = True
    while running:
        # --- Event handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                move_step = 50  # cm per langkah

                if event.key == pygame.K_ESCAPE:
                    running = False

                # --- Pergerakan robot (WASD) ---
                elif event.key == pygame.K_w:  # Maju
                    robot_x += math.cos(robot_yaw) * move_step
                    robot_y += math.sin(robot_yaw) * move_step

                elif event.key == pygame.K_s:  # Mundur
                    robot_x -= math.cos(robot_yaw) * move_step
                    robot_y -= math.sin(robot_yaw) * move_step

                elif event.key == pygame.K_a:  # Geser kiri (strafe)
                    strafe_angle = robot_yaw - math.pi / 2
                    robot_x += math.cos(strafe_angle) * move_step
                    robot_y += math.sin(strafe_angle) * move_step

                elif event.key == pygame.K_d:  # Geser kanan (strafe)
                    strafe_angle = robot_yaw + math.pi / 2
                    robot_x += math.cos(strafe_angle) * move_step
                    robot_y += math.sin(strafe_angle) * move_step

                # --- Rotasi heading ---
                elif event.key == pygame.K_q:  # Putar kiri
                    robot_yaw -= math.radians(15)

                elif event.key == pygame.K_e:  # Putar kanan
                    robot_yaw += math.radians(15)

                # --- Reset ---
                elif event.key == pygame.K_r:  # Reset posisi
                    robot_x = 0.0
                    robot_y = 0.0
                    robot_yaw = 0.0
                    wall_points.clear()
                    print("[RESET] Posisi & peta di-reset.")

                # --- Scan manual ---
                elif event.key == pygame.K_SPACE:
                    if simulation_mode:
                        do_scan()
                        print(f"[SCAN] Servo={current_servo_angle:.0f}°, Jarak={current_distance:.0f}cm")

                # --- Zoom ---
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    scale = min(scale * 1.2, 10.0)
                elif event.key == pygame.K_MINUS:
                    scale = max(scale / 1.2, 0.2)

        # --- Update auto-sweep (simulasi) atau terima data MQTT (real) ---
        if simulation_mode and auto_sweep:
            update_auto_sweep()
        elif not simulation_mode:
            # Mode REAL — data sudah diproses di callback MQTT on_message
            # Hanya update visualisasi, tidak perlu scan ulang
            pass

        # --- Gambar semua elemen ---
        screen.fill(COLOR_BG)

        # 1. Grid
        draw_grid(screen, scale, MAP_CENTER_X, MAP_CENTER_Y)

        # 2. Wall points (permanen, akumulatif)
        draw_wall_points(screen, scale)

        # 3. Robot + scanner
        draw_robot(screen, scale)

        # 4. Minimap
        draw_minimap(screen)

        # 5. Info bar
        draw_info_bar(screen, scale)

        # 6. FPS counter
        fps = clock.get_fps()
        fps_text = font_fps.render(f"FPS: {fps:.1f}", True, COLOR_TEXT_DIM)
        screen.blit(fps_text, (WINDOW_WIDTH - 80, 5))

        # --- Update display ---
        pygame.display.flip()
        clock.tick(60)  # Max 60 FPS

    # --- Cleanup ---
    if mqtt_client is not None:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()