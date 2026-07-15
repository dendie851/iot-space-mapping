#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// WiFi Event Handler untuk monitoring koneksi
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi Connected - IP: " + WiFi.localIP().toString());
      digitalWrite(PIN_LED_BUILTIN, HIGH);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] Disconnected - attempting reconnect...");
      digitalWrite(PIN_LED_BUILTIN, LOW);
      WiFi.reconnect();
      break;
    default:
      break;
  }
}

// ============================================================
// KONFIGURASI - Sesuaikan dengan lingkungan Anda
// ============================================================

// --- WiFi ---
const char* ssid         = "NAMA_WIFI_ANDA";
const char* password     = "PASSWORD_WIFI_ANDA";

// --- MQTT ---
const char* mqtt_server  = "IP_LAPTOP_ANDA"; // IP komputer/MQTT broker
const int   mqtt_port    = 1883;
const char* mqtt_topic   = "esp32/radar";
const char* mqtt_client_id = "ESP32RadarClient";

// --- Topik Status (Telemetry) ---
const char* mqtt_topic_status  = "esp32/status";
const char* mqtt_topic_cmd     = "esp32/cmd";

// ============================================================
// WIRING DIAGRAM - HC-SR04 ke ESP32
// ============================================================
/*
 * ESP32 Pinout (berdasarkan gambar wiring):
 * 
 * HC-SR04 Ultrasonic Sensor:
 *    ESP32 GPIO 5  (D1)  ----> HC-SR04 Trig
 *    ESP32 GPIO 17 (D2)  ----> HC-SR04 Echo
 *    ESP32 3V3/5V         ----> HC-SR04 VCC
 *    ESP32 GND            ----> HC-SR04 GND
 * 
 * Dynamo Servo:
 *    ESP32 GPIO 18        ----> Servo Signal (oranye/kuning)
 *    ESP32 5V             ----> Servo VCC (merah)
 *    ESP32 GND            ----> Servo GND (coklat/hitam)
 * 
 * Catatan: Jika servo bergerak lemah/berat, gunakan power supply
 *          eksternal 5V untuk servo (jangan hanya pakai 5V dari ESP32)
 */
// ============================================================

// --- PIN Hardware ---
// HC-SR04 Ultrasonic Sensor connections
const int PIN_TRIG    = 5;   // HC-SR04 Trigger -> ESP32 Pin D1 (GPIO 5)
const int PIN_ECHO    = 17;  // HC-SR04 Echo -> ESP32 Pin D2 (GPIO 17)

// Servo Motor (Dynamo Servo) connection
const int PIN_SERVO   = 18;  // Servo Signal -> ESP32 GPIO 18

// Built-in LED untuk indikasi koneksi (aktif HIGH)
const int PIN_LED_BUILTIN = 2;

// --- Konfigurasi Radar ---
const int   sudutMin      = 0;
const int   sudutMax      = 180;
const int   sudutStep     = 2;
const int   servoDelay    = 30;    // ms per step servo
const long  timeoutWifi   = 30000; // 30 detik timeout koneksi WiFi

// ============================================================
// GLOBAL VARIABLES
// ============================================================
Servo myServo;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastReconnectAttempt = 0;
unsigned long lastStatusPublish    = 0;
const unsigned long statusInterval = 10000; // Publikasi status setiap 10 detik

int currentAngle = 0;
int direction    = 1;  // 1 = maju, -1 = mundur

// ============================================================
// FUNGSI UTILITY - Sensor Ultrasonic HC-SR04
// ============================================================

// HC-SR04: Membaca jarak dalam centimeter (2cm - 400cm)
// - Trigger: kirim pulse 10µs HIGH
// - Echo: baca durasi pulse HIGH (timeout 30ms)
long hitungJarak() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long durasi = pulseIn(PIN_ECHO, HIGH, 30000); // timeout 30ms
  if (durasi == 0) return -1; // gagal membaca

  long jarakCm = durasi * 0.034 / 2;
  return jarakCm;
}

// ============================================================
// MANAJEMEN KONEKSI WiFi (RELIABLE & AUTO-RECONNECT)
// ============================================================

bool connectWiFi() {
  Serial.print("Menghubungkan ke WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);  // Bersihkan koneksi lama
  delay(100);
  WiFi.begin(ssid, password);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // Blink LED internal sebagai indikasi proses koneksi
    digitalWrite(PIN_LED_BUILTIN, !digitalRead(PIN_LED_BUILTIN));

    // Timeout jika terlalu lama
    if (millis() - startAttempt > timeoutWifi) {
      Serial.println("\n[ERROR] Gagal konek WiFi - Timeout!");
      return false;
    }
  }

  Serial.println("\nWiFi Terhubung!");
  Serial.print("IP Address : ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI       : ");
  Serial.println(WiFi.RSSI());

  digitalWrite(PIN_LED_BUILTIN, HIGH); // LED nyala stabil jika WiFi OK
  return true;
}

// Memonitor koneksi WiFi dan reconnect otomatis jika putus
bool maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Koneksi terputus! Mencoba reconnect...");
    digitalWrite(PIN_LED_BUILTIN, LOW);
    
    // Reconnect dengan backoff strategy
    static int reconnectAttempts = 0;
    static unsigned long lastAttempt = 0;
    
    if (millis() - lastAttempt > 5000) {  // Coba setiap 5 detik
      lastAttempt = millis();
      reconnectAttempts++;
      
      if (reconnectAttempts > 10) {
        // Restart WiFi stack jika terlalu banyak gagal
        Serial.println("[WiFi] Restarting WiFi stack...");
        WiFi.disconnect(true);
        delay(1000);
        reconnectAttempts = 0;
      }
      
      WiFi.disconnect();
      bool result = connectWiFi();
      if (result) {
        reconnectAttempts = 0;
      }
      return result;
    }
  }
  return true;
}

// ============================================================
// MANAJEMEN KONEKSI MQTT (RELIABLE & AUTO-RECONNECT)
// ============================================================

bool connectMQTT() {
  Serial.print("Menghubungkan ke MQTT Broker: ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.println(mqtt_port);

  if (client.connect(mqtt_client_id)) {
    Serial.println("MQTT Terhubung!");

    // Subscribe ke topic command untuk menerima perintah
    client.subscribe(mqtt_topic_cmd);
    Serial.print("Subscribe ke: ");
    Serial.println(mqtt_topic_cmd);

    // Publikasikan status online
    client.publish(mqtt_topic_status, "{\"status\":\"online\",\"device\":\"esp32-radar\"}");
    return true;
  } else {
    Serial.print("MQTT Gagal, rc=");
    Serial.println(client.state());
    return false;
  }
}

// Memonitor dan reconnect MQTT jika putus
void maintainMQTT() {
  if (!client.connected()) {
    Serial.println("[MQTT] Koneksi terputus! Mencoba reconnect...");

    // Pastikan WiFi masih terhubung sebelum reconnect MQTT
    if (maintainWiFi()) {
      connectMQTT();
    }
  }
  client.loop();
}

// ============================================================
// CALLBACK MQTT - Menerima perintah dari broker
// ============================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Pesan diterima [");
  Serial.print(topic);
  Serial.print("] : ");

  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Proses perintah sederhana
  if (String(topic) == mqtt_topic_cmd) {
    if (message == "stop") {
      // Hentikan scanning
      direction = 0;
      Serial.println("[CMD] Radar dihentikan");
    } else if (message == "start") {
      // Mulai scanning
      direction = 1;
      Serial.println("[CMD] Radar dimulai");
    } else if (message == "reset") {
      // Reset posisi servo ke 0
      myServo.write(0);
      currentAngle = 0;
      Serial.println("[CMD] Servo di-reset ke posisi 0");
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("  ESP32 RADAR MAPPING - EDGE DEVICE");
  Serial.println("========================================");

  // Inisialisasi pin HC-SR04
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW); // Pastikan trigger LOW saat startup
  
  // Inisialisasi LED indikator
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  digitalWrite(PIN_LED_BUILTIN, LOW);
  
  // Register WiFi event handler untuk monitoring otomatis
  WiFi.onEvent(WiFiEvent);

  // Attach Servo
  myServo.attach(PIN_SERVO);
  myServo.write(0);
  delay(500);

  // Koneksi WiFi (dengan retry & timeout handling)
  if (!connectWiFi()) {
    Serial.println("[WARNING] WiFi tidak terhubung, akan mencoba lagi di loop...");
  }

  // Konfigurasi MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  // Koneksi MQTT
  if (WiFi.status() == WL_CONNECTED) {
    connectMQTT();
  }

  Serial.println("Setup selesai!\n");
}

// ============================================================
// LOOP UTAMA - Edge Computing Architecture
// ============================================================

void loop() {
  // 1️ PASTIKAN KONEKSI TETAP HIDUP (Connection Maintenance)
  maintainWiFi();
  maintainMQTT();

  // 2️ EDGE COMPUTING - Proses data di edge device
  //    sebelum dikirim ke cloud/server

  // Jika ada perintah untuk menghentikan scanning
  if (direction == 0) {
    // Tetap maintain koneksi walau scanning berhenti
    delay(100);
    return;
  }

  // --- Scan Radar (Edge Processing) ---
  // Compute sudut berikutnya
  currentAngle += (sudutStep * direction);

  // Balik arah jika mencapai batas
  if (currentAngle >= sudutMax) {
    currentAngle = sudutMax;
    direction = -1;
  } else if (currentAngle <= sudutMin) {
    currentAngle = sudutMin;
    direction = 1;
  }

  // Gerakkan servo ke sudut yang dituju
  myServo.write(currentAngle);
  delay(servoDelay);

  // Baca jarak dari sensor ultrasonik
  long jarak = hitungJarak();

  // *** Edge Filtering ***
  // Jika jarak -1 (error) atau di luar range (> 400cm), skip data
  if (jarak > 0 && jarak <= 400) {
    // Format payload JSON untuk data terstruktur
    String payload = "{\"angle\":" + String(currentAngle) +
                     ",\"distance\":" + String(jarak) +
                     ",\"rssi\":" + String(WiFi.RSSI()) + "}";

    // Publikasikan ke MQTT
    client.publish(mqtt_topic, payload.c_str());

    // Debug serial
    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print("°, Jarak: ");
    Serial.print(jarak);
    Serial.println(" cm");
  }

  // 3️ PUBLIKASI STATUS PERIODIK (Telemetry)
  unsigned long now = millis();
  if (now - lastStatusPublish >= statusInterval) {
    lastStatusPublish = now;

    // Kirim status ke MQTT
    String statusPayload = "{\"status\":\"running\",\"wifi_rssi\":" +
                           String(WiFi.RSSI()) + ",\"uptime_ms\":" +
                           String(now) + "}";
    client.publish(mqtt_topic_status, statusPayload.c_str());

    Serial.print("[STATUS] Uptime: ");
    Serial.print(now / 1000);
    Serial.println(" detik");
  }
}