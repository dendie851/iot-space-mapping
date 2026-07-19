#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// ============================================================
// HARDWARE SETUP & CONFIGURATION
// ============================================================

// --- PIN Hardware (Dipindahkan ke atas agar tidak error scope) --
// HC-SR04 Ultrasonic Sensor connections
const int PIN_TRIG    = 2;   // D2 (GPIO 2)  ----> Hubungkan ke HC-SR04 Trig
const int PIN_ECHO    = 4;   // D4 (GPIO 4)  ----> Hubungkan ke HC-SR04 Echo

// Servo Motor (Dynamo Servo) connection
const int PIN_SERVO   = 15;  // D15 (GPIO 15) ----> Hubungkan ke Sinyal Servo

// PIN LED Indikator
const int PIN_LED_BUILTIN = 13; // GPIO 13

// --- WiFi Configuration ---
const char* ssid         = "PutriTunggal";
const char* password     = "uu311009";

// --- MQTT Configuration ---
const char* mqtt_server  = "192.168.100.35"; // IP komputer/MQTT broker
const int   mqtt_port    = 1883;
const char* mqtt_topic   = "esp32/radar";
const char* mqtt_client_id = "ESP32RadarClient";

// --- Topik Status (Telemetry) ---
const char* mqtt_topic_status  = "esp32/status";
const char* mqtt_topic_cmd     = "esp32/cmd";

// ============================================================
// WIRING DIAGRAM REFERENCE
// ============================================================
/*
 * HC-SR04 Ultrasonic Sensor:
 *    ESP32 GPIO 2  (D2)  ----> HC-SR04 Trig
 *    ESP32 GPIO 4  (D4)  ----> HC-SR04 Echo
 *    ESP32 3V3/5V        ----> HC-SR04 VCC
 *    ESP32 GND            ----> HC-SR04 GND
 * 
 * Dynamo Servo:
 *    ESP32 GPIO 15 (D15) ----> Servo Signal (oranye/kuning)
 *    ESP32 5V             ----> Servo VCC (merah)
 *    ESP32 GND            ----> Servo GND (coklat/hitam)
 */
// ============================================================

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
// MANAJEMEN KONEKSI WiFi
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

bool maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Koneksi terputus! Mencoba reconnect...");
    digitalWrite(PIN_LED_BUILTIN, LOW);
    
    static int reconnectAttempts = 0;
    static unsigned long lastAttempt = 0;
    
    if (millis() - lastAttempt > 5000) {  // Coba setiap 5 detik
      lastAttempt = millis();
      reconnectAttempts++;
      
      if (reconnectAttempts > 10) {
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
// MANAJEMEN KONEKSI MQTT
// ============================================================

bool connectMQTT() {
  Serial.print("Menghubungkan ke MQTT Broker: ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.println(mqtt_port);

  if (client.connect(mqtt_client_id)) {
    Serial.println("MQTT Terhubung!");
    client.subscribe(mqtt_topic_cmd);
    Serial.print("Subscribe ke: ");
    Serial.println(mqtt_topic_cmd);

    client.publish(mqtt_topic_status, "{\"status\":\"online\",\"device\":\"esp32-radar\"}");
    return true;
  } else {
    Serial.print("MQTT Gagal, rc=");
    Serial.println(client.state());
    // Tambahkan informasi state untuk debugging
    switch(client.state()) {
      case -4: Serial.println(" -> MQTT_CONNECTION_TIMEOUT"); break;
      case -3: Serial.println(" -> MQTT_CONNECTION_LOST"); break;
      case -2: Serial.println(" -> MQTT_CONNECT_FAILED"); break;
      case -1: Serial.println(" -> MQTT_DISCONNECTED"); break;
      case 1:  Serial.println(" -> MQTT_CONNECT_BAD_PROTOCOL"); break;
      case 2:  Serial.println(" -> MQTT_CONNECT_BAD_CLIENT_ID"); break;
      case 3:  Serial.println(" -> MQTT_CONNECT_UNAVAILABLE"); break;
      case 4:  Serial.println(" -> MQTT_CONNECT_BAD_CREDENTIALS"); break;
      case 5:  Serial.println(" -> MQTT_CONNECT_UNAUTHORIZED"); break;
    }
    return false;
  }
}

void maintainMQTT() {
  if (!client.connected()) {
    Serial.println("[MQTT] Koneksi terputus! Mencoba reconnect...");
    if (maintainWiFi()) {
      connectMQTT();
    }
  }
  client.loop();
}

// ============================================================
// CALLBACK MQTT - Menerima perintah
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

  if (String(topic) == mqtt_topic_cmd) {
    if (message == "stop") {
      direction = 0;
      Serial.println("[CMD] Radar dihentikan");
    } else if (message == "start") {
      direction = 1;
      Serial.println("[CMD] Radar dimulai");
    } else if (message == "reset") {
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
  digitalWrite(PIN_TRIG, LOW);
  
  // Inisialisasi LED indikator
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  digitalWrite(PIN_LED_BUILTIN, LOW);
  
  // Register WiFi event handler untuk monitoring otomatis
  WiFi.onEvent(WiFiEvent);

  // Attach Servo
  myServo.attach(PIN_SERVO);
  myServo.write(0);
  delay(500);

  // Koneksi WiFi
  if (!connectWiFi()) {
    Serial.println("[WARNING] WiFi tidak terhubung, akan mencoba lagi di loop...");
  }

  // Konfigurasi MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  if (WiFi.status() == WL_CONNECTED) {
    connectMQTT();
  }

  Serial.println("Setup selesai!\n");
}

// ============================================================
// LOOP UTAMA
// ============================================================

void loop() {
  maintainWiFi();
  maintainMQTT();

  if (direction == 0) {
    delay(100);
    return;
  }

  // --- Scan Radar ---
  currentAngle += (sudutStep * direction);

  if (currentAngle >= sudutMax) {
    currentAngle = sudutMax;
    direction = -1;
  } else if (currentAngle <= sudutMin) {
    currentAngle = sudutMin;
    direction = 1;
  }

  myServo.write(currentAngle);
  delay(servoDelay);

  long jarak = hitungJarak();

  // Edge Filtering
  if (jarak > 0 && jarak <= 15) {
    String payload = String(currentAngle) + "," + String(jarak);

    client.publish(mqtt_topic, payload.c_str());

    Serial.print("Angle: ");
    Serial.print(currentAngle);
    Serial.print("°, Jarak: ");
    Serial.print(jarak);
    Serial.println(" cm");
  }

  // Publikasi status berkala
  unsigned long now = millis();
  if (now - lastStatusPublish >= statusInterval) {
    lastStatusPublish = now;

    String statusPayload = "{\"status\":\"running\",\"wifi_rssi\":" +
                           String(WiFi.RSSI()) + ",\"uptime_ms\":" +
                           String(now) + "}";
    client.publish(mqtt_topic_status, statusPayload.c_str());

    Serial.print("[STATUS] Uptime: ");
    Serial.print(now / 1000);
    Serial.println(" detik");
  }
}