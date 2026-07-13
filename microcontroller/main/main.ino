#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// --- KONFIGURASI WIFI & MQTT ---
const char* ssid         = "NAMA_WIFI_ANDA";
const char* password     = "PASSWORD_WIFI_ANDA";
const char* mqtt_server  = "IP_LAPTOP_ANDA"; // Gunakan IP lokal laptop Anda (Contoh: 192.168.1.x)
const char* mqtt_topic   = "esp32/radar";

// --- KONFIGURASI PIN HARDWARE ---
const int PIN_SERVO   = 18;  // Pin data Servo (PWM)
const int PIN_TRIG    = 5;   // Pin Trigger Ultrasonik
const int PIN_ECHO    = 17;  // Pin Echo Ultrasonik

Servo myServo;
WiFiClient espClient;
PubSubClient client(espClient);

long hitungJarak() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long durasi = pulseIn(PIN_ECHO, HIGH);
  long jarakCm = durasi * 0.034 / 2; // Konversi ke Centimeter
  return jarakCm;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_TRIG, OUT);
  pinMode(PIN_ECHO, IN);
  myServo.attach(PIN_SERVO);
  
  // Koneksi Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  client.setServer(mqtt_server, 1883);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT Broker...");
    if (client.connect("ESP32RadarClient")) {
      Serial.println("Terhubung!");
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" Mencoba lagi dalam 2 detik");
      delay(2000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Sapuan Radar: Bergerak dari 0 ke 180 derajat
  for (int sudut = 0; sudut <= 180; sudut += 2) {
    myServo.write(sudut);
    delay(30); // Beri waktu servo berputar
    
    long jarak = hitungJarak();
    
    // Format payload: "sudut,jarak" -> Contoh: "45,120"
    String payload = String(sudut) + "," + String(jarak);
    client.publish(mqtt_topic, payload.c_str());
    
    Serial.println(payload);
  }

  // Sapuan Balik: Bergerak dari 180 kembali ke 0 derajat
  for (int sudut = 180; sudut >= 0; sudut -= 2) {
    myServo.write(sudUT);
    delay(30);
    
    long jarak = hitungJarak();
    
    String payload = String(sudut) + "," + String(jarak);
    client.publish(mqtt_topic, payload.c_str());
    
    Serial.println(payload);
  }
}