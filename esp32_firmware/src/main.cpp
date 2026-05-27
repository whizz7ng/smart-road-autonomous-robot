#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ====== 설정 (여기만 수정하면 됨) ======
const char* WIFI_SSID = "asia-edu_2G";       // ← 본인 WiFi SSID
const char* WIFI_PASSWORD = "12345678";   // ← 본인 WiFi 비밀번호
const char* OTA_HOSTNAME = "esp32";             // 네트워크에서 보일 이름
const char* OTA_PASSWORD = "ota1234";               // OTA 업로드 시 사용할 비밀번호

// ====== 세이프 모드 폴백용 AP 설정 ======
const char* AP_SSID = "UGV-Recovery";
const char* AP_PASSWORD = "recovery123";

// WiFi 연결 타임아웃 (초)
const int WIFI_TIMEOUT_SEC = 15;

bool wifiConnected = false;

void setupWiFi() {
  Serial.println();
  Serial.println("--- WiFi Scan ---");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int n = WiFi.scanNetworks();
  Serial.printf("Found %d networks:\n", n);
  for (int i = 0; i < n; i++) {
    Serial.printf("  %2d: %-32s  RSSI=%4d dBm  enc=%d  ch=%d\n",
      i + 1,
      WiFi.SSID(i).c_str(),
      WiFi.RSSI(i),
      WiFi.encryptionType(i),
      WiFi.channel(i)
    );
  }
  Serial.println("--- Scan done ---\n");

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < WIFI_TIMEOUT_SEC * 2) {
    delay(500);
    Serial.printf(". status=%d\n", WiFi.status());
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.printf("WiFi failed. Final status=%d\n", WiFi.status());
    Serial.println("Starting AP recovery mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
}

void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA: Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== UGV ESP32 Firmware Boot ===");

  setupWiFi();
  setupOTA();

  Serial.println("Setup complete.");
  Serial.println(">>> OTA v1 LIVE <<<");
}

void loop() {
  ArduinoOTA.handle();

  // 살아있다는 확인용 heartbeat (10초마다)
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 10000) {
    lastBeat = millis();
    Serial.print("[alive] mode=");
    Serial.print(wifiConnected ? "STA" : "AP");
    Serial.print(" ip=");
    Serial.println(wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString());
  }
}