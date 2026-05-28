#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <math.h>

// =============================================================
//                        설정
// =============================================================
const char* WIFI_SSID     = "asia-edu_2G";
const char* WIFI_PASSWORD = "12345678";
const char* OTA_HOSTNAME  = "esp32";
const char* OTA_PASSWORD  = "ota1234";

const char* AP_SSID     = "UGV-Recovery";
const char* AP_PASSWORD = "recovery123";

const int WIFI_TIMEOUT_SEC = 15;

// UART (라즈파이): UART0 = Serial = GPIO 1(TX), 3(RX) - PCB 직결
const uint32_t UART_BAUD = 115200;

// 디버그 UDP (USB Serial 못 쓰니까 디버그는 WiFi로)
const uint16_t DEBUG_PORT = 4212;
WiFiUDP udpDebug;
IPAddress broadcastIP(192, 168, 0, 255);

// =============================================================
//                  모터 (TB6612FNG) 핀
// =============================================================
#define PIN_AIN1  21
#define PIN_AIN2  17
#define PIN_PWMA  25
#define PIN_BIN1  22
#define PIN_BIN2  23
#define PIN_PWMB  26

#define PWM_CH_A     0
#define PWM_CH_B     1
#define PWM_FREQ     20000
#define PWM_RES_BITS 8

// 모터 PWM 한계값
const int MAX_PWM = 230;   // 최대 PWM (255 풀스케일 약간 아래)
const int MIN_PWM = 50;    // 데드존 - 이 이하는 모터가 윙윙거리기만 함

// 워치독: 마지막 명령 후 이 시간 지나면 자동 정지
const unsigned long CMD_TIMEOUT_MS = 500;

// =============================================================
//                  IMU (QMI8658 + AK09918)
// =============================================================
#define S_SDA 32
#define S_SCL 33

#define QMI8658_ADDR     0x6B
#define QMI8658_WHO_AM_I 0x00
#define QMI8658_CTRL1    0x02
#define QMI8658_CTRL2    0x03
#define QMI8658_CTRL3    0x04
#define QMI8658_CTRL5    0x06
#define QMI8658_CTRL7    0x08
#define QMI8658_STATUS0  0x2E
#define QMI8658_AX_L     0x35
#define QMI8658_RESET    0x60

#define AK09918_ADDR       0x0C
#define AK09918_WIA2       0x01
#define AK09918_ST1        0x10
#define AK09918_HXL        0x11
#define AK09918_CNTL2      0x31
#define AK09918_CONT_100HZ 0x08

#define ACC_SCALE  4096.0f
#define GYRO_SCALE 64.0f

#define Kp 2.0f
#define Ki 0.005f

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;
unsigned long lastUpdate = 0;

float   accOffset[3]  = {0, 0, 0};
float   gyroOffset[3] = {0, 0, 0};
int16_t magOffset[3]  = {0, 0, 0};

bool imuReady = false;

// =============================================================
//                  전역 상태
// =============================================================
bool wifiConnected = false;
bool otaRunning = false;
unsigned long lastCmdMs = 0;
int16_t currentLeft  = 0;
int16_t currentRight = 0;

// =============================================================
//                  디버그 (UDP only)
// =============================================================
void dbg(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (WiFi.status() == WL_CONNECTED) {
    udpDebug.beginPacket(broadcastIP, DEBUG_PORT);
    udpDebug.write((const uint8_t*)buf, n);
    udpDebug.write('\n');
    udpDebug.endPacket();
  }
}

// =============================================================
//                  모터 제어
// =============================================================
void motorStop() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
}

// 좌 모터 PWM (-255 ~ +255). 회전 방향 이전 확정대로 HIGH/LOW 뒤집어둠.
void setLeftPWM(int16_t pwm) {
  uint8_t abs_pwm = (uint8_t)constrain(abs(pwm), 0, 255);
  // 데드존: 너무 작은 값은 0 처리
  if (abs_pwm > 0 && abs_pwm < MIN_PWM) abs_pwm = 0;

  if (pwm > 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else if (pwm < 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
  }
  ledcWrite(PWM_CH_A, abs_pwm);
}

void setRightPWM(int16_t pwm) {
  uint8_t abs_pwm = (uint8_t)constrain(abs(pwm), 0, 255);
  if (abs_pwm > 0 && abs_pwm < MIN_PWM) abs_pwm = 0;

  if (pwm > 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  } else if (pwm < 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
  }
  ledcWrite(PWM_CH_B, abs_pwm);
}

void setupMotors() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_PWMA, PWM_CH_A);
  ledcAttachPin(PIN_PWMB, PWM_CH_B);

  motorStop();
}

// 좌우 PWM 직접 적용
void applyPwm(int16_t left, int16_t right) {
  left  = constrain(left,  -MAX_PWM, MAX_PWM);
  right = constrain(right, -MAX_PWM, MAX_PWM);
  setLeftPWM(left);
  setRightPWM(right);
  currentLeft  = left;
  currentRight = right;
}

// =============================================================
//                  JSON 명령 처리
// =============================================================
// 명령 형식 (FSM이 PWM 직접 계산):
//   {"T":"m","L":<-255..255>,"R":<-255..255>}   -- 모터 PWM 직접 지정
//   {"T":"e"}                                    -- 비상정지
//   {"T":"ping"}                                 -- 살아있나 (응답: {"T":"pong"})
void processCommand(const char* json_line) {
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, json_line);
  if (err) {
    dbg("[json err] %s on: %s", err.c_str(), json_line);
    return;
  }

  const char* type = doc["T"] | "";

  if (strcmp(type, "m") == 0) {
    int l = doc["L"] | 0;
    int r = doc["R"] | 0;
    applyPwm(l, r);
    lastCmdMs = millis();
    dbg("[cmd m] L=%d R=%d", l, r);
  }
  else if (strcmp(type, "e") == 0) {
    motorStop();
    currentLeft  = 0;
    currentRight = 0;
    lastCmdMs = millis();
    dbg("[cmd e] emergency stop");
  }
  else if (strcmp(type, "ping") == 0) {
    Serial.println("{\"T\":\"pong\"}");
    lastCmdMs = millis();
  }
  else {
    dbg("[cmd unknown] T=%s", type);
  }
}

// UART에서 한 줄(\n까지)씩 받아서 처리
void handleUart() {
  static char line[256];
  static size_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        line[idx] = '\0';
        processCommand(line);
        idx = 0;
      }
    } else if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    } else {
      // 라인 너무 길면 버림 (오버플로 방지)
      idx = 0;
    }
  }
}

// 워치독: 일정 시간 명령 없으면 자동 정지
void checkWatchdog() {
  bool moving = (currentLeft != 0) || (currentRight != 0);
  if (moving && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    dbg("[watchdog] timeout -> STOP");
    motorStop();
    currentLeft  = 0;
    currentRight = 0;
  }
}

// =============================================================
//                  IMU 텔레메트리 송신 (JSON, 50Hz)
// =============================================================
void sendImuJson(float roll, float pitch, float yaw,
                 float ax, float ay, float az,
                 float gx, float gy, float gz) {
  Serial.printf(
    "{\"T\":\"imu\",\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}\n",
    roll, pitch, yaw, ax, ay, az, gx, gy, gz);
}

// =============================================================
//                  I2C / IMU 헬퍼
// =============================================================
void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(addr, len);
  for (int i = 0; i < len; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  uint8_t val = 0;
  readRegs(addr, reg, &val, 1);
  return val;
}

bool qmi8658Init() {
  writeReg(QMI8658_ADDR, QMI8658_RESET, 0xB0);
  delay(50);
  uint8_t whoami = readReg(QMI8658_ADDR, QMI8658_WHO_AM_I);
  dbg("QMI8658 WHO_AM_I: 0x%02X", whoami);
  if (whoami != 0x05) return false;
  writeReg(QMI8658_ADDR, QMI8658_CTRL1, 0x60);
  writeReg(QMI8658_ADDR, QMI8658_CTRL2, 0x23);
  writeReg(QMI8658_ADDR, QMI8658_CTRL3, 0x53);
  writeReg(QMI8658_ADDR, QMI8658_CTRL5, 0x00);
  writeReg(QMI8658_ADDR, QMI8658_CTRL7, 0x03);
  delay(50);
  return true;
}

bool qmi8658Read(float acc[3], float gyro[3]) {
  uint8_t status = readReg(QMI8658_ADDR, QMI8658_STATUS0);
  if (!(status & 0x03)) return false;
  uint8_t buf[12];
  if (!readRegs(QMI8658_ADDR, QMI8658_AX_L, buf, 12)) return false;
  int16_t raw_ax = (int16_t)(buf[1]  << 8 | buf[0]);
  int16_t raw_ay = (int16_t)(buf[3]  << 8 | buf[2]);
  int16_t raw_az = (int16_t)(buf[5]  << 8 | buf[4]);
  int16_t raw_gx = (int16_t)(buf[7]  << 8 | buf[6]);
  int16_t raw_gy = (int16_t)(buf[9]  << 8 | buf[8]);
  int16_t raw_gz = (int16_t)(buf[11] << 8 | buf[10]);
  acc[0]  = raw_ax / ACC_SCALE  - accOffset[0];
  acc[1]  = raw_ay / ACC_SCALE  - accOffset[1];
  acc[2]  = raw_az / ACC_SCALE  - accOffset[2];
  gyro[0] = raw_gx / GYRO_SCALE - gyroOffset[0];
  gyro[1] = raw_gy / GYRO_SCALE - gyroOffset[1];
  gyro[2] = raw_gz / GYRO_SCALE - gyroOffset[2];
  return true;
}

bool ak09918Init() {
  uint8_t wia = readReg(AK09918_ADDR, AK09918_WIA2);
  dbg("AK09918 WIA2: 0x%02X", wia);
  if (wia != 0x0C) return false;
  writeReg(AK09918_ADDR, AK09918_CNTL2, AK09918_CONT_100HZ);
  delay(10);
  return true;
}

bool ak09918Read(float mag[3]) {
  uint8_t st1 = readReg(AK09918_ADDR, AK09918_ST1);
  if (!(st1 & 0x01)) return false;
  uint8_t buf[8];
  if (!readRegs(AK09918_ADDR, AK09918_HXL, buf, 8)) return false;
  if (buf[7] & 0x08) return false;
  int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);
  mag[0] = (raw_x - magOffset[0]) * 0.15f;
  mag[1] = (raw_y - magOffset[1]) * 0.15f;
  mag[2] = (raw_z - magOffset[2]) * 0.15f;
  return true;
}

void calibrateIMU() {
  dbg(">>> Calibrating IMU (keep still for 3s)");
  delay(3000);
  float accSum[3] = {0}, gyroSum[3] = {0};
  int count = 0;
  for (int i = 0; i < 200; i++) {
    float acc[3], gyro[3];
    if (qmi8658Read(acc, gyro)) {
      for (int j = 0; j < 3; j++) { accSum[j] += acc[j]; gyroSum[j] += gyro[j]; }
      count++;
    }
    delay(5);
  }
  if (count > 0) {
    for (int j = 0; j < 3; j++) {
      accOffset[j]  = accSum[j]  / count;
      gyroOffset[j] = gyroSum[j] / count;
    }
    accOffset[2] -= 1.0f;
  }
  dbg("Acc  Offset: %.4f %.4f %.4f", accOffset[0], accOffset[1], accOffset[2]);
  dbg("Gyro Offset: %.4f %.4f %.4f", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
}

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz) {
  unsigned long now = micros();
  float halfT = (now - lastUpdate) / 2000000.0f;
  lastUpdate = now;
  float norm;
  norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm == 0) return;
  ax /= norm; ay /= norm; az /= norm;
  norm = sqrt(mx*mx + my*my + mz*mz);
  if (norm == 0) return;
  mx /= norm; my /= norm; mz /= norm;
  float hx = 2*(mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
  float hy = 2*(mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
  float hz = 2*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));
  float bx = sqrt(hx*hx + hy*hy);
  float bz = hz;
  float vx = 2*(q1*q3 - q0*q2);
  float vy = 2*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  float wx = 2*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2));
  float wy = 2*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3));
  float wz = 2*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2));
  float ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  float ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  float ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
  exInt += ex * Ki * halfT;
  eyInt += ey * Ki * halfT;
  ezInt += ez * Ki * halfT;
  gx = gx * DEG_TO_RAD + Kp*ex + exInt;
  gy = gy * DEG_TO_RAD + Kp*ey + eyInt;
  gz = gz * DEG_TO_RAD + Kp*ez + ezInt;
  q0 += (-q1*gx - q2*gy - q3*gz) * halfT;
  q1 += ( q0*gx + q2*gz - q3*gy) * halfT;
  q2 += ( q0*gy - q1*gz + q3*gx) * halfT;
  q3 += ( q0*gz + q1*gy - q2*gx) * halfT;
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

// =============================================================
//                  WiFi / OTA
// =============================================================
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  IPAddress local_IP(192, 168, 0, 50);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);
  WiFi.config(local_IP, gateway, subnet, primaryDNS);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < WIFI_TIMEOUT_SEC * 2) {
    delay(500);
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);
    wifiConnected = true;
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
  }
}

void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    otaRunning = true;
    dbg("OTA: Start - stopping motors");
    motorStop();
  });
  ArduinoOTA.onEnd([]() { dbg("OTA: End"); });
  ArduinoOTA.onError([](ota_error_t err) { dbg("OTA Error[%u]", err); });
  ArduinoOTA.begin();
}

// =============================================================
//                  Setup / Loop
// =============================================================
void setup() {
  Serial.begin(UART_BAUD);
  delay(500);

  setupMotors();
  setupWiFi();
  setupOTA();

  dbg("=== UGV ESP32 Boot (JSON {L,R} protocol) ===");
  dbg("UART0 (Serial) <-> Pi @ %u baud", UART_BAUD);
  dbg("WiFi: %s, IP: %s",
      wifiConnected ? "STA" : "AP",
      wifiConnected ? WiFi.localIP().toString().c_str()
                    : WiFi.softAPIP().toString().c_str());

  Wire.begin(S_SDA, S_SCL);
  Wire.setClock(400000);
  delay(50);

  if (qmi8658Init() && ak09918Init()) {
    calibrateIMU();
    lastUpdate = micros();
    imuReady = true;
    dbg("IMU ready");
  } else {
    dbg("IMU init FAILED");
  }

  dbg(">>> Setup complete <<<");
}

void loop() {
  ArduinoOTA.handle();
  if (!otaRunning) {
    handleUart();
  }
  checkWatchdog();

  static unsigned long lastImuMs = 0;
  static unsigned long lastTelMs = 0;
  static float lastRoll = 0, lastPitch = 0, lastYaw = 0;
  static float lastAcc[3] = {0,0,0}, lastGyro[3] = {0,0,0};

  if (imuReady && millis() - lastImuMs >= 10) {
    lastImuMs = millis();
    float acc[3], gyro[3], mag[3];
    bool accOk = qmi8658Read(acc, gyro);
    bool magOk = ak09918Read(mag);
    if (accOk) {
      float mx = magOk ? mag[0] : 0;
      float my = magOk ? mag[1] : 0;
      float mz = magOk ? mag[2] : 0;
      mahonyUpdate(gyro[0], gyro[1], gyro[2],
                   acc[0],  acc[1],  acc[2],
                   mx, my, mz);
      lastRoll  =  atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RAD_TO_DEG;
      lastPitch =  asin( 2*(q0*q2 - q1*q3))                         * RAD_TO_DEG;
      lastYaw   =  atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * RAD_TO_DEG;
      memcpy(lastAcc,  acc,  sizeof(lastAcc));
      memcpy(lastGyro, gyro, sizeof(lastGyro));
    }
  }

  if (!otaRunning && millis() - lastTelMs >= 20) {   // 50Hz
    lastTelMs = millis();
    if (imuReady) {
      sendImuJson(lastRoll, lastPitch, lastYaw,
                  lastAcc[0], lastAcc[1], lastAcc[2],
                  lastGyro[0], lastGyro[1], lastGyro[2]);
    }
  }

  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 10000) {
    lastBeat = millis();
    dbg("[alive] L=%d R=%d ip=%s",
        currentLeft, currentRight,
        wifiConnected ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str());
  }
  delay(1);
}
