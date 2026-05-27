#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <math.h>

// =============================================================
//                        설정
// =============================================================
const char* WIFI_SSID     = "asia-edu_2G";
const char* WIFI_PASSWORD = "12345678";
const char* OTA_HOSTNAME  = "esp32";
const char* OTA_PASSWORD  = "ota1234";

// 세이프 모드 폴백 AP
const char* AP_SSID     = "UGV-Recovery";
const char* AP_PASSWORD = "recovery123";

const int WIFI_TIMEOUT_SEC = 15;

// UDP 포트
const uint16_t CMD_PORT       = 4210;   // PC -> ESP32 명령
const uint16_t TELEMETRY_PORT = 4211;   // ESP32 -> PC IMU 데이터
// PC IP는 첫 명령 패킷 받을 때 자동으로 학습함. (브로드캐스트 폴백 가능)

// =============================================================
//                  모터 (TB6612FNG) 핀
//   ※ 실제 보드 배선 기준 (어제 동료 확인 완료)
//   STBY는 보드에서 3.3V 직결로 추정 → 코드 제어 X
// =============================================================
#define PIN_AIN1  21   // A모터 방향1
#define PIN_AIN2  17   // A모터 방향2
#define PIN_PWMA  25   // A모터 PWM
#define PIN_BIN1  22   // B모터 방향1
#define PIN_BIN2  23   // B모터 방향2
#define PIN_PWMB  26   // B모터 PWM

// 엔코더 핀 (현재는 미사용, 나중에 PID 제어용)
// AENCA=35, AENCB=34, BENCA=27, BENCB=16

// PWM 설정 (ESP32 Arduino core 2.x: ledcSetup + ledcAttachPin)
#define PWM_CH_A     0       // A모터 PWM 채널
#define PWM_CH_B     1       // B모터 PWM 채널
#define PWM_FREQ     20000   // 20kHz (가청 영역 회피)
#define PWM_RES_BITS 8       // 0~255

// 기본 속도 (0~255). 필요시 조절.
const uint8_t SPEED_FWD   = 100;
const uint8_t SPEED_TURN  = 50;   // 완만한 회전 안쪽 바퀴
const uint8_t SPEED_PIVOT = 110;   // 제자리 회전

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
WiFiUDP udpCmd;
WiFiUDP udpTel;

bool wifiConnected = false;
IPAddress pcAddr;          // PC IP 자동 학습용
bool      pcAddrKnown = false;

unsigned long lastCmdMs = 0;

// 현재 명령 (텔레메트리에 같이 실어줌)
char currentCmd = 'S';

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

// A모터(좌): speed > 0 전진, < 0 후진, 0 정지
void setLeft(int16_t speed) {
  uint8_t pwm = (uint8_t)constrain(abs(speed), 0, 255);
  if (speed > 0) {
    digitalWrite(PIN_AIN1, LOW);   // HIGH → LOW
    digitalWrite(PIN_AIN2, HIGH);  // LOW → HIGH
  } else if (speed < 0) {
    digitalWrite(PIN_AIN1, HIGH);  // LOW → HIGH
    digitalWrite(PIN_AIN2, LOW);   // HIGH → LOW
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
  }
  ledcWrite(PWM_CH_A, pwm);
}

void setRight(int16_t speed) {
  uint8_t pwm = (uint8_t)constrain(abs(speed), 0, 255);
  if (speed > 0) {
    digitalWrite(PIN_BIN1, LOW);   // HIGH → LOW
    digitalWrite(PIN_BIN2, HIGH);  // LOW → HIGH
  } else if (speed < 0) {
    digitalWrite(PIN_BIN1, HIGH);  // LOW → HIGH
    digitalWrite(PIN_BIN2, LOW);   // HIGH → LOW
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
  }
  ledcWrite(PWM_CH_B, pwm);
}

void setupMotors() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // ESP32 Arduino core 2.x 의 옛 API
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_PWMA, PWM_CH_A);
  ledcAttachPin(PIN_PWMB, PWM_CH_B);

  motorStop();
}

void applyCommand(char c) {
  switch (c) {
    case 'W':   // 직진
      setLeft( SPEED_FWD);
      setRight(SPEED_FWD);
      break;
    case 'A':   // 완만한 좌회전 (좌 느리게, 우 정상)
      setLeft( SPEED_TURN);
      setRight(SPEED_FWD);
      break;
    case 'D':   // 완만한 우회전
      setLeft( SPEED_FWD);
      setRight(SPEED_TURN);
      break;
    case 'Q':   // 제자리 좌회전 (좌 후진, 우 전진)
      setLeft(-SPEED_PIVOT);
      setRight( SPEED_PIVOT);
      break;
    case 'E':   // 제자리 우회전
      setLeft( SPEED_PIVOT);
      setRight(-SPEED_PIVOT);
      break;
    case 'S':   // 정지
    case 'X':   // 비상정지
    default:
      motorStop();
      c = 'S';
      break;
  }
  currentCmd = c;
}

// =============================================================
//                  I2C / IMU 헬퍼 (imu_test.ino와 동일)
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
  Serial.printf("QMI8658 WHO_AM_I: 0x%02X\n", whoami);
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
  Serial.printf("AK09918 WIA2: 0x%02X\n", wia);
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
  Serial.println(">>> 평평한 곳에 가만히 (3초 후 캘리브레이션)");
  delay(3000);
  Serial.println("Calibrating...");

  float accSum[3] = {0}, gyroSum[3] = {0};
  int count = 0;
  for (int i = 0; i < 200; i++) {
    float acc[3], gyro[3];
    if (qmi8658Read(acc, gyro)) {
      for (int j = 0; j < 3; j++) {
        accSum[j]  += acc[j];
        gyroSum[j] += gyro[j];
      }
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
  Serial.printf("Acc  Offset: %.4f %.4f %.4f\n", accOffset[0], accOffset[1], accOffset[2]);
  Serial.printf("Gyro Offset: %.4f %.4f %.4f\n", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
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
  Serial.println("\n--- WiFi Setup ---");
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
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print("\nWiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed. Starting AP recovery...");
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
    Serial.println("OTA: Start - stopping motors");
    motorStop();   // 안전: OTA 중 모터 정지
  });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA: End"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\r", (p / (t / 100)));
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("OTA Error[%u]\n", err);
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready.");
}

// =============================================================
//                  UDP 명령 수신
// =============================================================
void handleUdpCommand() {
  int packetSize = udpCmd.parsePacket();
  if (packetSize <= 0) return;

  // PC 주소 학습
  pcAddr = udpCmd.remoteIP();
  pcAddrKnown = true;

  char buf[8] = {0};
  int len = udpCmd.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;

  char c = toupper(buf[0]);
  applyCommand(c);
  lastCmdMs = millis();
}

// 워치독: 일정 시간 명령 없으면 자동 정지
void checkWatchdog() {
  if (currentCmd != 'S' && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    Serial.println("[watchdog] no command -> STOP");
    applyCommand('S');
  }
}

// =============================================================
//                  IMU 텔레메트리 송신 (50Hz)
// =============================================================
void sendTelemetry(float roll, float pitch, float yaw,
                   float ax, float ay, float az,
                   float gx, float gy, float gz) {
  if (!wifiConnected) return;
  if (!pcAddrKnown)   return;   // PC가 한 번 명령 보낸 후에야 주소 알 수 있음

  char json[256];
  int n = snprintf(json, sizeof(json),
    "{\"t\":%lu,\"cmd\":\"%c\",\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}",
    millis(), currentCmd, roll, pitch, yaw, ax, ay, az, gx, gy, gz);

  udpTel.beginPacket(pcAddr, TELEMETRY_PORT);
  udpTel.write((const uint8_t*)json, n);
  udpTel.endPacket();
}

// =============================================================
//                  Setup / Loop
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== UGV ESP32 Boot ===");

  setupMotors();
  Serial.println("Motors initialized (stopped).");

  setupWiFi();
  setupOTA();

  if (wifiConnected) {
    udpCmd.begin(CMD_PORT);
    Serial.printf("UDP command listening on %u\n", CMD_PORT);
    Serial.printf("Telemetry will be sent to learned PC IP : %u\n", TELEMETRY_PORT);
  }

  // IMU
  Wire.begin(S_SDA, S_SCL);
  Wire.setClock(400000);
  delay(50);

  if (qmi8658Init() && ak09918Init()) {
    calibrateIMU();
    lastUpdate = micros();
    imuReady = true;
    Serial.println("IMU ready.");
  } else {
    Serial.println("IMU init FAILED - continuing without IMU.");
  }

  Serial.println(">>> Setup complete <<<");
}

void loop() {
  ArduinoOTA.handle();
  handleUdpCommand();
  checkWatchdog();

  // IMU 100Hz 업데이트, 텔레메트리 50Hz 송신
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

  if (millis() - lastTelMs >= 20) {   // 50Hz
    lastTelMs = millis();
    if (imuReady) {
      sendTelemetry(lastRoll, lastPitch, lastYaw,
                    lastAcc[0], lastAcc[1], lastAcc[2],
                    lastGyro[0], lastGyro[1], lastGyro[2]);
    }
  }

  // 살아있다는 신호
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 10000) {
    lastBeat = millis();
    Serial.printf("[alive] mode=%s ip=%s cmd=%c pcKnown=%d\n",
      wifiConnected ? "STA" : "AP",
      wifiConnected ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str(),
      currentCmd, pcAddrKnown);
  }
}
