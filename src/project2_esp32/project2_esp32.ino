#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

////////////////////////////////////////////////////////////////////////////////
// ====== wifi 설정 (여기만 수정하면 됨) ======
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
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// TB6612 모터 드라이버 핀(AIN1, AIN2, APWM, AENCA, AENCB, BIN1, BIN2, BPWM, BENCA, BENCB)
const uint8_t AIN1 = 21;    
const uint8_t AIN2 = 17;    
const uint8_t PWMA = 25;    

const uint8_t BIN1 = 22;    
const uint8_t BIN2 = 23;    
const uint8_t PWMB = 26;    

// 엔코더 핀
const uint16_t AENCA = 35;  // A 방향 판단
const uint16_t AENCB = 34;  // A 인터럽트 (RISING)
const uint16_t BENCA = 27;  // B 방향 판단
const uint16_t BENCB = 16;  // B 인터럽트 (RISING)
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// 전역 변수.
// PWM 설정
const uint32_t PWM_FREQ = 20000;          // 20kHz
const uint8_t PWM_RESOLUTION = 8;         // 8bit (0-255)
const int MAX_PWM_STEP = 2;               // 한 번에 변화할 수 있는 PWM 최대 증감 폭

// 가속도 제어
long target_speed_pwm = 0;                // 사용자 지정. 목표 pwm 값
long apply_speed_pwm = 0;                 // 마지막으로 기준이 된 pwm 값. 여기에 PID, L/R 조정값이 반영되어 최종값이 된다.
unsigned long last_accel_time = 0;        // 마지막으로 계단 속도를 올린 시간
const unsigned long ACCEL_INTERVAL = 100; // 속도 갱신 주기 (ms)

// 운동 상태. 
bool is_running = false;
enum StopState {
  STOP_IDLE,
  STOP_DECEL,
  STOP_BRAKE,
  STOP_COMPLETE
  // STATE_STOP,
  // STATE_RUN,
  // STATE_ACCEL,
  // STATE_DECEL,

};
StopState stop_state = STOP_COMPLETE; 
unsigned long stop_start_time = 0;

// 엔코더 카운트 변수
volatile long A_wheel_pulse_count = 0;
volatile long B_wheel_pulse_count = 0;
long A_wheel_pulse_last = 0;
long B_wheel_pulse_last = 0;

////////////////////////////////////////////////////////////////////////////////
// 함수 선언
void simpleMotorTest(int testPWM);
void simpleMotorTest2(int testPWM);
void smoothStop();
void setMotorSpeed(char motor, int speed);
void setupWiFi();
void setupOTA();
void PrintIMUdata(EulerAngles stAngles, IMU_ST_SENSOR_DATA_FLOAT stGyroRawData, IMU_ST_SENSOR_DATA_FLOAT stAccelRawData, IMU_ST_SENSOR_DATA stMagnRawData);
////////////////////////////////////////////////////////////////////////////////

// 엔코더 카운트 - 인터럽트 서비스 루틴
void IRAM_ATTR encoderA_ISR() {
  if (digitalRead(AENCA) == HIGH) {
    A_wheel_pulse_count++;
  } else {
    A_wheel_pulse_count--;
  }
}

void IRAM_ATTR encoderB_ISR() {
  if (digitalRead(BENCA) == HIGH) {
    B_wheel_pulse_count++;
  } else {
    B_wheel_pulse_count--;
  }
}


// ============================================
// setup() 함수
// ============================================
void setup() {
  Serial.begin(115200);
  while(!Serial) {}  
  delay(500);
  Serial.println("\n=== ESP32 setup Function ===");

  // wifi setup
  setupWiFi();
  setupOTA();
  
  // 모터 핀 출력 설정
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
  
  pinMode(AENCA, INPUT_PULLUP);
  pinMode(AENCB, INPUT_PULLUP);
  pinMode(BENCA, INPUT_PULLUP);
  pinMode(BENCB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(AENCB), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(BENCB), encoderB_ISR, RISING);

  // imu init
  imuInit();
  
  Serial.println("\n=== ESP32 setup Function Complete ===");
}

// ============================================
// loop() 메인 루프
// ============================================
void loop() {
  ArduinoOTA.handle();

  // 살아있다는 확인용 heartbeat (10초마다)
  // static unsigned long lastBeat = 0;
  // if (millis() - lastBeat > 10000) {
  //   lastBeat = millis();
  //   Serial.print("[alive] mode=");
  //   Serial.print(wifiConnected ? "STA" : "AP");
  //   Serial.print(" ip=");
  //   Serial.println(wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString());
  // } temperary disable
  
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case '0':
        target_speed_pwm = 20; 
        simpleMotorTest2(target_speed_pwm);
        break;
      case '1':
        Serial.println("[명령] 1단 주행 - PWM 50");
        target_speed_pwm = 50; 
        is_running = true;
        simpleMotorTest(target_speed_pwm);
        break;    

      case '2':
        Serial.println("[명령] 1단 주행 - PWM 100");
        target_speed_pwm = 100; 
        is_running = true;
        simpleMotorTest(target_speed_pwm);
        break;   

      case '3':
        Serial.println("[명령] 1단 주행 - PWM 150");
        target_speed_pwm = 150; 
        is_running = true;
        simpleMotorTest(target_speed_pwm);
        break;   

      case '4':
        Serial.println("[명령] 1단 주행 - PWM 200");
        target_speed_pwm = 200; 
        is_running = true;
        simpleMotorTest(target_speed_pwm);
        break;  

      case '5':
        Serial.println("[명령] 1단 주행 - PWM 250");
        target_speed_pwm = 250; 
        is_running = true;
        simpleMotorTest(target_speed_pwm);
        break;  
        
      case 'x': // stop
        if (is_running) {
          Serial.println("[명령] 감속 정지 - PWM 0");
          target_speed_pwm = 0; 
          is_running = false;
          stop_state = STOP_IDLE;
        }
        break;

      case 'w':
        target_speed_pwm = 80; 
        setMotorSpeed('A', target_speed_pwm);
        setMotorSpeed('B', target_speed_pwm);
        is_running = true;
        break;
        
      case 'a' : // 50 : 75 pwm turn
        target_speed_pwm = 80; 
        setMotorSpeed('A', target_speed_pwm);
        setMotorSpeed('B', target_speed_pwm + 40);
        is_running = true;
        break;

        
      case 's' : // 50 pwm back
        target_speed_pwm = -80; 
        setMotorSpeed('A', target_speed_pwm);
        setMotorSpeed('B', target_speed_pwm);
        is_running = true;
        break;
        
      case 'd' : // 75 : 50 pwm turn
        target_speed_pwm = 80; 
        setMotorSpeed('A', target_speed_pwm + 40);
        setMotorSpeed('B', target_speed_pwm);
        is_running = true;
        break;
        
      case 'm':
        simpleMotorTest(125);
        break;
    }
  }
  EulerAngles stAngles;
  IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
  IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  PrintIMUdata(stAngles, stGyroRawData, stAccelRawData, stMagnRawData);

  
  // 가속도 프로파일 제어 및 주행 제어
  if (is_running) {
  //   bool isBack = false;
  //   if (target_speed_pwm < 0)
  //     isBack = true;

  //   if (apply_speed_pwm < target_speed_pwm) {
  //     if (millis() - last_accel_time >= ACCEL_INTERVAL) {
  //       apply_speed_pwm += MAX_PWM_STEP;
  //       if (apply_speed_pwm > target_speed_pwm) {
  //         apply_speed_pwm = target_speed_pwm; 
  //       }
  //       last_accel_time = millis();
  //     }
  //   }
  //   moveForwardPID(apply_speed_pwm);
  } else if (stop_state != STOP_COMPLETE) {
    smoothStop();
  }
  
  delay(20);
}

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

// 상태 머신(State Machine) 기반의 부드러운 감속 및 급브레이크 정지 함수
void smoothStop() {
  switch (stop_state) {
    case STOP_IDLE:
      stop_start_time = millis();
      stop_state = STOP_DECEL;
      Serial.println("[정지상태] 감속 단계 진입");
      break;
      
    case STOP_DECEL:
      // 주기적으로 속도를 줄임
      if (millis() - last_accel_time >= ACCEL_INTERVAL) {
        apply_speed_pwm -= (MAX_PWM_STEP * 2); // 정지할 때는 가속보다 조금 더 빠르게 감속
        if (apply_speed_pwm <= 0) {
          apply_speed_pwm = 0;
          stop_state = STOP_BRAKE;
          stop_start_time = millis(); // 브레이크 시작 시간 초기화
          Serial.println("[정지상태] 쇼트 브레이크 단계 진입");
        }
        // moveForwardPID(apply_speed_pwm);
        last_accel_time = millis();
      }
      break;
      
    case STOP_BRAKE:
      // TB6612의 양방향 핀을 모두 HIGH로 만들어 모터를 전기적으로 lock (쇼트 브레이크)
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, HIGH);
      ledcWrite(PWMA, 255);
      ledcWrite(PWMB, 255);
      
      // 완전히 멈출 수 있도록 200ms 동안 브레이크 유지
      if (millis() - stop_start_time >= 200) {
        stop_state = STOP_COMPLETE;
        Serial.println("[정지상태] 정지 완료 (IDLE)");
      }
      break;
      
    case STOP_COMPLETE:
      // 완전히 정지되어 대기하는 상태 (아무것도 하지 않음, 모터 전류 해제)
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      ledcWrite(PWMA, 0);
      ledcWrite(PWMB, 0);
      
      // PID 변수 리셋 (다음 주행을 위해)
      // error_integral = 0;
      // prev_error = 0;
      A_wheel_pulse_count = 0;
      B_wheel_pulse_count = 0;
      break;
  }
}

// 하드웨어 점검용 단순 모터 구동 테스트 함수
void simpleMotorTest(int testPWM = 20) {

  Serial.println("[테스트] 3초 구동 후 정지. 과정 중 증가된 엔코더 카운트 출력");

  // =========================
  // 시작 카운트 저장
  // =========================
  noInterrupts();

  long start_A = A_wheel_pulse_count;
  long start_B = B_wheel_pulse_count;

  interrupts();

  // =========================
  // 모터 구동
  // =========================
  setMotorSpeed('A', testPWM);
  setMotorSpeed('B', testPWM);

  delay(3000);

  // =========================
  // 브레이크
  // =========================
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);

  delay(200);

  // =========================
  // 완전 정지
  // =========================
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  // =========================
  // 종료 카운트 읽기
  // =========================
  noInterrupts();

  long end_A = A_wheel_pulse_count;
  long end_B = B_wheel_pulse_count;

  interrupts();

  // =========================
  // delta 계산
  // =========================
  long delta_A = end_A - start_A;
  long delta_B = end_B - start_B;

  Serial.print("Test PWM : ");
  Serial.print(testPWM);

  Serial.print("    A_delta : ");
  Serial.print(delta_A);

  Serial.print("    B_delta : ");
  Serial.println(delta_B);

  Serial.println("[테스트] 종료");
}

void simpleMotorTest2(int testPWM) {
  Serial.println("[테스트] 2초간 전진 후 1초간 브레이크 테스트");
  
  // 기준점 초기화
  A_wheel_pulse_last = A_wheel_pulse_count;
  B_wheel_pulse_last = B_wheel_pulse_count;

  // 전진 하이 레벨 제어
  setMotorSpeed('A', testPWM);
  setMotorSpeed('B', testPWM);
  
  delay(1000);
  
    noInterrupts();     // 잠시 인터럽트를 멈추고 변수 값을 안전하게 스냅샷 복사
    long current_A = A_wheel_pulse_count;
    long current_B = B_wheel_pulse_count;
    interrupts();
    long delta_A = current_A - A_wheel_pulse_last;
    long delta_B = current_B - B_wheel_pulse_last;
  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

    Serial.print("Test PWM value : ");       Serial.print(testPWM);
    Serial.print("    A_pulse : ");          Serial.print(current_A); 
    Serial.print("    A_delta : ");          Serial.print(delta_A); 
    Serial.print("    |    B_pulse : ");     Serial.print(current_B); 
    Serial.print("    B_delta : ");          Serial.println(delta_B);
}

// 모터 개별 방향 및 속도 제어 헬퍼 함수
void setMotorSpeed(char motor, int speed) {
  if (motor == 'A') {
    if (speed > 0) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    } else if (speed < 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    } else {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
    }
    ledcWrite(PWMA, abs(speed));
  } 
  else if (motor == 'B') {
    if (speed > 0) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    } else if (speed < 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    } else {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }
    ledcWrite(PWMB, abs(speed));
  }
}

void PrintIMUdata(EulerAngles stAngles, IMU_ST_SENSOR_DATA_FLOAT stGyroRawData, IMU_ST_SENSOR_DATA_FLOAT stAccelRawData, IMU_ST_SENSOR_DATA stMagnRawData)
{
	// ==========================
	// Angle 출력
	// ==========================
	Serial.print("Angle Roll : ");
	Serial.print(stAngles.roll);

	Serial.print("    Pitch : ");
	Serial.print(stAngles.pitch);

	Serial.print("    Yaw : ");
	Serial.print(stAngles.yaw);

	// ==========================
	// Gyro 출력
	// ==========================
	Serial.print("    Gyro X : ");
	Serial.print(stGyroRawData.X);

	Serial.print("    Gyro Y : ");
	Serial.print(stGyroRawData.Y);

	Serial.print("    Gyro Z : ");
	Serial.println(stGyroRawData.Z);

	// ==========================
	// Accel 출력
	// ==========================
	Serial.print("    Accel X : ");
	Serial.print(stAccelRawData.X);

	Serial.print("    Accel Y : ");
	Serial.print(stAccelRawData.Y);

	Serial.print("    Accel Z : ");
	Serial.print(stAccelRawData.Z);

	// ==========================
	// Magnetometer 출력
	// ==========================
	Serial.print("    Magn X : ");
	Serial.print(stMagnRawData.s16X);

	Serial.print("    Magn Y : ");
	Serial.print(stMagnRawData.s16Y);

	Serial.print("    Magn Z : ");
	Serial.println(stMagnRawData.s16Z);
}
