#pragma once

// ====================================
// 펌웨어 정보
// ====================================
#define FW_NAME    "UGV_ESP32"
#define FW_VERSION "0.1.0"

// ====================================
// 네트워크 설정
// ====================================
#define OTA_HOSTNAME      "ugv-esp32"
#define OTA_PORT          3232

// WiFi 연결 타임아웃 (초)
#define WIFI_CONNECT_TIMEOUT_SEC 15

// 세이프 모드 (AP) 설정
#define AP_SSID           "UGV-Recovery"
#define AP_PASSWORD       "recovery123"

// ====================================
// 시리얼 설정
// ====================================
#define SERIAL_BAUD       115200

// ====================================
// 시스템
// ====================================
#define HEARTBEAT_INTERVAL_MS  10000   // 살아있다는 로그 주기
#define WATCHDOG_TIMEOUT_SEC   30      // 코드 멈추면 자동 재부팅