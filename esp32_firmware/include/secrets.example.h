#pragma once

// ====================================
// 비밀 정보 템플릿
// ====================================
// 사용법:
//   1. 이 파일을 secrets.h로 복사하세요.
//      Windows: copy secrets.example.h secrets.h
//      Linux/Mac: cp secrets.example.h secrets.h
//   2. secrets.h를 본인 환경에 맞게 수정하세요.
//   3. secrets.h는 Git에 올라가지 않습니다 (.gitignore 처리됨).
// ====================================

// WiFi
#define WIFI_SSID         "YOUR_WIFI_SSID"
#define WIFI_PASSWORD     "YOUR_WIFI_PASSWORD"

// OTA 업로드 비밀번호
#define OTA_PASSWORD      "YOUR_OTA_PASSWORD"