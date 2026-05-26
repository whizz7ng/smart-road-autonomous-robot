# ESP32 Firmware

라즈베리파이4와 함께 동작하는 ESP32 펌웨어. WiFi OTA로 업로드합니다.

## 보드
- ESP32-WROOM-32 기반
- 라즈베리파이 위에 40핀 헤더로 스택

## 개발 환경 셋업 (한 번만)

### Prerequisites
- WSL2 Ubuntu 24.04+ (Windows 11)
- PlatformIO Core (`pipx install platformio`)

### 1. WSL mirrored networking 활성화
`%USERPROFILE%\.wslconfig` (Windows 측):

```ini
[wsl2]
networkingMode=mirrored

[experimental]
hostAddressLoopback=true
```

설정 후 `wsl --shutdown` (PowerShell에서).

### 2. Windows 방화벽 규칙 추가
PowerShell 관리자 권한으로:

```powershell
New-NetFirewallRule -DisplayName "ESP32 OTA Host Port" -Direction Inbound -Protocol UDP -LocalPort 48266 -Action Allow -Profile Any
```

### 3. Secrets 파일 작성
```bash
cp include/secrets.example.h include/secrets.h
# include/secrets.h 열어서 WiFi SSID/비밀번호 입력
```

## 빌드 & 업로드

### OTA (평소)
```bash
cd esp32_firmware
pio run -e esp32dev_ota -t upload --upload-port <ESP32_IP>
```

ESP32의 IP는 시리얼 모니터 부팅 로그에서 확인하거나, 공유기 관리 페이지에서 확인.

### USB (OTA 망가졌을 때 복구용)
보드의 Type-C 포트에 케이블 연결 후, Windows에서:
```powershell
cd Z:\home\<user>\smart-road-autonomous-robot\esp32_firmware
C:\Users\<user>\.platformio\penv\Scripts\platformio.exe run -e esp32dev_usb -t upload
```

## 세이프 모드
WiFi 연결 실패 시 ESP32가 자동으로 AP 모드 진입:
- SSID: `UGV-Recovery`
- Password: `recovery123`
- AP IP: `192.168.4.1`

이 상태에서도 OTA 업로드 가능 (PC를 UGV-Recovery에 연결 후 `192.168.4.1`로).
