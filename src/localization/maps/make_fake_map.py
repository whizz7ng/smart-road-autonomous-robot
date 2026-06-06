#!/usr/bin/env python3
"""
make_fake_map - AMCL 테스트용 가짜 맵 생성
  5m x 5m 정사각형 방 (외곽 벽). 해상도 0.05 m/pixel = 100x100 pixels.
  출력: fake_map.pgm + fake_map.yaml

사용법:
  python3 make_fake_map.py
"""
import struct

# ===== 맵 사양 =====
RES = 0.05          # m/pixel
WIDTH_M = 5.0       # 가로 5m
HEIGHT_M = 5.0      # 세로 5m
WALL_PX = 3         # 벽 두께 (픽셀)
ORIGIN = (-2.5, -2.5, 0.0)   # 맵 좌하단의 월드 좌표

W = int(WIDTH_M / RES)   # 100
H = int(HEIGHT_M / RES)  # 100

# 픽셀 값: 254=free(흰색), 0=occupied(검은색), 205=unknown(회색)
FREE = 254
OCC = 0

# ===== PGM 데이터 생성 =====
pixels = bytearray()
for y in range(H):
    for x in range(W):
        if (x < WALL_PX or x >= W - WALL_PX
                or y < WALL_PX or y >= H - WALL_PX):
            pixels.append(OCC)
        else:
            pixels.append(FREE)

# ===== PGM 파일 쓰기 (binary P5) =====
with open("fake_map.pgm", "wb") as f:
    header = f"P5\n{W} {H}\n255\n".encode()
    f.write(header)
    f.write(bytes(pixels))

# ===== YAML 파일 쓰기 =====
yaml_content = f"""image: fake_map.pgm
mode: trinary
resolution: {RES}
origin: [{ORIGIN[0]}, {ORIGIN[1]}, {ORIGIN[2]}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
with open("fake_map.yaml", "w") as f:
    f.write(yaml_content)

print(f"생성 완료:")
print(f"  fake_map.pgm  ({W} x {H} pixels)")
print(f"  fake_map.yaml")
print(f"  실제 크기: {WIDTH_M}m x {HEIGHT_M}m, 해상도 {RES}m/px")
print(f"  원점(좌하단): {ORIGIN}")
