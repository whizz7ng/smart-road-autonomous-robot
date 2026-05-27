from ultralytics import YOLO

def main():
    
    model = YOLO("runs/detect/best/weights/best.pt")

    # 추론 시작 (테스트 이미지 폴더나 이미지 경로를 source에 입력)
    results = model.predict(
        source="datasets/val/images", # 테스트할 이미지들이 있는 경로
        save=True,                   # 결과 이미지 저장
        conf=0.5,                    # 신뢰도 임계값 (0.5 이상만 표시)
        show=True                    # 결과 화면 표시
    )

if __name__ == "__main__":
    main()