from ultralytics import YOLO

def main():
    # 기본 모델 로드
    model = YOLO("yolov8n.pt")

    # 통합 데이터셋 학습 (오탐지 방지를 위한 증강 설정 추가)
    model.train(
        data="data.yaml",
        epochs=100,
        imgsz=640,
        batch=4,
        patience=20,     # 성능 개선이 20 에폭 동안 없으면 조기 종료
        workers=2,
        project=".",     # 현재 폴더를 프로젝트 루트로
        name="best",     # ./best 폴더에 결과 저장
        
        # 오탐지 방지를 위한 고급 설정
        mosaic=1.0,      # 주변 환경 맥락 학습 (기본값)
        mixup=0.2,       # 두 이미지를 섞어 형태 변별력 향상
        fliplr=0.5,      # 좌우 반전
        degrees=10.0     # 약간의 회전으로 각도 변화 대응
    )

if __name__ == "__main__":
    main()