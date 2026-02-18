from ultralytics import YOLO
from pathlib import Path

# 1. 모델 로드
# 입력 파일 이름을 '1112best.pt'로 지정합니다.
model = YOLO('1112best.pt')

# 2. OpenVINO로 모델 변환 (내보내기)
# 이 코드를 실행하면 입력 파일 이름(1112best)을 기반으로
# '1112best_openvino_model' 폴더가 자동으로 생성됩니다.
export_result_path = model.export(
    format='openvino',
    half=True,       # FP16 (반정밀도) 양자화 활성화
)

# 3. 결과 확인 및 출력
output_dir = Path(export_result_path)
print("\nOpenVINO 변환이 성공적으로 완료되었습니다.")
print(f"결과가 '{output_dir}' 폴더에 저장되었습니다.")
print(f"생성된 파일: {output_dir / '1112best.xml'} 과 {output_dir / '1112best.bin'}")
