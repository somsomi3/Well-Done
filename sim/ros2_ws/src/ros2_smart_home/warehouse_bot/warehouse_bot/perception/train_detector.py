# train_ssd.py
import os
import tensorflow as tf
from object_detection import model_lib_v2
from datetime import datetime

# GPU 할당 (3번 GPU 사용)
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = "3"

# GPU 설정
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        # 메모리 증가 설정 (OOM 방지)
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        print(f"GPU 할당 성공: {gpus}")
    except RuntimeError as e:
        print(f"GPU 설정 오류: {e}")
else:
    print("GPU를 찾을 수 없습니다!")

# 경로 설정
pipeline_config_path = "pipeline.config"  # 같은 폴더에 있음
model_dir = "trained_model"  # 체크포인트 저장 폴더

# 현재 시간 기록 (로그 파일명용)
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
print(f"학습 시작 시간: {current_time}")

# 모델 학습
# train_steps를 지정하지 않으면 pipeline.config에 정의된 값 사용
model_lib_v2.train_loop(
    pipeline_config_path=pipeline_config_path,
    model_dir=model_dir,
    use_tpu=False,
    checkpoint_every_n=1000  # 1000 스텝마다 체크포인트 저장
)

print(f"학습 완료: {datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}")
