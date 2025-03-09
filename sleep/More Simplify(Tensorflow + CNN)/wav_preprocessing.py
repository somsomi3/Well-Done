import os
import numpy as np
import librosa
import matplotlib.pyplot as plt
from tensorflow.keras.utils import to_categorical

def load_and_preprocess_data(data_dir, n_mels=128, duration=5):
    X = []
    y = []
    class_names = os.listdir(data_dir)
    
    for class_idx, class_name in enumerate(class_names):
        class_dir = os.path.join(data_dir, class_name)
        for audio_file in os.listdir(class_dir):
            audio_path = os.path.join(class_dir, audio_file)
            
            # 오디오 로드 및 리샘플링
            audio, sr = librosa.load(audio_path, duration=duration, sr=22050)
            
            # 멜 스펙트로그램 생성
            mel_spec = librosa.feature.melspectrogram(y=audio, sr=sr, n_mels=n_mels)
            mel_spec_db = librosa.power_to_db(mel_spec, ref=np.max)
            
            # 정규화
            mel_spec_db = (mel_spec_db - np.min(mel_spec_db)) / (np.max(mel_spec_db) - np.min(mel_spec_db))
            
            X.append(mel_spec_db)
            y.append(class_idx)
    
    X = np.array(X)
    y = np.array(y)
    
    # 차원 추가 (채널 차원)
    X = X.reshape(X.shape + (1,))
    
    # 레이블 원-핫 인코딩
    y = to_categorical(y)
    
    return X, y, class_names

# 데이터 불러오기 및 전처리
data_dir = "path/to/your/audio/data"  # 오디오 데이터가 있는 디렉토리 경로로 변경하세요
X, y, class_names = load_and_preprocess_data(data_dir)

# 데이터 형태 확인
print("X shape:", X.shape)
print("y shape:", y.shape)
print("Classes:", class_names)

# # 스펙트로그램 시각화 예시
# plt.figure(figsize=(10, 4))
# librosa.display.specshow(X[0, :, :, 0], y_axis='mel', x_axis='time')
# plt.colorbar(format='%+2.0f dB')
# plt.title('Mel spectrogram')
# plt.tight_layout()
# plt.show()

# 데이터 분할 (학습 데이터와 검증 데이터)
from sklearn.model_selection import train_test_split

X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)

# CNN 모델 생성
from tensorflow.keras import layers, models

def create_model(input_shape, num_classes):
    model = models.Sequential([
        layers.Conv2D(32, (3, 3), activation='relu', input_shape=input_shape),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.Flatten(),
        layers.Dense(64, activation='relu'),
        layers.Dense(num_classes, activation='softmax')
    ])
    return model

input_shape = X_train.shape[1:]
num_classes = len(class_names)

model = create_model(input_shape, num_classes)

# 모델 컴파일
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

# 모델 요약 출력
model.summary()

# 모델 학습
history = model.fit(X_train, y_train, 
                    epochs=20, 
                    batch_size=32, 
                    validation_data=(X_val, y_val))

# 학습 결과 시각화
plt.figure(figsize=(12, 4))

plt.subplot(1, 2, 1)
plt.plot(history.history['accuracy'], label='Training Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.title('Model Accuracy')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Model Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()

plt.tight_layout()
plt.show()

# 모델 저장
model.save('sleep_sound_model.h5')

print("모델이 'sleep_sound_model.h5'로 저장되었습니다.")