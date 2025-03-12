import numpy as np
import librosa
from tensorflow.keras.models import load_model

def preprocess_audio(audio_path, n_mels=128, duration=5):
    audio, sr = librosa.load(audio_path, duration=duration, sr=22050)
    mel_spec = librosa.feature.melspectrogram(y=audio, sr=sr, n_mels=n_mels)
    mel_spec_db = librosa.power_to_db(mel_spec, ref=np.max)
    mel_spec_db = (mel_spec_db - np.min(mel_spec_db)) / (np.max(mel_spec_db) - np.min(mel_spec_db))
    return mel_spec_db.reshape(1, mel_spec_db.shape[0], mel_spec_db.shape[1], 1)

def predict_sleep_state(model, audio_path, class_names):
    preprocessed_audio = preprocess_audio(audio_path)
    prediction = model.predict(preprocessed_audio)
    predicted_class_index = np.argmax(prediction[0])
    predicted_class = class_names[predicted_class_index]
    confidence = prediction[0][predicted_class_index]
    return predicted_class, confidence

# 저장된 모델 불러오기
model = load_model('sleep_sound_model.h5')

# 클래스 이름 정의 (학습 시 사용한 순서대로)
class_names = ['안자는 상태', '자는 숨소리', '코골이', '수면무호흡']

# 새로운 오디오 파일에 대한 예측
new_audio_path = 'path/to/new/audio/file.wav'  # 새로운 오디오 파일 경로
predicted_class, confidence = predict_sleep_state(model, new_audio_path, class_names)

print(f"예측된 수면 상태: {predicted_class}")
print(f"신뢰도: {confidence:.2f}")