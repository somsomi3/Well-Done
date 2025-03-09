import numpy as np
import librosa
import sounddevice as sd
from tensorflow.keras.models import load_model
import time
from datetime import datetime, timedelta

# 모델 로드
model = load_model('sleep_sound_model.h5')

# 상태 정의
STATES = ['안자는 상태', '자는 숨소리', '코골이', '수면무호흡']

def preprocess_audio(audio, sr, n_mels=128):
    mel_spec = librosa.feature.melspectrogram(y=audio, sr=sr, n_mels=n_mels)
    mel_spec_db = librosa.power_to_db(mel_spec, ref=np.max)
    mel_spec_db = (mel_spec_db - np.min(mel_spec_db)) / (np.max(mel_spec_db) - np.min(mel_spec_db))
    return mel_spec_db.reshape(1, mel_spec_db.shape[0], mel_spec_db.shape[1], 1)

def predict_state(audio, sr):
    processed_audio = preprocess_audio(audio, sr)
    prediction = model.predict(processed_audio)
    return np.argmax(prediction[0])

def record_audio(duration, samplerate):
    return sd.rec(int(samplerate * duration), samplerate=samplerate, channels=1, dtype='float32')

def analyze_sleep(sleep_duration):
    samplerate = 22050
    interval = 30  # 30초 간격
    sleep_states = []
    
    start_time = datetime.now()
    end_time = start_time + timedelta(hours=sleep_duration)
    
    while datetime.now() < end_time:
        audio = record_audio(interval, samplerate)
        sd.wait()
        
        state_counts = {state: 0 for state in STATES}
        
        for i in range(0, len(audio), samplerate):  # 1초 단위로 분석
            chunk = audio[i:i+samplerate]
            state = predict_state(chunk, samplerate)
            state_counts[STATES[state]] += 1
        
        # 우선순위에 따라 상태 결정
        if state_counts['수면무호흡'] > 0:
            final_state = '수면무호흡'
        elif state_counts['코골이'] > 0:
            final_state = '코골이'
        elif state_counts['자는 숨소리'] > 0:
            final_state = '자는 숨소리'
        else:
            final_state = '안자는 상태'
        
        sleep_states.append(final_state)
        print(f"현재 상태 (최근 30초): {final_state}")
    
    return sleep_states

def summarize_sleep(sleep_states):
    total_intervals = len(sleep_states)
    state_counts = {state: sleep_states.count(state) for state in STATES}
    
    print("\n수면 분석 결과:")
    for state, count in state_counts.items():
        duration = count * 0.5  # 30초 = 0.5분
        percentage = (count / total_intervals) * 100
        print(f"{state}: {duration:.1f}분 ({percentage:.1f}%)")

def analyze_wav_file(file_path, model, interval=30):
    # 오디오 파일 로드
    audio, sr = librosa.load(file_path, sr=22050)
    
    total_duration = len(audio) / sr
    intervals = int(total_duration / interval)
    
    sleep_states = []
    
    for i in range(intervals):
        start = i * interval * sr
        end = (i + 1) * interval * sr
        audio_chunk = audio[start:end]
        
        state_counts = {state: 0 for state in STATES}
        
        for j in range(0, len(audio_chunk), sr):  # 1초 단위로 분석
            chunk = audio_chunk[j:j+sr]
            state = predict_state(chunk, sr)
            state_counts[STATES[state]] += 1
        
        # 우선순위에 따라 상태 결정
        if state_counts['수면무호흡'] > 0:
            final_state = '수면무호흡'
        elif state_counts['코골이'] > 0:
            final_state = '코골이'
        elif state_counts['자는 숨소리'] > 0:
            final_state = '자는 숨소리'
        else:
            final_state = '안자는 상태'
        
        sleep_states.append(final_state)
        print(f"시간: {i*interval}-{(i+1)*interval}초, 상태: {final_state}")
    
    return sleep_states

# 메인 실행
if __name__ == "__main__":
    # 모델 로드
    model = load_model('sleep_sound_model.h5')
    
    analysis_type = input("실시간 분석을 하시겠습니까? 아니면 WAV 파일을 분석하시겠습니까? (실시간/WAV): ").lower()
    
    if analysis_type == '실시간':
        sleep_duration = float(input("수면 시간을 몇 시간으로 설정하시겠습니까? "))
        print(f"{sleep_duration}시간 동안 수면 분석을 시작합니다.")
        sleep_states = analyze_sleep(sleep_duration)
    elif analysis_type == 'wav':
        file_path = input("분석할 WAV 파일의 경로를 입력하세요: ")
        print(f"{file_path} 파일을 분석합니다.")
        sleep_states = analyze_wav_file(file_path, model)
    else:
        print("잘못된 입력입니다. 프로그램을 종료합니다.")
        exit()
    
    summarize_sleep(sleep_states)